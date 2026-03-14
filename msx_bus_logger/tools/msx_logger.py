#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
msx_logger.py - MSXバスロガー バイナリ受信スクリプト

使用方法:
    python msx_logger.py [--port COM8] [--output FILE] [--warn-drop] [--save-raw]

オプション:
    --port      COMポート指定（デフォルト: COM8）
    --output    CSVファイル名（デフォルト: msx_log_YYYYMMDD_HHMMSS.csv）
    --warn-drop DROP警告をコンソールに表示する（デフォルト: OFF）
    --save-raw  バイナリをそのまま .bin ファイルにも保存する（デフォルト: OFF）

依存ライブラリ:
    pip install pyserial

Picoのバイナリ転送モード（TEXT_MODE=0）と組み合わせて使用する。

パケットフォーマット（1イベント = 12バイト固定長）:
    Byte 0   : マジックバイト1 (0xAA)
    Byte 1   : マジックバイト2 (0x55)
    Byte 2   : FLAGS
    Byte 3   : DROP_SEQ
    Byte 4-5 : ADDR (little-endian, A0-A15)
    Byte 6   : DATA (D0-D7)
    Byte 7   : 予約 (0x00)
    Byte 8-11: タイムスタンプ (uint32_t, little-endian, μs)

FLAGSのビット定義:
    bit0: RD（/RD がアサート）
    bit1: WR（/WR がアサート）
    bit2: IO（/IORQ がアサート）
    bit4: DROP_LATCHED
    bit6: DMA_LAG

IOアドレス定義:
    VDP (TMS9918/V9938/V9958):
        0x98: データポート RD/WR
        0x99: コントロールポート WR
        0x9A: パレットポート WR (V9938以降)
        0x9B: インダイレクトレジスタ WR (V9938以降)
    PSG (AY-3-8910):
        0xA0: レジスタ番号 WR
        0xA1: データ WR
        0xA2: データ RD
    OPLL (YM2413 / MSX-Music):
        0x7C: レジスタ番号 WR
        0x7D: データ WR
        0x7E: ステータス RD（稀）
    MSX-Audio (Y8950):
        0xC0: レジスタ番号 WR (ch1)
        0xC1: データ WR/RD (ch1)
        0xC2: レジスタ番号 WR (ch2)
        0xC3: データ WR (ch2)
    SCC / SCC-I (Konami):
        メモリマップドのため IO では捕捉不可（MEM_WR で対応予定）
"""

import argparse
import csv
import struct
import sys
import time
from datetime import datetime

import serial

# ─────────────────────────────────────────
# パケット定義
# ─────────────────────────────────────────
MAGIC       = bytes([0xAA, 0x55])
PACKET_SIZE = 12

# struct フォーマット: '<2sBBHBBI'
# (magic:2s, flags:B, drop_seq:B, addr:H, data:B, reserved:B, timestamp:I)
FMT = '<2sBBHBBI'

# FLAGSビット定義
FLAG_RD           = 1 << 0  # /RD アサート
FLAG_WR           = 1 << 1  # /WR アサート
FLAG_IO           = 1 << 2  # /IORQ アサート
FLAG_DROP_LATCHED = 1 << 4  # DROP ラッチフラグ
FLAG_DMA_LAG      = 1 << 6  # DMA 遅延フラグ

# ─────────────────────────────────────────
# IOアドレスセット定義（下位8ビットで照合）
# ─────────────────────────────────────────
VDP_ADDRS       = frozenset([0x98, 0x99, 0x9A, 0x9B])  # VDP (TMS9918/V9938/V9958)
PSG_ADDRS       = frozenset([0xA0, 0xA1, 0xA2])         # PSG (AY-3-8910)
OPLL_WR_ADDRS   = frozenset([0x7C, 0x7D])               # OPLL WR (YM2413)
OPLL_RD_ADDRS   = frozenset([0x7E])                     # OPLL RD (ステータス)
MSXAUDIO_ADDRS  = frozenset([0xC0, 0xC1, 0xC2, 0xC3])  # MSX-Audio (Y8950)

BYTES_PER_EVENT = 12


def parse_args():
    """コマンドライン引数を解析する"""
    parser = argparse.ArgumentParser(
        description='MSXバスロガー バイナリ受信スクリプト',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  python msx_logger.py --port COM8
  python msx_logger.py --port /dev/ttyACM0 --warn-drop --save-raw
  python msx_logger.py --port COM8 --output my_log.csv
        """
    )
    parser.add_argument('--port',      default='COM8',      help='COMポート指定（デフォルト: COM8）')
    parser.add_argument('--output',    default=None,        help='CSVファイル名（デフォルト: 自動生成）')
    parser.add_argument('--warn-drop', action='store_true', help='DROP警告をコンソールに表示する')
    parser.add_argument('--save-raw',  action='store_true', help='バイナリを .bin にも保存する')
    return parser.parse_args()


def read_exact(ser, n):
    """シリアルポートから正確に n バイト読み込む"""
    buf = b''
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise IOError('シリアルポートから読み込めませんでした（タイムアウト）')
        buf += chunk
    return buf


def sync_to_magic(ser, raw_file=None):
    """0xAA 0x55 を探してフレーム同期する。同期完了で True を返す。"""
    b = ser.read(1)
    if not b:
        return False
    if raw_file:
        raw_file.write(b)

    while True:
        if b[0] == 0xAA:
            b2 = ser.read(1)
            if not b2:
                return False
            if raw_file:
                raw_file.write(b2)
            if b2[0] == 0x55:
                return True
            b = b2
            continue
        b = ser.read(1)
        if not b:
            return False
        if raw_file:
            raw_file.write(b)


def fmt_num(n):
    """整数をカンマ区切りでフォーマットする"""
    return f'{n:,}'


def fmt_rate(r):
    """レート値を右揃え・カンマ区切りでフォーマットする"""
    return f'{r:>10,.1f}'


def receive_loop(ser, csv_writer, raw_file, warn_drop, csv_filename, raw_filename):
    """メイン受信ループ。Ctrl+C で終了しサマリを表示する。"""

    # ── 集計カウンタ ──────────────────────────
    seq                = 0
    count_io           = 0
    count_vdp_rd       = 0
    count_vdp_wr       = 0
    count_psg_rd       = 0
    count_psg_wr       = 0
    count_opll_rd      = 0
    count_opll_wr      = 0
    count_msxaudio_rd  = 0
    count_msxaudio_wr  = 0
    count_drop_flag    = 0
    last_drop_seq      = 0

    # ── peak 計測（PC時刻ベース 1秒ウィンドウ）──
    window_start    = time.time()
    window_count    = 0
    window_bytes    = 0
    peak_rate       = 0.0   # events/sec
    peak_throughput = 0.0   # KB/sec

    start_time = time.time()

    try:
        while True:
            if not sync_to_magic(ser, raw_file):
                continue

            rest = read_exact(ser, PACKET_SIZE - 2)
            if raw_file:
                raw_file.write(rest)

            packet = MAGIC + rest
            try:
                magic, flags, drop_seq, addr, data, _reserved, timestamp = \
                    struct.unpack(FMT, packet)
            except struct.error:
                continue

            # ── FLAGSビット展開 ──────────────────
            rd           = bool(flags & FLAG_RD)
            wr           = bool(flags & FLAG_WR)
            io           = bool(flags & FLAG_IO)
            drop_latched = bool(flags & FLAG_DROP_LATCHED)
            dma_lag      = bool(flags & FLAG_DMA_LAG)

            # ── デバイス別カウント ────────────────
            addr_lo = addr & 0xFF

            if io:
                count_io += 1
                if addr_lo in VDP_ADDRS:
                    if rd:
                        count_vdp_rd += 1
                    else:
                        count_vdp_wr += 1
                elif addr_lo in PSG_ADDRS:
                    if rd:
                        count_psg_rd += 1
                    else:
                        count_psg_wr += 1
                elif addr_lo in OPLL_WR_ADDRS:
                    count_opll_wr += 1
                elif addr_lo in OPLL_RD_ADDRS:
                    count_opll_rd += 1
                elif addr_lo in MSXAUDIO_ADDRS:
                    if rd:
                        count_msxaudio_rd += 1
                    else:
                        count_msxaudio_wr += 1

            if drop_latched:
                count_drop_flag += 1
            last_drop_seq = drop_seq

            # ── peak 更新（1秒ウィンドウ）──────────
            now = time.time()
            window_count += 1
            window_bytes += BYTES_PER_EVENT
            elapsed_win = now - window_start
            if elapsed_win >= 1.0:
                r  = window_count / elapsed_win
                tp = (window_bytes / 1024.0) / elapsed_win
                if r  > peak_rate:       peak_rate       = r
                if tp > peak_throughput: peak_throughput = tp
                window_start = now
                window_count = 0
                window_bytes = 0

            # ── DROP 警告 ─────────────────────────
            if warn_drop and drop_latched:
                print(
                    f'[!DROP] seq={seq} drop_seq={drop_seq} @ {timestamp}us',
                    file=sys.stderr
                )

            # ── CSV書き込み ──────────────────────
            csv_writer.writerow([
                seq,
                timestamp,
                f'0x{addr:04X}',
                addr,
                f'0x{data:02X}',
                data,
                f'0x{flags:02X}',
                int(rd),
                int(wr),
                int(io),
                int(drop_latched),
                int(dma_lag),
                drop_seq,
            ])

            seq += 1

    except KeyboardInterrupt:
        elapsed  = time.time() - start_time
        avg_rate = seq / elapsed if elapsed > 0 else 0.0
        avg_tp   = (seq * BYTES_PER_EVENT / 1024.0) / elapsed if elapsed > 0 else 0.0

        # ── サマリ表示 ────────────────────────────
        W_LABEL = 25

        def row(label, value, indent=0):
            pad = '  ' * indent
            lbl = f'{pad}{label}'
            return f'{lbl:<{W_LABEL}}: {value:>12}'

        print()
        print('=== Capture Summary ===')
        print(row('Total Events',             fmt_num(seq)))
        print(row('IO Access',                fmt_num(count_io),          indent=1))
        print(row('VDP RD (0x98-0x9B)',       fmt_num(count_vdp_rd),      indent=2))
        print(row('VDP WR (0x98-0x9B)',       fmt_num(count_vdp_wr),      indent=2))
        print(row('PSG RD (0xA2)',            fmt_num(count_psg_rd),      indent=2))
        print(row('PSG WR (0xA0-0xA1)',       fmt_num(count_psg_wr),      indent=2))
        print(row('OPLL RD (0x7E)',           fmt_num(count_opll_rd),     indent=2))
        print(row('OPLL WR (0x7C-0x7D)',      fmt_num(count_opll_wr),     indent=2))
        print(row('MSX-Audio RD (0xC1)',      fmt_num(count_msxaudio_rd), indent=2))
        print(row('MSX-Audio WR (0xC0-0xC3)', fmt_num(count_msxaudio_wr), indent=2))
        print(row('SCC/SCC-I',               '(MEM_WR)',                  indent=2))
        print(row('DROP Latch',              fmt_num(count_drop_flag)))
        print(row('DROP Sequence',           fmt_num(last_drop_seq)))
        print(row('Elapsed Time',            f'{elapsed:>10.1f} sec'))
        print(row('Bus Event Rate',          f'{fmt_rate(avg_rate)} events/sec  Avg'))
        print(row('',                        f'{fmt_rate(peak_rate)} events/sec  Max'))
        print(row('Throughput',              f'{fmt_rate(avg_tp)} KB/sec     Avg'))
        print(row('',                        f'{fmt_rate(peak_throughput)} KB/sec     Max'))
        print(row('Output (CSV)',            csv_filename))
        if raw_filename:
            print(row('Output (RAW)',        raw_filename))


def main():
    args = parse_args()

    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_filename  = args.output if args.output else f'msx_log_{timestamp_str}.csv'
    raw_filename  = csv_filename.replace('.csv', '.bin') if args.save_raw else None

    print(f'Port        : {args.port}')
    print(f'Output(CSV) : {csv_filename}')
    if raw_filename:
        print(f'Output(RAW) : {raw_filename}')
    print('Press Ctrl+C to stop.')
    print()

    try:
        ser = serial.Serial(args.port, baudrate=115200, timeout=1)
    except serial.SerialException as e:
        print(f'Error: Cannot open port "{args.port}": {e}', file=sys.stderr)
        sys.exit(1)

    try:
        with open(csv_filename, 'w', newline='', encoding='utf-8') as csv_file:
            raw_file_ctx = open(raw_filename, 'wb') if raw_filename else None
            try:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([
                    'seq', 'timestamp_us', 'addr_hex', 'addr_dec',
                    'data_hex', 'data_dec', 'flags_hex',
                    'rd', 'wr', 'io', 'drop_latched', 'dma_lag', 'drop_seq',
                ])
                receive_loop(ser, csv_writer, raw_file_ctx, args.warn_drop,
                             csv_filename, raw_filename)
            finally:
                if raw_file_ctx:
                    raw_file_ctx.close()
    finally:
        ser.close()

    print(f'Output(CSV) : {csv_filename}')
    if raw_filename:
        print(f'Output(RAW) : {raw_filename}')


if __name__ == '__main__':
    main()