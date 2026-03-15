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

パケットフォーマット（全パケット 11バイト固定長）:
  共通ヘッダ:
    Byte 0   : 0xAA  (マジックバイト1)
    Byte 1   : 0x55  (マジックバイト2)
    Byte 2   : TYPE  (0x01=STATUS, 0x02=EVENT)

  STATUSパケット（TYPE=0x01）:
    Byte 3   : DROP_LATCH  (uint8_t, 0 or 1)
    Byte 4   : DROP_SEQ    (uint8_t)
    Byte 5-8 : TIMESTAMP   (uint32_t, μs, little-endian)
    Byte 9-10: PENDING     (uint16_t, little-endian)

  EVENTパケット（TYPE=0x02）:
    Byte 3   : FLAGS  (下記参照)
    Byte 4-5 : ADDR   (uint16_t, little-endian, A0-A15)
    Byte 6   : DATA   (uint8_t, D0-D7)
    Byte 7-10: 予約   (0x00 × 4バイト)

FLAGSビット定義（EVENTパケット Byte 3）:
    bit 0: /RD    (GPIO24がLowでアサート → 1)
    bit 1: /WR    (GPIO25がLowでアサート → 1)
    bit 2: /IORQ  (GPIO26がLowでアサート → 1)
    bit 3: /SLTSL (GPIO27がLowでアサート → 1)
    bit 4: /WAIT  (GPIO28がLowでアサート → 1)
    bit 5-7: 未使用 (0)

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
        メモリマップドのため IORQ では捕捉できない（MEM_WR で対応）
        0x9000        : SCCバンク切り替え (MEM WR)
        0x9800-0x9FFF : SCC 波形データ (MEM WR)
        0xB800-0xBFFF : SCC-I 波形データ (MEM WR)
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
MAGIC           = bytes([0xAA, 0x55])
PACKET_SIZE     = 11   # 全パケット固定長（STATUS/EVENT共通）

PKT_TYPE_STATUS = 0x01
PKT_TYPE_EVENT  = 0x02

# STATUSパケット struct フォーマット: '<2sBBBIH'
# (magic:2s, type:B, drop_latch:B, drop_seq:B, timestamp:I, pending:H)
FMT_STATUS = '<2sBBBIH'

# EVENTパケット struct フォーマット: '<2sBBHB4s'
# (magic:2s, type:B, flags:B, addr:H, data:B, reserved:4s)
FMT_EVENT = '<2sBBHB4s'

# FLAGSビット定義（EVENTパケット）
FLAG_RD    = 1 << 0  # /RD アサート
FLAG_WR    = 1 << 1  # /WR アサート
FLAG_IO    = 1 << 2  # /IORQ アサート
FLAG_SLTSL = 1 << 3  # /SLTSL アサート
FLAG_WAIT  = 1 << 4  # /WAIT アサート

# ─────────────────────────────────────────
# IOアドレスセット定義（下位8ビットで照合）
# ─────────────────────────────────────────
VDP_ADDRS       = frozenset([0x98, 0x99, 0x9A, 0x9B])  # VDP (TMS9918/V9938/V9958)
PSG_ADDRS       = frozenset([0xA0, 0xA1, 0xA2])         # PSG (AY-3-8910)
OPLL_WR_ADDRS   = frozenset([0x7C, 0x7D])               # OPLL WR (YM2413)
OPLL_RD_ADDRS   = frozenset([0x7E])                     # OPLL RD (ステータス)
MSXAUDIO_ADDRS  = frozenset([0xC0, 0xC1, 0xC2, 0xC3])  # MSX-Audio (Y8950)


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
    """0xAA 0x55 を探してフレーム同期する。
    同期完了後 TYPE バイトと残りデータを読む。
    戻り値: (pkt_type, rest_bytes) または (None, None)
    rest_bytes は TYPE バイトの後の (PACKET_SIZE - 3) バイト。
    """
    b = ser.read(1)
    if not b:
        return None, None
    if raw_file:
        raw_file.write(b)

    while True:
        if b[0] == 0xAA:
            b2 = ser.read(1)
            if not b2:
                return None, None
            if raw_file:
                raw_file.write(b2)
            if b2[0] == 0x55:
                # マジック検出: TYPEバイトを読む
                type_b = ser.read(1)
                if not type_b:
                    return None, None
                if raw_file:
                    raw_file.write(type_b)
                pkt_type = type_b[0]
                # 残りのバイトを読む (PACKET_SIZE - 3 バイト)
                try:
                    rest = read_exact(ser, PACKET_SIZE - 3)
                except IOError:
                    return None, None
                if raw_file:
                    raw_file.write(rest)
                return pkt_type, rest
            b = b2
            continue
        b = ser.read(1)
        if not b:
            return None, None
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
    seq                = 0   # EVENTパケットのシーケンス番号
    count_io           = 0
    count_vdp_rd       = 0
    count_vdp_wr       = 0
    count_psg_rd       = 0
    count_psg_wr       = 0
    count_opll_rd      = 0
    count_opll_wr      = 0
    count_msxaudio_rd  = 0
    count_msxaudio_wr  = 0
    count_mem_wr       = 0
    count_scc_wr       = 0
    count_scci_wr      = 0
    count_scc_bank     = 0
    count_status_pkts  = 0

    # STATUSパケットから取得する状態
    drop_latch_ever    = False  # 一度でも drop_latch=1 を受信したか
    drop_seq_latest    = 0
    latest_timestamp   = 0     # 直近STATUSパケットのタイムスタンプ（μs）

    # ── peak 計測（PC時刻ベース 1秒ウィンドウ）──
    window_start       = time.time()
    window_event_count = 0
    window_bytes       = 0
    peak_rate          = 0.0   # events/sec
    peak_throughput    = 0.0   # KB/sec

    start_time = time.time()

    try:
        while True:
            pkt_type, rest = sync_to_magic(ser, raw_file)
            if pkt_type is None:
                continue

            raw_packet = MAGIC + bytes([pkt_type]) + rest

            if pkt_type == PKT_TYPE_STATUS:
                try:
                    _, _, drop_latch, drop_seq_val, timestamp, pending = \
                        struct.unpack(FMT_STATUS, raw_packet)
                except struct.error:
                    continue

                # STATUSパケット受信時の状態更新
                if drop_latch:
                    drop_latch_ever = True
                drop_seq_latest  = drop_seq_val
                latest_timestamp = timestamp
                count_status_pkts += 1

                # ── DROP 警告 ─────────────────────────
                if warn_drop and drop_latch:
                    print(
                        f'[!DROP] status drop_seq={drop_seq_val} pending={pending}'
                        f' @ {timestamp}us',
                        file=sys.stderr
                    )

                # ── peak 更新（STATUSパケットもバイト計上）──
                now = time.time()
                window_bytes += PACKET_SIZE
                elapsed_win = now - window_start
                if elapsed_win >= 1.0:
                    r  = window_event_count / elapsed_win
                    tp = (window_bytes / 1024.0) / elapsed_win
                    if r  > peak_rate:       peak_rate       = r
                    if tp > peak_throughput: peak_throughput = tp
                    window_start       = now
                    window_event_count = 0
                    window_bytes       = 0

                continue

            elif pkt_type == PKT_TYPE_EVENT:
                try:
                    _, _, flags, addr, data, _reserved = \
                        struct.unpack(FMT_EVENT, raw_packet)
                except struct.error:
                    continue

                # ── FLAGSビット展開 ──────────────────
                rd    = bool(flags & FLAG_RD)
                wr    = bool(flags & FLAG_WR)
                io    = bool(flags & FLAG_IO)
                sltsl = bool(flags & FLAG_SLTSL)
                wait  = bool(flags & FLAG_WAIT)

                # ── アクセス種別導出 ─────────────────
                is_io_wr  = io and wr
                is_io_rd  = io and rd
                is_mem_wr = (not io) and wr
                is_mem_rd = (not io) and rd

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

                if is_mem_wr:
                    count_mem_wr += 1
                    if 0x9800 <= addr <= 0x9FFF:
                        count_scc_wr += 1
                    elif 0xB800 <= addr <= 0xBFFF:
                        count_scci_wr += 1
                    elif addr == 0x9000:
                        count_scc_bank += 1

                # ── peak 更新（1秒ウィンドウ）──────────
                now = time.time()
                window_event_count += 1
                window_bytes       += PACKET_SIZE
                elapsed_win = now - window_start
                if elapsed_win >= 1.0:
                    r  = window_event_count / elapsed_win
                    tp = (window_bytes / 1024.0) / elapsed_win
                    if r  > peak_rate:       peak_rate       = r
                    if tp > peak_throughput: peak_throughput = tp
                    window_start       = now
                    window_event_count = 0
                    window_bytes       = 0

                # ── CSV書き込み ──────────────────────
                csv_writer.writerow([
                    seq,
                    latest_timestamp,    # STATUSパケットから取得した直近タイムスタンプ
                    f'0x{addr:04X}',
                    addr,
                    f'0x{data:02X}',
                    data,
                    f'0x{flags:02X}',
                    int(rd),
                    int(wr),
                    int(io),
                    int(sltsl),
                    int(wait),
                    int(is_mem_wr),
                    int(is_mem_rd),
                    int(is_io_wr),
                    int(is_io_rd),
                ])

                seq += 1

            else:
                # 未知のパケットタイプ: スキップ
                continue

    except KeyboardInterrupt:
        elapsed  = time.time() - start_time
        avg_rate = seq / elapsed if elapsed > 0 else 0.0
        avg_tp   = ((seq + count_status_pkts) * PACKET_SIZE / 1024.0) / elapsed \
                   if elapsed > 0 else 0.0

        # ── サマリ表示 ────────────────────────────
        W_LABEL = 29

        def row(label, value, indent=0):
            pad = '  ' * indent
            lbl = f'{pad}{label}'
            return f'{lbl:<{W_LABEL}}: {value:>12}'

        print()
        print('=== Capture Summary ===')
        print(row('Total Events',               fmt_num(seq)))
        print(row('IO Access',                  fmt_num(count_io),          indent=1))
        print(row('VDP RD (0x98-0x9B)',         fmt_num(count_vdp_rd),      indent=2))
        print(row('VDP WR (0x98-0x9B)',         fmt_num(count_vdp_wr),      indent=2))
        print(row('PSG RD (0xA2)',              fmt_num(count_psg_rd),      indent=2))
        print(row('PSG WR (0xA0-0xA1)',         fmt_num(count_psg_wr),      indent=2))
        print(row('OPLL RD (0x7E)',             fmt_num(count_opll_rd),     indent=2))
        print(row('OPLL WR (0x7C-0x7D)',        fmt_num(count_opll_wr),     indent=2))
        print(row('MSX-Audio RD (0xC1)',        fmt_num(count_msxaudio_rd), indent=2))
        print(row('MSX-Audio WR (0xC0-0xC3)',   fmt_num(count_msxaudio_wr), indent=2))
        print(row('MEM Write',                  fmt_num(count_mem_wr),      indent=1))
        print(row('SCC WR (0x9800-0x9FFF)',    fmt_num(count_scc_wr),      indent=2))
        print(row('SCC-I WR (0xB800-0xBFFF)',  fmt_num(count_scci_wr),     indent=2))
        print(row('SCC Bank SW (0x9000)',       fmt_num(count_scc_bank),    indent=2))
        print(row('DROP Latch',                 fmt_num(1 if drop_latch_ever else 0)))
        print(row('DROP Sequence',              fmt_num(drop_seq_latest)))
        print(row('STATUS Packets',             fmt_num(count_status_pkts)))
        print(row('Elapsed Time',               f'{elapsed:>10.1f} sec'))
        print(row('Bus Event Rate',             f'{fmt_rate(avg_rate)} events/sec  Avg'))
        print(row('',                           f'{fmt_rate(peak_rate)} events/sec  Max'))
        print(row('Throughput',                 f'{fmt_rate(avg_tp)} KB/sec     Avg'))
        print(row('',                           f'{fmt_rate(peak_throughput)} KB/sec     Max'))
        print(row('Output (CSV)',               csv_filename))
        if raw_filename:
            print(row('Output (RAW)',           raw_filename))


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
                    'seq', 'last_status_ts_us', 'addr_hex', 'addr_dec',
                    'data_hex', 'data_dec', 'flags_hex',
                    'rd', 'wr', 'io', 'sltsl', 'wait',
                    'is_mem_wr', 'is_mem_rd', 'is_io_wr', 'is_io_rd',
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