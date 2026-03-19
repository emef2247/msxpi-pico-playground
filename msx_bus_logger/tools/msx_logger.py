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

パケットフォーマット（12バイト固定長）:

  STATUSパケット (TYPE=0x01):
    Byte 0   : マジックバイト1 (0xAA)
    Byte 1   : マジックバイト2 (0x55)
    Byte 2   : TYPE (0x01)
    Byte 3   : SM_ID (0x00=SM0/WR, 0x01=SM1/IORQ)
    Byte 4   : DROP_LATCH
    Byte 5   : DROP_SEQ
    Byte 6-9 : TIMESTAMP (uint32_t, little-endian, μs)
    Byte 10-11: PENDING (uint16_t, little-endian)

  EVENTパケット (TYPE=0x02):
    Byte 0   : マジックバイト1 (0xAA)
    Byte 1   : マジックバイト2 (0x55)
    Byte 2   : TYPE (0x02)
    Byte 3   : SM_ID (0x00=SM0/WR, 0x01=SM1/IORQ)
    Byte 4   : FLAGS
    Byte 5-6 : ADDR (uint16_t, little-endian, A0-A15)
    Byte 7   : DATA (D0-D7)
    Byte 8-11: 予約 (0x00 × 4バイト)

FLAGSのビット定義:
    bit0: RD    (/RD がアサート)
    bit1: WR    (/WR がアサート)
    bit2: IO    (/IORQ がアサート)
    bit3: SLTSL (/SLTSL がアサート)
    bit4: WAIT  (/WAIT がアサート)

SM別イベント種別:
    SM0 (/WR トリガー):
        FLAGS & IO=1 & WR=1 → IO Write  → IOカウント
        FLAGS & IO=0 & WR=1 → MEM Write → MEMカウント
    SM1 (/IORQ トリガー):
        FLAGS & IO=1 & WR=1 → IO Write (重複) → 除外
        FLAGS & IO=1 & RD=1 → IO Read          → IOカウント

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
        メモリマップドのため IO では捕捉不可（MEM_WR で対応）
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

PKT_TYPE_STATUS = 0x01
PKT_TYPE_EVENT  = 0x02

SM_ID_WR   = 0x00  # SM0: /WR トリガー
SM_ID_IORQ = 0x01  # SM1: /IORQ トリガー
SM_ID_RD   = 0x02  # SM2: /RD トリガー

# STATUSパケット: '<2sBBBBIH'
# (magic:2s, type:B, sm_id:B, drop_latch:B, drop_seq:B, timestamp:I, pending:H)
FMT_STATUS = '<2sBBBBIH'

# EVENTパケット: '<2sBBBHB4s'
# (magic:2s, type:B, sm_id:B, flags:B, addr:H, data:B, reserved:4s)
FMT_EVENT = '<2sBBBHB4s'

# FLAGSビット定義
FLAG_RD    = 1 << 0  # /RD アサート
FLAG_WR    = 1 << 1  # /WR アサート
FLAG_IO    = 1 << 2  # /IORQ アサート
FLAG_SLTSL = 1 << 3  # /SLTSL アサート
FLAG_WAIT  = 1 << 4  # /WAIT アサート

# ─────────────────────────────────────────
# IOアドレスセット定義（下位8ビットで照合）
# ─────────────────────────────────────────
VDP_ADDRS         = frozenset([0x98, 0x99, 0x9A, 0x9B])  # VDP (TMS9918/V9938/V9958)
PSG_WR_ADDRS      = frozenset([0xA0, 0xA1])               # PSG WR (AY-3-8910)
PSG_RD_ADDRS      = frozenset([0xA2])                     # PSG RD
OPLL_WR_ADDRS     = frozenset([0x7C, 0x7D])               # OPLL WR (YM2413)
OPLL_RD_ADDRS     = frozenset([0x7E])                     # OPLL RD (ステータス)
MSXAUDIO_WR_ADDRS = frozenset([0xC0, 0xC1, 0xC2, 0xC3])  # MSX-Audio WR (Y8950)
MSXAUDIO_RD_ADDRS = frozenset([0xC1])                     # MSX-Audio RD

# MEM Read アドレス範囲（SM2 /RD トリガー用）
SCC_RD_RANGE    = (0x9800, 0x9FFF)   # SCC RD
SCCI_RD_RANGE   = (0xB800, 0xBFFF)   # SCC-I RD


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
    seq               = 0    # CSVに書き込んだイベント数
    count_total_sm0   = 0    # SM0 EVENTパケット総数
    count_total_sm1   = 0    # SM1 EVENTパケット総数（除外分含む）
    count_total_sm2   = 0    # SM2 EVENTパケット総数
    count_io          = 0    # IO Write + IO Read
    count_io_wr       = 0    # IO Write (SM0由来)
    count_io_rd       = 0    # IO Read (SM1由来)
    count_io_rd_sm2   = 0    # IO Read (SM2由来、SM1と重複するが別集計)
    count_vdp_rd      = 0
    count_vdp_wr      = 0
    count_psg_rd      = 0
    count_psg_wr      = 0
    count_opll_rd     = 0
    count_opll_wr     = 0
    count_msxaudio_rd = 0
    count_msxaudio_wr = 0
    count_mem_wr      = 0    # MEM Write (SM0由来)
    count_scc_wr      = 0    # 0x9800-0x9FFF
    count_scci_wr     = 0    # 0xB800-0xBFFF
    count_scc_banksw  = 0    # 0x9000
    count_mem_rd      = 0    # MEM Read (SM2由来)
    count_scc_rd      = 0    # 0x9800-0x9FFF
    count_scci_rd     = 0    # 0xB800-0xBFFF

    # ── STATUS トラッキング ───────────────────
    drop_latch_sm0    = 0
    drop_seq_sm0      = 0
    drop_latch_sm1    = 0
    drop_seq_sm1      = 0
    drop_latch_sm2    = 0
    drop_seq_sm2      = 0
    count_status_sm0  = 0
    count_status_sm1  = 0
    count_status_sm2  = 0

    # ── peak 計測（PC時刻ベース 1秒ウィンドウ）──
    window_start    = time.time()
    window_count    = 0    # CSV書き込みイベント数
    window_bytes    = 0    # 受信バイト数（全パケット）
    peak_rate       = 0.0   # events/sec
    peak_throughput = 0.0   # KB/sec

    total_received  = 0    # 受信総バイト数（全パケット）

    start_time = time.time()

    try:
        while True:
            if not sync_to_magic(ser, raw_file):
                continue

            rest = read_exact(ser, PACKET_SIZE - 2)
            if raw_file:
                raw_file.write(rest)

            packet = MAGIC + rest
            pkt_type = rest[0]  # Byte 2 = TYPE

            total_received += PACKET_SIZE

            write_csv = False

            if pkt_type == PKT_TYPE_STATUS:
                # ── STATUSパケット処理 ───────────────
                try:
                    _, _type, sm_id, drop_latch, drop_seq_val, timestamp, pending = \
                        struct.unpack(FMT_STATUS, packet)
                except struct.error:
                    continue

                if sm_id == SM_ID_WR:
                    drop_latch_sm0 = drop_latch
                    drop_seq_sm0   = drop_seq_val
                    count_status_sm0 += 1
                elif sm_id == SM_ID_IORQ:
                    drop_latch_sm1 = drop_latch
                    drop_seq_sm1   = drop_seq_val
                    count_status_sm1 += 1
                elif sm_id == SM_ID_RD:
                    drop_latch_sm2 = drop_latch
                    drop_seq_sm2   = drop_seq_val
                    count_status_sm2 += 1

                if warn_drop and drop_latch:
                    print(
                        f'[!DROP SM{sm_id}] drop_seq={drop_seq_val}'
                        f' @ {timestamp}us pending={pending}',
                        file=sys.stderr
                    )

            elif pkt_type == PKT_TYPE_EVENT:
                # ── EVENTパケット処理 ────────────────
                try:
                    _, _type, sm_id, flags, addr, data, _reserved = \
                        struct.unpack(FMT_EVENT, packet)
                except struct.error:
                    continue

                rd     = bool(flags & FLAG_RD)
                wr     = bool(flags & FLAG_WR)
                io     = bool(flags & FLAG_IO)
                sltsl  = bool(flags & FLAG_SLTSL)
                wait_f = bool(flags & FLAG_WAIT)

                addr_lo = addr & 0xFF

                is_mem_wr = False
                is_mem_rd = False
                is_io_wr  = False
                is_io_rd  = False

                if sm_id == SM_ID_WR:
                    # SM0: /WR トリガー → 全件CSVへ
                    count_total_sm0 += 1
                    write_csv = True

                    if io and wr:
                        # IO Write
                        is_io_wr = True
                        count_io += 1
                        count_io_wr += 1
                        if addr_lo in VDP_ADDRS:
                            count_vdp_wr += 1
                        elif addr_lo in PSG_WR_ADDRS:
                            count_psg_wr += 1
                        elif addr_lo in OPLL_WR_ADDRS:
                            count_opll_wr += 1
                        elif addr_lo in MSXAUDIO_WR_ADDRS:
                            count_msxaudio_wr += 1
                    elif not io and wr:
                        # MEM Write
                        is_mem_wr = True
                        count_mem_wr += 1
                        if 0x9800 <= addr <= 0x9FFF:
                            count_scc_wr += 1
                        elif 0xB800 <= addr <= 0xBFFF:
                            count_scci_wr += 1
                        if addr == 0x9000:
                            count_scc_banksw += 1

                elif sm_id == SM_ID_IORQ:
                    # SM1: /IORQ トリガー
                    count_total_sm1 += 1

                    if io and wr:
                        # IO Write (SM0と重複) → 除外
                        write_csv = False
                    elif io and rd:
                        # IO Read → CSVへ
                        is_io_rd = True
                        write_csv = True
                        count_io += 1
                        count_io_rd += 1
                        if addr_lo in VDP_ADDRS:
                            count_vdp_rd += 1
                        elif addr_lo in PSG_RD_ADDRS:
                            count_psg_rd += 1
                        elif addr_lo in OPLL_RD_ADDRS:
                            count_opll_rd += 1
                        elif addr_lo in MSXAUDIO_RD_ADDRS:
                            count_msxaudio_rd += 1
                    else:
                        # その他 → 除外
                        write_csv = False

                elif sm_id == SM_ID_RD:
                    # SM2: /RD トリガー
                    count_total_sm2 += 1
                    write_csv = True

                    if io and rd:
                        # IO Read（SM1と重複するが別集計）
                        is_io_rd = True
                        count_io_rd_sm2 += 1
                    elif not io and rd:
                        # MEM Read
                        is_mem_rd = True
                        count_mem_rd += 1
                        if SCC_RD_RANGE[0] <= addr <= SCC_RD_RANGE[1]:
                            count_scc_rd += 1
                        elif SCCI_RD_RANGE[0] <= addr <= SCCI_RD_RANGE[1]:
                            count_scci_rd += 1
                    else:
                        # rd=False: SM2は /RD トリガーなので通常発生しない（グリッチ等の防衛的処理）
                        write_csv = False

                if write_csv:
                    # Use PC-side elapsed time as a timestamp reference.
                    # EVENT packets do not carry a Pico timestamp (reserved bytes are 0x00)
                    # in the new packet format; STATUS packets carry Pico timestamps instead.
                    ts_us = int((time.time() - start_time) * 1_000_000)
                    csv_writer.writerow([
                        seq,
                        ts_us,
                        sm_id,
                        f'0x{addr:04X}',
                        addr,
                        f'0x{data:02X}',
                        data,
                        f'0x{flags:02X}',
                        int(rd),
                        int(wr),
                        int(io),
                        int(sltsl),
                        int(wait_f),
                        int(is_mem_wr),
                        int(is_mem_rd),
                        int(is_io_wr),
                        int(is_io_rd),
                    ])
                    seq += 1

            else:
                # 未知のパケット種別 → スキップ
                continue

            # ── peak 更新（1秒ウィンドウ）──────────
            now = time.time()
            if write_csv:
                window_count += 1
            window_bytes += PACKET_SIZE
            elapsed_win = now - window_start
            if elapsed_win >= 1.0:
                r  = window_count / elapsed_win
                tp = (window_bytes / 1024.0) / elapsed_win
                if r  > peak_rate:       peak_rate       = r
                if tp > peak_throughput: peak_throughput = tp
                window_start = now
                window_count = 0
                window_bytes = 0

    except KeyboardInterrupt:
        elapsed  = time.time() - start_time
        avg_rate = seq / elapsed if elapsed > 0 else 0.0
        avg_tp   = (total_received / 1024.0) / elapsed if elapsed > 0 else 0.0

        # ── サマリ表示 ────────────────────────────
        # W_LABEL = width of the label column.
        # Longest label (at indent=2): "    MSX-Audio WR (0xC0-0xC3)" = 28 chars.
        W_LABEL = 28

        def row(label, value, indent=0):
            pad = '  ' * indent
            lbl = f'{pad}{label}'
            return f'{lbl:<{W_LABEL}}: {value:>12}'

        print()
        print('=== Capture Summary ===')
        print(row('Total Events (SM0/WR)',        fmt_num(count_total_sm0)))
        print(row('Total Events (SM1/IORQ)',       fmt_num(count_total_sm1)))
        print(row('Total Events (SM2/RD)',         fmt_num(count_total_sm2)))
        print(row('IO Access',                     fmt_num(count_io),             indent=1))
        print(row('VDP RD (0x98-0x9B)',            fmt_num(count_vdp_rd),         indent=2))
        print(row('VDP WR (0x98-0x9B)',            fmt_num(count_vdp_wr),         indent=2))
        print(row('PSG RD (0xA2)',                 fmt_num(count_psg_rd),         indent=2))
        print(row('PSG WR (0xA0-0xA1)',            fmt_num(count_psg_wr),         indent=2))
        print(row('OPLL RD (0x7E)',                fmt_num(count_opll_rd),        indent=2))
        print(row('OPLL WR (0x7C-0x7D)',           fmt_num(count_opll_wr),        indent=2))
        print(row('MSX-Audio RD (0xC1)',           fmt_num(count_msxaudio_rd),    indent=2))
        print(row('MSX-Audio WR (0xC0-0xC3)',      fmt_num(count_msxaudio_wr),    indent=2))
        print(row('MEM Write',                     fmt_num(count_mem_wr),         indent=1))
        print(row('SCC WR (0x9800-0x9FFF)',        fmt_num(count_scc_wr),         indent=2))
        print(row('SCC-I WR (0xB800-0xBFFF)',      fmt_num(count_scci_wr),        indent=2))
        print(row('SCC Bank SW (0x9000)',          fmt_num(count_scc_banksw),     indent=2))
        print(row('MEM Read',                      fmt_num(count_mem_rd),         indent=1))
        print(row('SCC RD (0x9800-0x9FFF)',        fmt_num(count_scc_rd),         indent=2))
        print(row('SCC-I RD (0xB800-0xBFFF)',      fmt_num(count_scci_rd),        indent=2))
        print(row('IO RD (SM2, 0x98-0xFF)',        fmt_num(count_io_rd_sm2),      indent=1))
        print(row('DROP Latch SM0',                fmt_num(drop_latch_sm0)))
        print(row('DROP Sequence SM0',             fmt_num(drop_seq_sm0)))
        print(row('DROP Latch SM1',                fmt_num(drop_latch_sm1)))
        print(row('DROP Sequence SM1',             fmt_num(drop_seq_sm1)))
        print(row('DROP Latch SM2',                fmt_num(drop_latch_sm2)))
        print(row('DROP Sequence SM2',             fmt_num(drop_seq_sm2)))
        print(row('STATUS Packets SM0',            fmt_num(count_status_sm0)))
        print(row('STATUS Packets SM1',            fmt_num(count_status_sm1)))
        print(row('STATUS Packets SM2',            fmt_num(count_status_sm2)))
        print(row('Elapsed Time',                  f'{elapsed:>10.1f} sec'))
        print(row('Bus Event Rate',                f'{fmt_rate(avg_rate)} events/sec  Avg'))
        print(row('',                              f'{fmt_rate(peak_rate)} events/sec  Max'))
        print(row('Throughput',                    f'{fmt_rate(avg_tp)} KB/sec     Avg'))
        print(row('',                              f'{fmt_rate(peak_throughput)} KB/sec     Max'))
        print(row('Output (CSV)',                  csv_filename))
        if raw_filename:
            print(row('Output (RAW)',              raw_filename))


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
                    'seq', 'timestamp_us', 'sm_id', 'addr_hex', 'addr_dec',
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
