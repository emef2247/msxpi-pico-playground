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

IOアドレス定義（テーブル駆動方式）:
    IO_PORT_TABLE  : IOポートアドレス範囲 → デバイス名のマッピング
    MEM_PORT_TABLE : メモリアドレス範囲 → デバイス名のマッピング
    方向（RD/WR）はFLAGSビットで判定するため、テーブルには含まない。
"""

import argparse
import csv
import struct
import sys
import time
from collections import defaultdict
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
# IOポートテーブル（テーブル駆動方式）
# (addr_lo, addr_hi, device_name, description)
# 下位8ビットで照合する。優先順位: 先に定義されたエントリが優先。
# ─────────────────────────────────────────
IO_PORT_TABLE = [
    (0x7C, 0x7D, 'OPLL',      'OPLL WR (YM2413/MSX-Music)'),
    (0x7E, 0x7E, 'OPLL',      'OPLL RD status (YM2413)'),
    (0x98, 0x9B, 'VDP',       'VDP (TMS9918/V9938/V9958)'),
    (0xA0, 0xA2, 'PSG',       'PSG (AY-3-8910)'),
    (0xC0, 0xC3, 'MSX-Audio', 'MSX-Audio (Y8950)'),
]

# MEMアドレステーブル（16ビット全体で照合）
# (addr_lo, addr_hi, device_name, description)
MEM_PORT_TABLE = [
    (0x9000, 0x9000, 'SCC Bank',  'SCC バンク切り替え'),
    (0x9800, 0x9FFF, 'SCC WR',    'SCC 波形データ WR'),
    (0xB800, 0xBFFF, 'SCC-I WR',  'SCC-I 波形データ WR'),
]


def _build_io_map():
    """IO_PORT_TABLE からアドレス→デバイス名の辞書を生成する（先勝ち）"""
    addr_map = {}
    for lo, hi, name, _ in IO_PORT_TABLE:
        for addr in range(lo, hi + 1):
            if addr not in addr_map:
                addr_map[addr] = name
    return addr_map


IO_MAP = _build_io_map()


def lookup_io_device(addr_lo):
    """IOアドレス（下位8ビット）からデバイス名を返す"""
    return IO_MAP.get(addr_lo, 'Other IO')


def lookup_mem_device(addr):
    """メモリアドレス（16ビット）からデバイス名を返す。一致しなければ None"""
    for lo, hi, name, _ in MEM_PORT_TABLE:
        if lo <= addr <= hi:
            return name
    return None


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
    count_event        = 0   # EVENTパケット総数
    count_io           = 0   # IOアクセス（IORQ=Low かつ WR=Low）
    count_mem_wr       = 0   # MEM Write（テーブル一致分）
    count_mem_wr_other = 0   # MEM Write（テーブル不一致分）
    count_status_pkts  = 0

    io_device_counts  = defaultdict(int)  # key: device_name
    mem_device_counts = defaultdict(int)  # key: device_name

    # STATUSパケットから取得する状態
    drop_latch_ever    = False  # 一度でも drop_latch=1 を受信したか
    drop_seq_latest    = 0

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

                # ── CSV書き込み（STATUSパケット）──────────
                csv_writer.writerow([
                    '', 'STATUS', timestamp,
                    '', '', '', '', '',
                    '', '', '',
                    drop_latch, drop_seq_val, pending,
                    '',
                ])

                continue

            elif pkt_type == PKT_TYPE_EVENT:
                try:
                    _, _, flags, addr, data, _reserved = \
                        struct.unpack(FMT_EVENT, raw_packet)
                except struct.error:
                    continue

                # ── FLAGSビット展開 ──────────────────
                wr    = bool(flags & FLAG_WR)
                io    = bool(flags & FLAG_IO)
                sltsl = bool(flags & FLAG_SLTSL)

                # ── アクセス種別・デバイス別カウント ──────
                if io and wr:
                    # IO アクセス（IORQ=Low かつ WR=Low）
                    count_io += 1
                    device = lookup_io_device(addr & 0xFF)
                    io_device_counts[device] += 1
                elif wr and not io:
                    # MEM Write（IORQ=High かつ WR=Low）
                    device = lookup_mem_device(addr)
                    if device is not None:
                        count_mem_wr += 1
                        mem_device_counts[device] += 1
                    else:
                        count_mem_wr_other += 1

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

                # ── デバイスラベル（CSV用）──────────────
                if io and wr:
                    device_label = lookup_io_device(addr & 0xFF)
                elif wr and not io:
                    device_label = lookup_mem_device(addr) or ''
                else:
                    device_label = ''

                # ── CSV書き込み（EVENTパケット）──────────
                csv_writer.writerow([
                    count_event, 'EVENT', '',
                    f'0x{addr:04X}', addr,
                    f'0x{data:02X}', data,
                    f'0x{flags:02X}',
                    int(wr), int(io), int(sltsl),
                    '', '', '',
                    device_label,
                ])

                count_event += 1

            else:
                # 未知のパケットタイプ: スキップ
                continue

    except KeyboardInterrupt:
        elapsed  = time.time() - start_time
        avg_rate = count_event / elapsed if elapsed > 0 else 0.0
        avg_tp   = ((count_event + count_status_pkts) * PACKET_SIZE / 1024.0) / elapsed \
                   if elapsed > 0 else 0.0

        # ── サマリ表示 ────────────────────────────
        W_LABEL = 29

        def row(label, value, indent=0):
            pad = '  ' * indent
            lbl = f'{pad}{label}'
            return f'{lbl:<{W_LABEL}}: {value:>12}'

        print()
        print('=== Capture Summary ===')
        print(row('Total Events',   fmt_num(count_event)))
        print(row('IO Access',      fmt_num(count_io),    indent=1))

        # io_device_counts のうち count > 0 のものを IO_PORT_TABLE の順番で表示
        printed_io = set()
        for lo, hi, name, _ in IO_PORT_TABLE:
            if name in printed_io:
                continue
            printed_io.add(name)
            cnt = io_device_counts.get(name, 0)
            if cnt > 0:
                print(row(name, fmt_num(cnt), indent=2))

        # MEM Write（0件なら行ごと非表示）
        if count_mem_wr > 0 or count_mem_wr_other > 0:
            print(row('MEM Write', fmt_num(count_mem_wr + count_mem_wr_other), indent=1))
            for _, _, name, _ in MEM_PORT_TABLE:
                cnt = mem_device_counts.get(name, 0)
                if cnt > 0:
                    print(row(name, fmt_num(cnt), indent=2))

        print(row('DROP Latch',     fmt_num(1 if drop_latch_ever else 0)))
        print(row('DROP Sequence',  fmt_num(drop_seq_latest)))
        print(row('STATUS Packets', fmt_num(count_status_pkts)))
        print(row('Elapsed Time',   f'{elapsed:>10.1f} sec'))
        print(row('Bus Event Rate', f'{fmt_rate(avg_rate)} events/sec  Avg'))
        print(row('',               f'{fmt_rate(peak_rate)} events/sec  Max'))
        print(row('Throughput',     f'{fmt_rate(avg_tp)} KB/sec     Avg'))
        print(row('',               f'{fmt_rate(peak_throughput)} KB/sec     Max'))
        print(row('Output (CSV)',   csv_filename))
        if raw_filename:
            print(row('Output (RAW)', raw_filename))


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
                    'seq', 'type', 'timestamp_us', 'addr_hex', 'addr_dec',
                    'data_hex', 'data_dec', 'flags_hex',
                    'wr', 'io', 'sltsl',
                    'drop_latch', 'drop_seq', 'pending',
                    'device',
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