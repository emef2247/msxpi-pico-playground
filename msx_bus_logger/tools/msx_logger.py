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
from collections import defaultdict
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
# IO ポートテーブル（https://www.msx.org/wiki/I/O_Ports_List ベース）
# (addr_lo, addr_hi, direction, device_name, description)
# direction: 'R'=Read のみ, 'W'=Write のみ, 'RW'=両方
# ─────────────────────────────────────────
IO_PORT_TABLE = [
    (0x00, 0x07, 'RW', 'Internal',   'MSX system internal'),
    (0x08, 0x08, 'W',  'VDP',        'Wait disable (MSX2+/turboR)'),
    (0x0F, 0x0F, 'RW', 'System',     'Software reset / system flags'),
    (0x10, 0x17, 'RW', 'MSX-Audio',  'MSX-Audio PCM port (Y8950)'),
    (0x1A, 0x1B, 'RW', 'FM-PAC',     'FM-PAC (MSX-Music clone, Panasonic)'),
    (0x20, 0x27, 'RW', 'RS-232',     'RS-232C (MSX2+)'),
    (0x2E, 0x2F, 'RW', 'System',     'MSX2+ system'),
    (0x3F, 0x3F, 'W',  'TurboR',     'CPU mode switch (turboR)'),
    (0x40, 0x4F, 'RW', 'Disk',       'Floppy disk controller (WD2793/TC8566)'),
    (0x50, 0x57, 'RW', 'SCSI',       'SCSI interface (MSX-SCSI)'),
    (0x60, 0x67, 'RW', 'Kanji',      'Kanji ROM controller'),
    (0x70, 0x77, 'RW', 'MIDI',       'MIDI interface'),
    (0x7C, 0x7C, 'W',  'OPLL',       'OPLL register select (YM2413/MSX-Music)'),
    (0x7D, 0x7D, 'W',  'OPLL',       'OPLL data write (YM2413/MSX-Music)'),
    (0x7E, 0x7E, 'R',  'OPLL',       'OPLL status read (YM2413)'),
    (0x80, 0x87, 'RW', 'RS-232',     'RS-232C (8251/8253)'),
    (0x90, 0x90, 'RW', 'Printer',    'Printer strobe / status'),
    (0x91, 0x91, 'W',  'Printer',    'Printer data'),
    (0x98, 0x98, 'RW', 'VDP',        'VDP data port (TMS9918/V9938/V9958)'),
    (0x99, 0x99, 'RW', 'VDP',        'VDP register / status port'),
    (0x9A, 0x9A, 'W',  'VDP',        'VDP palette port (V9938/V9958)'),
    (0x9B, 0x9B, 'W',  'VDP',        'VDP indirect register port (V9938/V9958)'),
    (0xA0, 0xA0, 'W',  'PSG',        'PSG register select (AY-3-8910)'),
    (0xA1, 0xA1, 'W',  'PSG',        'PSG data write (AY-3-8910)'),
    (0xA2, 0xA2, 'R',  'PSG',        'PSG data read (AY-3-8910)'),
    (0xA8, 0xA8, 'RW', 'PPI',        'PPI port A / slot select (8255)'),
    (0xA9, 0xA9, 'R',  'PPI',        'PPI port B / keyboard row (8255)'),
    (0xAA, 0xAA, 'RW', 'PPI',        'PPI port C / key/CAS/beep (8255)'),
    (0xAB, 0xAB, 'W',  'PPI',        'PPI control register (8255)'),
    (0xB0, 0xB3, 'RW', 'Calendar',   'Clock/Calendar (RP5C01)'),
    (0xB8, 0xBB, 'RW', 'LightPen',   'Light pen'),
    # C0h-C3h: MSX-Audio 優先（Moonsound alternative ports より先に定義）
    (0xC0, 0xC0, 'W',  'MSX-Audio', 'MSX-Audio register select ch1 (Y8950)'),
    (0xC1, 0xC1, 'RW', 'MSX-Audio', 'MSX-Audio data/status ch1 (Y8950)'),
    (0xC2, 0xC2, 'W',  'MSX-Audio', 'MSX-Audio register select ch2 (Y8950)'),
    (0xC3, 0xC3, 'W',  'MSX-Audio', 'MSX-Audio data ch2 (Y8950)'),
    (0xC4, 0xC7, 'RW', 'Moonsound', 'Moonsound / OPL4 (YMF278B) primary ports'),
    (0xD0, 0xD7, 'RW', 'Disk-Ext',   'FDC extended (Microsol)'),
    (0xD8, 0xD9, 'RW', 'Kanji',     'Kanji ROM JIS1 address'),
    (0xDA, 0xDB, 'RW', 'Kanji',     'Kanji ROM JIS2 address'),
    (0xE0, 0xE3, 'RW', 'MemMapper', 'Memory mapper control'),
    (0xE4, 0xE7, 'R',  'MemMapper', 'Memory mapper slot register read'),
    (0xE8, 0xEB, 'RW', 'System',    'MSX2+ / turboR system'),
    (0xF0, 0xF3, 'RW', 'TurboR',    'turboR / MSX-Engine control'),
    (0xF4, 0xF4, 'RW', 'System',    'System flags (MSX2+/turboR)'),
    (0xF5, 0xF5, 'RW', 'System',    'System flags B'),
    (0xF6, 0xF7, 'RW', 'System',    'Color bus / system'),
    (0xF8, 0xFB, 'RW', 'TurboR',    'turboR internal'),
    (0xFC, 0xFF, 'RW', 'MemMapper', 'Memory mapper page select (page 0-3)'),
]


def build_port_map():
    """IO_PORT_TABLE から高速ルックアップ辞書を生成する（先着優先）"""
    port_map = {}
    for lo, hi, direction, device_name, _ in IO_PORT_TABLE:
        for addr in range(lo, hi + 1):
            if addr not in port_map:
                port_map[addr] = (device_name, direction)
    return port_map


PORT_MAP = build_port_map()


def lookup_io_device(addr_lo, is_rd):
    """アドレスとアクセス方向からデバイス名を返す"""
    entry = PORT_MAP.get(addr_lo)
    if entry is None:
        return 'Unknown'
    device_name, direction = entry
    if direction == 'RW':
        return device_name
    if direction == 'R' and is_rd:
        return device_name
    if direction == 'W' and not is_rd:
        return device_name
    return f'{device_name}(?dir)'


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
    io_device_counts   = defaultdict(int)   # key: (device_name, 'RD' or 'WR')
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
                device = lookup_io_device(addr_lo, rd)
                io_device_counts[(device, 'RD' if rd else 'WR')] += 1

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
            device_label = lookup_io_device(addr_lo, rd) if io else ''
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
                device_label,
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
        print(row('IO Access',                fmt_num(count_io),  indent=1))

        # デバイス代表アドレス昇順でデバイス別カウントを表示
        printed_devices = set()
        for addr in range(0x100):
            entry = PORT_MAP.get(addr)
            if not entry:
                continue
            dev = entry[0]
            if dev in printed_devices:
                continue
            printed_devices.add(dev)
            rd_cnt = io_device_counts.get((dev, 'RD'), 0)
            wr_cnt = io_device_counts.get((dev, 'WR'), 0)
            if rd_cnt == 0 and wr_cnt == 0:
                continue
            if rd_cnt > 0:
                print(row(f'{dev} RD', fmt_num(rd_cnt), indent=2))
            if wr_cnt > 0:
                print(row(f'{dev} WR', fmt_num(wr_cnt), indent=2))

        # Unknown（テーブル未定義ポート）があれば末尾に表示
        unknown_rd = io_device_counts.get(('Unknown', 'RD'), 0)
        unknown_wr = io_device_counts.get(('Unknown', 'WR'), 0)
        if unknown_rd > 0:
            print(row('Unknown RD', fmt_num(unknown_rd), indent=2))
        if unknown_wr > 0:
            print(row('Unknown WR', fmt_num(unknown_wr), indent=2))

        print(row('SCC/SCC-I',               '(MEM_WR)',          indent=2))
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