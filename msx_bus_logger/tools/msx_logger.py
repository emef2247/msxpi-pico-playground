#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
msx_logger.py - MSXバスロガー バイナリ受信スクリプト

使用方法:
    python msx_logger.py [--port COM8] [--output FILE] [--warn-drop] [--save-raw]
                         [--filter-io RANGE] [--filter-mem RANGE]
                         [--filter-type TYPES] [--no-io] [--no-mem]

オプション:
    --port          COMポート指定（デフォルト: COM8）
    --output        CSVファイル名（デフォルト: msx_log_YYYYMMDD_HHMMSS.csv）
    --warn-drop     DROP警告をコンソールに表示する（デフォルト: OFF）
    --save-raw      バイナリをそのまま .bin ファイルにも保存する（デフォルト: OFF）
    --filter-io     IOアクセスのアドレス範囲フィルタ（例: 0x00-0xFF）
    --filter-mem    MEMアクセスのアドレス範囲フィルタ（例: 0x4000-0x7FFF）
    --filter-type   アクセス種別フィルタ（例: IO,MEM,RD,WR カンマ区切り）
    --no-io         IOアクセスを記録しない
    --no-mem        MEMアクセスを記録しない

依存ライブラリ:
    pip install pyserial

Picoのバイナリ転送モード（TEXT_MODE=0）と組み合わせて使用する。

パケットフォーマット（1イベント = 12バイト固定長）:
    Byte 0   : マジックバイト1 (0xAA)
    Byte 1   : マジックバイト2 (0x55)
    Byte 2   : FLAGS（GPIO生値統一: bit0=/RD, bit1=/WR, bit2=/IORQ, bit3=/SLTSL, bit4=DROP, bit6=DMA遅延）
    Byte 3   : DROP_SEQ
    Byte 4-5 : ADDR (little-endian, A0-A15)
    Byte 6   : DATA (D0-D7)
    Byte 7   : 予約 (0x00)
    Byte 8-11: タイムスタンプ (uint32_t, little-endian, μs)

FLAGSのビット定義（すべて GPIO 生値、0=Low, 1=High）:
    bit0: /RD    GPIO24 生値（0=Low=アサート中）
    bit1: /WR    GPIO25 生値（0=Low=アサート中）
    bit2: /IORQ  GPIO26 生値（0=Low=アサート中）
    bit3: /SLTSL GPIO27 生値（0=Low=アサート中）
    bit4: DROP_LATCHED
    bit5: TX_DROP（TXリングバッファ溢れ）
    bit6: DMA_LAG
"""

import argparse
import csv
import struct
import sys
import time
from datetime import datetime

import serial

# パケット定義
MAGIC = bytes([0xAA, 0x55])
PACKET_SIZE = 12

# パケット構造: AA 55 FLAGS DROP_SEQ ADDR_LO ADDR_HI DATA RESERVED TS0 TS1 TS2 TS3
# struct フォーマット: '<2sBBHBBI'
# (magic:2s, flags:B, drop_seq:B, addr:H, data:B, reserved:B, timestamp:I)
FMT = '<2sBBHBBI'

# FLAGSビット定義（すべて GPIO 生値、0=Low, 1=High）
FLAG_RD           = 1 << 0  # GPIO24 /RD    生値（0=Low=アサート中）
FLAG_WR           = 1 << 1  # GPIO25 /WR    生値（0=Low=アサート中）
FLAG_IO           = 1 << 2  # GPIO26 /IORQ  生値（0=Low=アサート中）
FLAG_SLTSL        = 1 << 3  # GPIO27 /SLTSL 生値（0=Low=アサート中）
FLAG_DROP_LATCHED = 1 << 4  # DROP ラッチフラグ
FLAG_TX_DROP      = 1 << 5  # TXリングバッファ溢れ
FLAG_DMA_LAG      = 1 << 6  # DMA 遅延フラグ


def parse_range(s):
    """アドレス範囲文字列（例: 0x00-0xFF）を (min, max) タプルに変換する"""
    parts = s.split('-', 1)
    if len(parts) != 2:
        raise ValueError(f'無効な範囲形式: {s}（例: 0x00-0xFF）')
    try:
        lo = int(parts[0], 0)
        hi = int(parts[1], 0)
    except ValueError as e:
        raise ValueError(f'無効なアドレス値: {s}（例: 0x00-0xFF）') from e
    return lo, hi


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
  python msx_logger.py --port COM8 --no-mem
  python msx_logger.py --port COM8 --filter-io 0x00-0xFF --filter-mem 0x4000-0x7FFF
  python msx_logger.py --port COM8 --filter-type IO,RD
        """
    )
    parser.add_argument(
        '--port',
        default='COM8',
        help='COMポート指定（デフォルト: COM8）'
    )
    parser.add_argument(
        '--output',
        default=None,
        help='CSVファイル名（デフォルト: msx_log_YYYYMMDD_HHMMSS.csv）'
    )
    parser.add_argument(
        '--warn-drop',
        action='store_true',
        help='DROP警告をコンソールに表示する（デフォルト: OFF）'
    )
    parser.add_argument(
        '--save-raw',
        action='store_true',
        help='バイナリをそのまま .bin ファイルにも保存する（デフォルト: OFF）'
    )
    parser.add_argument(
        '--filter-io',
        default=None,
        metavar='RANGE',
        help='IOアクセスのアドレス範囲フィルタ（例: 0x00-0xFF）'
    )
    parser.add_argument(
        '--filter-mem',
        default=None,
        metavar='RANGE',
        help='MEMアクセスのアドレス範囲フィルタ（例: 0x4000-0x7FFF）'
    )
    parser.add_argument(
        '--filter-type',
        default=None,
        metavar='TYPES',
        help='アクセス種別フィルタ（例: IO,MEM,RD,WR カンマ区切り）'
    )
    parser.add_argument(
        '--no-io',
        action='store_true',
        help='IOアクセスを記録しない'
    )
    parser.add_argument(
        '--no-mem',
        action='store_true',
        help='MEMアクセスを記録しない'
    )
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
    """受信バイトストリームから 0xAA 0x55 を探してフレーム同期する

    同期が取れた場合は True を返す。
    同期がずれた場合は1バイトずつスキップして再同期を試みる。
    """
    # 最初の1バイトを読む
    b = ser.read(1)
    if not b:
        return False
    if raw_file:
        raw_file.write(b)

    while True:
        if b[0] == 0xAA:
            # マジックバイト1発見、次のバイトを確認
            b2 = ser.read(1)
            if not b2:
                return False
            if raw_file:
                raw_file.write(b2)
            if b2[0] == 0x55:
                # マジックバイト2も一致 → 同期完了
                return True
            else:
                # マジックバイト2不一致 → b2 から再試行
                b = b2
                continue
        # マジックバイト1不一致 → 次のバイトへスキップ
        b = ser.read(1)
        if not b:
            return False
        if raw_file:
            raw_file.write(b)


def receive_loop(ser, csv_writer, raw_file, warn_drop,
                 io_range, mem_range, type_filter, no_io, no_mem,
                 csv_path, raw_path):
    """メイン受信ループ

    パケットを受信してCSV書き込みおよびコンソール表示を行う。
    Ctrl+C で終了し、サマリを表示する。
    """
    seq = 0
    io_count    = 0  # /IORQ Low（IOアクセス）カウント
    mem_count   = 0  # /SLTSL Low（MEM候補）カウント
    drop_count  = 0  # DROPラッチ検出カウント
    filter_count = 0  # フィルタ除外カウント
    start_time = time.time()

    try:
        while True:
            # フレーム同期（マジックバイト 0xAA 0x55 を探す）
            if not sync_to_magic(ser, raw_file):
                continue

            # マジック2バイトは消費済み、残り10バイトを読む
            rest = read_exact(ser, PACKET_SIZE - 2)
            if raw_file:
                raw_file.write(rest)

            # パケット全体を再構成して unpack
            packet = MAGIC + rest
            try:
                magic, flags, drop_seq, addr, data, _reserved, timestamp = struct.unpack(FMT, packet)
            except struct.error:
                # unpack 失敗時はスキップして再同期
                continue

            # FLAGSからビットを展開（すべて GPIO 生値、0=Low, 1=High）
            rd           = (flags >> 0) & 1  # GPIO24 /RD    生値（0=Low=アサート中）
            wr           = (flags >> 1) & 1  # GPIO25 /WR    生値（0=Low=アサート中）
            io           = (flags >> 2) & 1  # GPIO26 /IORQ  生値（0=Low=アサート中）
            sltsl        = (flags >> 3) & 1  # GPIO27 /SLTSL 生値（0=Low=アサート中）
            drop_latched = (flags >> 4) & 1
            dma_lag      = (flags >> 6) & 1
            # Byte7 は予約（無視）

            # ===== フィルタ判定 =====

            # --no-io: IO アクセスを除外（/IORQ=Low のパケット）
            if no_io and io == 0:
                filter_count += 1
                continue

            # --no-mem: MEM アクセスを除外（/SLTSL=Low のパケット）
            if no_mem and sltsl == 0:
                filter_count += 1
                continue

            # --filter-type: アクセス種別フィルタ（ANY マッチ）
            if type_filter:
                event_flags = set()
                if rd == 0:    event_flags.add('RD')
                if wr == 0:    event_flags.add('WR')
                if io == 0:    event_flags.add('IO')
                if sltsl == 0: event_flags.add('MEM')
                if not event_flags & type_filter:
                    filter_count += 1
                    continue

            # --filter-io: IO アクセスのアドレス範囲フィルタ
            if io_range and io == 0:
                lo, hi = io_range
                if not (lo <= addr <= hi):
                    filter_count += 1
                    continue

            # --filter-mem: MEM アクセスのアドレス範囲フィルタ
            if mem_range and sltsl == 0:
                lo, hi = mem_range
                if not (lo <= addr <= hi):
                    filter_count += 1
                    continue

            # ===== アクセス種別カウント =====
            # io=0 が /IORQ=Low（IOアクセス）
            # sltsl=0 が /SLTSL=Low（メモリアクセス候補）
            if io == 0:
                io_count += 1
            if sltsl == 0:
                mem_count += 1

            # DROP警告の表示
            if warn_drop and drop_latched:
                drop_count += 1
                print(
                    f'[!DROP] seq={seq} (drop_latched フラグ検出) @ timestamp={timestamp}us',
                    file=sys.stderr
                )

            # CSVへの書き込み
            csv_writer.writerow([
                seq,
                timestamp,
                f'0x{addr:04X}',
                addr,
                f'0x{data:02X}',
                data,
                f'0x{flags:02X}',
                rd,           # GPIO24 生値（0=Low, 1=High）
                wr,           # GPIO25 生値（0=Low, 1=High）
                io,           # GPIO26 生値（0=Low, 1=High）
                sltsl,        # GPIO27 生値（0=Low, 1=High）
                drop_latched,
                dma_lag,
                drop_seq,
            ])

            seq += 1

    except KeyboardInterrupt:
        # Ctrl+C 終了時のサマリ出力
        elapsed = time.time() - start_time
        rate = seq / elapsed if elapsed > 0 else 0.0
        throughput_kbps = (rate * 12) / 1024.0  # 12バイト/パケット
        print('\n=== 受信サマリ ===')
        print(f'受信イベント数   : {seq}')
        print(f'  うち /IORQ Low（IOアクセス）: {io_count}')
        print(f'  うち /SLTSL Low（MEM候補）  : {mem_count}')
        print(f'フィルタ除外数   : {filter_count}')
        print(f'DROPラッチ検出数 : {drop_count}')
        print(f'経過時間        : {elapsed:.1f} 秒')
        print(f'平均レート      : {rate:.1f} events/sec')
        print(f'スループット     : {throughput_kbps:.1f} KB/sec')
        print(f'保存先(CSV) : {csv_path}')
        if raw_path:
            print(f'保存先(RAW) : {raw_path}')


def main():
    args = parse_args()

    # フィルタ引数の解析
    try:
        io_range = parse_range(args.filter_io) if args.filter_io else None
    except ValueError as e:
        print(f'エラー: --filter-io {e}', file=sys.stderr)
        sys.exit(1)

    try:
        mem_range = parse_range(args.filter_mem) if args.filter_mem else None
    except ValueError as e:
        print(f'エラー: --filter-mem {e}', file=sys.stderr)
        sys.exit(1)

    # --filter-type をセットに変換（大文字正規化）
    # args.filter_type が None の場合は type_filter = None（フィルタなし）
    if args.filter_type:
        type_filter = set(args.filter_type.upper().split(','))
    else:
        type_filter = None

    # 出力ファイル名の決定
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_filename = args.output if args.output else f'msx_log_{timestamp_str}.csv'
    raw_filename = csv_filename.replace('.csv', '.bin') if args.save_raw else None

    print(f'ポート      : {args.port}')
    print(f'保存先(CSV) : {csv_filename}')
    if raw_filename:
        print(f'保存先(RAW) : {raw_filename}')
    print('Ctrl+C で終了します。')
    print()

    # シリアルポートのオープン
    # 注意: Pico の USB CDC では実際のボーレートは無関係（USB で固定速度）だが
    # pyserial のAPIに合わせて 115200 を指定する
    try:
        ser = serial.Serial(args.port, baudrate=115200, timeout=1)
    except serial.SerialException as e:
        print(f'エラー: シリアルポート "{args.port}" を開けません: {e}', file=sys.stderr)
        sys.exit(1)

    try:
        with open(csv_filename, 'w', newline='', encoding='utf-8') as csv_file:
            raw_file_ctx = open(raw_filename, 'wb') if raw_filename else None
            try:
                # CSVヘッダ書き込み
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([
                    'seq', 'timestamp_us', 'addr_hex', 'addr_dec',
                    'data_hex', 'data_dec', 'flags_hex',
                    'rd', 'wr', 'io', 'sltsl', 'drop_latched', 'dma_lag', 'drop_seq',
                ])

                # 受信ループ実行
                receive_loop(ser, csv_writer, raw_file_ctx, args.warn_drop,
                             io_range, mem_range, type_filter,
                             args.no_io, args.no_mem,
                             csv_filename, raw_filename)

            finally:
                if raw_file_ctx:
                    raw_file_ctx.close()
    finally:
        ser.close()


if __name__ == '__main__':
    main()
