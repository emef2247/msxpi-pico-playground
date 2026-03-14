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

# パケット構造: AA 55 FLAGS DROP_SEQ ADDR_LO ADDR_HI DATA 00 TS0 TS1 TS2 TS3
# struct フォーマット: '<2sBBHBBI'
# (magic:2s, flags:B, drop_seq:B, addr:H, data:B, reserved:B, timestamp:I)
FMT = '<2sBBHBBI'

# FLAGSビット定義
FLAG_RD           = 1 << 0  # /RD アサート
FLAG_WR           = 1 << 1  # /WR アサート
FLAG_IO           = 1 << 2  # /IORQ アサート
FLAG_DROP_LATCHED = 1 << 4  # DROP ラッチフラグ
FLAG_DMA_LAG      = 1 << 6  # DMA 遅延フラグ


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


def receive_loop(ser, csv_writer, raw_file, warn_drop):
    """メイン受信ループ

    パケットを受信してCSV書き込みおよびコンソール表示を行う。
    Ctrl+C で終了し、サマリを表示する。
    """
    seq = 0
    drop_count = 0
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

            # FLAGSからビットを展開
            rd           = 1 if (flags & FLAG_RD)           else 0
            wr           = 1 if (flags & FLAG_WR)           else 0
            io           = 1 if (flags & FLAG_IO)           else 0
            drop_latched = 1 if (flags & FLAG_DROP_LATCHED) else 0
            dma_lag      = 1 if (flags & FLAG_DMA_LAG)      else 0

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
                rd,
                wr,
                io,
                drop_latched,
                dma_lag,
                drop_seq,
            ])

            seq += 1

    except KeyboardInterrupt:
        # Ctrl+C 終了時のサマリ出力
        elapsed = time.time() - start_time
        rate = seq / elapsed if elapsed > 0 else 0.0
        print('\n=== 受信サマリ ===')
        print(f'受信イベント数   : {seq}')
        print(f'DROPラッチ検出数 : {drop_count}')
        print(f'経過時間        : {elapsed:.1f} 秒')
        print(f'平均レート      : {rate:.1f} events/sec')


def main():
    args = parse_args()

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
                    'rd', 'wr', 'io', 'drop_latched', 'dma_lag', 'drop_seq',
                ])

                # 受信ループ実行
                receive_loop(ser, csv_writer, raw_file_ctx, args.warn_drop)

            finally:
                if raw_file_ctx:
                    raw_file_ctx.close()
    finally:
        ser.close()

    print(f'保存先(CSV) : {csv_filename}')
    if raw_filename:
        print(f'保存先(RAW) : {raw_filename}')


if __name__ == '__main__':
    main()
