# msxpi-pico-playground

This is an unofficial personal hobby project.  
It is NOT affiliated with the original MSXpi project or its author.  
Use at your own risk.

---

## Projects

### msx_bus_logger — MSX Bus Logger
MSXpi（Raspberry Pi Pico ベースの MSX カートリッジ）とピン互換のハードを使って、MSX の IO バスアクセスをリアルタイムにキャプチャ・記録します

PIO（Programmable IO）と DMA を活用し、Z80 のバスサイクルを取りこぼしなくキャプチャします。
キャプチャしたデータは USB CDC 経由で PC に転送し、CSV ファイルに保存します。

#### 特徴

- `/IORQ` トリガーによる IO アクセス全キャプチャ
- アドレス・データ・制御線（/RD, /WR, /IORQ, /SLTSL）を 32bit で同時サンプリング
- DROP 検出（DMA バッファオーバーラン時に FLAGS bit4 へラッチ、LED で通知）
- LED による DROP 状態のリアルタイム表示
  - 消灯：直近 1 秒で DROP なし
  - 低速点滅：DROP あり（軽微）
  - 高速点滅：DROP あり（重大）
- バイナリ転送モード（TEXT_MODE=0）と テキストモード（TEXT_MODE=1）切り替え対応
- Python 受信スクリプトによるデバイス別集計・統計出力

#### ハードウェア構成

```
MSX スロット
  └── MSXpi (Raspberry Pi Pico)
        ├── GPIO0-15  : A0-A15  アドレスバス
        ├── GPIO16-23 : D0-D7   データバス
        ├── GPIO24    : /RD
        ├── GPIO25    : /WR
        ├── GPIO26    : /IORQ
        ├── GPIO27    : /SLTSL
        ├── GPIO28    : /WAIT
        └── USB       : Host PC（データ転送）
```

回路図: [MSXπ.pdf](https://github.com/piigaa-densetu-two-dai/MSXpi/blob/main/MSX%CF%80.pdf)

#### ファイル構成

```
msx_bus_logger/
├── CMakeLists.txt
└── src/
    ├── main.c                  Pico ファームウェア本体
    ├── msx_bus_logger.pio      PIO プログラム（/IORQ トリガー）
    └── tools/
        └── msx_logger.py       PC側 受信・集計スクリプト
```

#### ビルドと書き込み

```sh
cd msxpi-pico-playground
mkdir build && cd build
cmake ..
make
# → msx_bus_logger/msx_bus_logger.uf2 が生成される
```

Pico を BOOTSEL モードで接続し、`msx_bus_logger.uf2` を書き込んでください。

#### 受信スクリプトの使い方

```sh
pip install pyserial
python msx_bus_logger/tools/msx_logger.py --port COM8
```

オプション：

| オプション | 説明 | デフォルト |
|------------|------|-----------|
| `--port`   | シリアルポート | `COM8` |
| `--output` | CSV 出力ファイル名 | 自動生成 |
| `--warn-drop` | DROP 警告をコンソール表示 | OFF |
| `--save-raw`  | バイナリを `.bin` にも保存 | OFF |

Ctrl+C で停止するとサマリが表示されます：

```
=== Capture Summary ===
Total Events             :    3,046,543
  IO Access              :    3,046,543
    VDP RD (0x98-0x9B)   :            0
    VDP WR (0x98-0x9B)   :    2,777,035
    PSG RD (0xA2)        :            0
    PSG WR (0xA0-0xA1)   :      220,000
    OPLL RD (0x7E)       :            0
    OPLL WR (0x7C-0x7D)  :           36
    MSX-Audio RD (0xC1)  :            0
    MSX-Audio WR (0xC0-0xC3):          0
    SCC/SCC-I            :     (MEM_WR)
DROP Latch               :            0
DROP Sequence            :            0
Elapsed Time             :      154.3 sec
Bus Event Rate           :   19,745.7 events/sec  Avg
                         :   27,040.5 events/sec  Max
Throughput               :      231.4 KB/sec     Avg
                         :      316.9 KB/sec     Max
Output (CSV)             : msx_log_20260315_000105.csv
```

> **Note**: SCC / SCC-I はメモリマップドデバイスのため、現バージョンの IO ロガーでは捕捉できません。将来の MEM_WR トレース機能で対応予定です。

#### パケットフォーマット（バイナリモード）

1 イベント = 12 バイト固定長

| Byte | 内容 |
|------|------|
| 0    | マジック 0xAA |
| 1    | マジック 0x55 |
| 2    | FLAGS |
| 3    | DROP_SEQ |
| 4-5  | ADDR (little-endian) |
| 6    | DATA |
| 7    | 予約 (0x00) |
| 8-11 | タイムスタンプ (uint32_t, μs) |

FLAGS ビット��義：

| bit | 意味 |
|-----|------|
| 0   | /RD アサート |
| 1   | /WR アサート |
| 2   | /IORQ アサート |
| 4   | DROP_LATCHED（過去に DROP 発生） |
| 6   | DMA_LAG |

---

## 開発環境の立ち上げ

以下は **WSL / Ubuntu 22.04** を想定した手順です。

### 1. 必要パッケージのインストール

```sh
sudo apt update
sudo apt install -y \
  git \
  cmake \
  gcc-arm-none-eabi \
  build-essential \
  libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib \
  python3 \
  python3-pip
```

---

### 2. pico-sdk の取得

作業ディレクトリは任意ですが、ここでは `~/pico` を使用します。

```sh
mkdir -p ~/pico
cd ~/pico
git clone https://github.com/raspberrypi/pico-sdk.git
```

サブモジュールの取得（必須）：

```sh
cd pico-sdk
git submodule update --init
```

---

### 3. 環境変数の設定

#### 一時的（ターミナルを閉じるまで）

```sh
export PICO_SDK_PATH=~/pico/pico-sdk
```

#### 永続化（おすすめ）

```sh
echo 'export PICO_SDK_PATH=$HOME/pico/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

確認：

```sh
echo $PICO_SDK_PATH
```

---

### 4. 動作確認（blink サンプル）

公式サンプルでビルド環境が正しいか確認します。

```sh
cd ~/pico
git clone https://github.com/raspberrypi/pico-examples.git
cd pico-examples
mkdir build && cd build
cmake ..
make -j4
```

成功すると `blink.uf2` が生成されます。

---

### 5. Pico への書き込み

1. Pico を **BOOTSEL モード**で接続（BOOTSEL ボタンを押しながら USB 接続）
2. Windows 側で USB ドライブとして認識される
3. `.uf2` ファイルをコピー
4. 自動的にリセットされ起動

---

### 6. 本プロジェクトのビルド

```sh
git clone https://github.com/emef2247/msxpi-pico-playground.git
cd msxpi-pico-playground
mkdir build && cd build
cmake ..
make
ls -l msx_bus_logger/msx_bus_logger.uf2
```

---

## 接続時の注意事項

MSXpi を MSX に装着し、MSXpi と Host PC 間を USB ケーブルで繋ぐ場合は以下の手順を守ってください。  
※ 電源逆流を避けるため、順序が重要です。

### 開始時

1. **MSX の電源が OFF の状態で** MSXpi をスロットに挿入
2. MSX の電源を ON
3. その後、Host PC と MSXpi を USB ケーブルで接続

### 終了時

1. USB ケーブルを抜き、Host PC と MSXpi の接続を解除
2. MSX の電源を OFF


---

## ライセンス・免責

本プロジェクトは個人の趣味による非公式実験コードです。  
ハードウェアの破損・データ損失等について、作者は一切の責任を負いません。