# msxpi-pico-playground

This is an unofficial personal hobby project.  
It is NOT affiliated with the original MSXpi project or its author.  
Use at your own risk.

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
````

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
mkdir build
cd build
cmake ..
make -j4
```

成功すると、以下のようなファイルが生成されます：

```
blink.uf2
```

---

### 5. Pico への書き込み

1. Pico を **BOOTSEL モード**で接続

   * BOOTSEL ボタンを押しながら USB 接続
2. Windows 側で USB ドライブとして認識される
3. `blink.uf2` をコピー
4. USB ケーブルを抜き差ししてリセット

Pico の LED が点灯すれば成功です。

---

### 6. 本プロジェクトのビルド

```sh
git clone https://github.com/emef2247/msxpi-pico-playground.git
cd msxpi-pico-playground
mkdir build
cd build
cmake ..
make
ls -l msx_bus_logger.uf2
```

---

## MSXpiをMSXに装着し、MSXpiとHostPC間をUSBケーブルで繋ぐ場合は以下の点にご注意ください
※ 電源逆流を避けるため、下記手順を必ず守ってください。

### 開始時

1. **MSX の電源が OFF の状態で** MSXpi をスロットに挿入
2. MSX の電源を ON
3. その後、Host PC と MSXpi を USB ケーブルで接続

### 終了時

1. USB ケーブルを抜き、Host PC と MSXpi の接続を解除
2. MSX の電源を OFF


---

## MSXpi ハードウェアについて

本ソフトウェアは、以下の MSXpi ハードウェアボードでの動作を想定しています。

* MSXpi hardware by piigaa-densetu-two-dai
  [https://github.com/piigaa-densetu-two-dai/MSXpi/blob/main/MSX%CF%80.pdf](https://github.com/piigaa-densetu-two-dai/MSXpi/blob/main/MSX%CF%80.pdf)

---

## ライセンス・免責

本プロジェクトは個人の趣味による非公式実験コードです。
ハードウェアの破損・データ損失等について、作者は一切の責任を負いません。

```

