# CYBER TRACE PAD — ファームウェアソースコード

本製品のファームウェアのソースコード全文と、ライセンス情報の公開ページです。
製品に同梱の説明書、または製品の詳細説明ページからリンクされています。

## 使用しているオープンソースソフトウェア

**megaTinyCore v2.6.11**(コアおよび Wire ライブラリを含む)

- Copyright (c) 2018-2023 Spence Konde and contributors
- Wire: Copyright (c) 2006 Nicholas Zambetti、Todd Krein、Chuck Todd、Spence Konde、MX682X による改変
- コア: Arduino(Copyright (c) 2005-2006 David A. Mellis)に由来し、megaTinyCore 向けに改変されたもの
- 配布元: https://github.com/SpenceKonde/megaTinyCore

これらは **GNU 劣等一般公衆利用許諾契約書 バージョン 2.1(LGPL-2.1)** に
基づいて使用しています。

### ライセンス全文

- https://github.com/SpenceKonde/megaTinyCore/blob/master/LICENSE.md
- https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html

## ソースコードの提供について

LGPL-2.1 が求める再リンクの権利を保証するため、本製品のファームウェアの
ソースコードを本リポジトリで公開しています。利用条件は [LICENSE](LICENSE) を
ご確認ください。

また、**本製品の最終出荷日から3年間**、上記ソフトウェアのソースコード
および再リンクに必要な資料を、頒布実費にてご提供します。ご希望の方は
下記までご連絡ください。

- お問い合わせ: `suilab.info(atmark)gmail.com`

## 製品ファームウェアとの対応

| 製品ファームウェア | 本リポジトリのタグ |
|---|---|
| Ver.1（量産初回ロット〜） | `v1.0` |
| Ver.1.1 | `v1.1` |

お手元の製品に対応するソースコードは上記タグから取得できます
（[Tags](../../tags) ページからZIP形式でダウンロード可能です）。
`main` ブランチは次版の開発状態を含むことがあります。

## ビルド方法(再リンク手順)

[PlatformIO](https://platformio.org/) を使用します。megaTinyCore は
PlatformIO が自動で取得しますが、改変したものと差し替える場合は
`~/.platformio/packages/framework-arduino-megaavr-megatinycore/` を
置き換えてください。

```bash
pip install platformio
cd touch_trail
pio run
```

生成物は `.pio/build/t1616/firmware.hex` です。書き込みは UPDI
(SerialUPDI, 230400 baud)で行います。

- MCU: ATtiny1616 @ 20 MHz(内蔵発振器)
- ビルド確認バージョン: megaTinyCore v2.6.11

## 収録内容

| ファイル | 内容 |
|---|---|
| `touch_trail/touch_trail.ino` | メインループ(タッチ→描画→表示) |
| `touch_trail/touch4x4.h` | 4×4 自己容量タッチ検出 |
| `touch_trail/shapefx.h` | ジェスチャー認識とエフェクト |
| `touch_trail/is31fl3731.h` | LEDドライバ(IS31FL3731) |
| `touch_trail/led_map.h` | LED座標→レジスタ変換表、ガンマ補正表 |
| `touch_trail/platformio.ini` | ビルド設定 |

本リポジトリはライセンス順守のためのソースコード公開を目的としており、
基板データ等のハードウェア情報は含まれません。
