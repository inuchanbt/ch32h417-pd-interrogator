# License scope and third-party notices

The root `LICENSE` applies to original code, tools, and documentation contributed
to this project. It does not relicense third-party material or remove conditions
on code derived from third-party sources. Existing file-level notices take
precedence for the material they cover.

## WCH CH32H417 EVT code

`vendor/wch/SRC/` contains WCH support code. `firmware/USBPD_SNK/` is based on
WCH's USBPD sink example and includes WCH-derived code, including modified files.
These portions retain their original notices and terms. They must not be treated
as unrestricted MIT code merely because this repository has a root MIT license.

WCH source headers identify Nanjing Qinheng Microelectronics Co., Ltd. as the
copyright holder and include this condition:

> Attention: This software (modified or not) and binary are used for
> microcontroller manufactured by Nanjing Qinheng Microelectronics.

Refer to the notices in each source file, for example
`vendor/wch/SRC/Core/core_riscv.c` and
`firmware/USBPD_SNK/Common/hardware.c`. Preserve those notices when redistributing
source or derived material. Original project contributions are offered under MIT
only to the extent of the contributors' rights; WCH-derived portions remain
subject to WCH's terms.

## Dependencies

Separately installed dependencies and development tools retain their own licenses.
The root MIT license does not replace those licenses.

## 日本語

ルートのMITライセンスは本プロジェクト独自のコード・ツール・文書に適用します。
`vendor/wch/SRC/` とWCHサンプルを基にした `firmware/USBPD_SNK/` 内のWCH由来部分は、変更済みのものも含めて元の著作権表示・利用条件を維持します。
WCHのヘッダには、ソースとバイナリをWCH製マイコンで使用する条件が記されています。
第三者由来の部分をMITで再ライセンスするものではありません。各ファイルの表示を確認してください。
