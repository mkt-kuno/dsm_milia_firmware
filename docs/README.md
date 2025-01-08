# Modbusボード Yamanin

![Zephyr logo](./logo-readme-dark.svg#gh-dark-mode-only)
![Zephyr logo](./logo-readme-light.svg#gh-light-mode-only)

## 概要
このリポジトリは、土質試験機制御用のModbusボードYamaninのファームウェアを提供します。  
これらのボードは、Modbus RTUプロトコルを使用して制御されるADCおよびDACモジュールを備えています。

Yamaninボードは16chのADC、8chのDAC、4chのLED(WS2812B+Debug)を搭載しており、  
Zephyr RTOS Project を使用しています。


---

## 特徴
- **MCU**: WeAct Studio STM32F411およびピン互換ボード
- **ADC**:
  - **HX711**: 8チャンネル（16ビット、ゲイン128、約10Hzサンプリングレート）
  - **ADS1115**: 8チャンネル（16ビット、±6.114V範囲、チャンネルあたり実効32Hzサンプリングレート）
- **DAC**: GP8403
  - Trio: 6チャンネル（12ビット、0-10V出力、チャンネルあたり最大20mA）
  - Quartet: 8チャンネル（12ビット、0-10V出力、チャンネルあたり最大20mA）
- **Modbus通信**:
  - RTUモード対応
  - ボーレート設定可能: 38400 bps (USBダイレクトのため特に上限下限の指定なし)

---

## ハードウェア設定
### ピンアサイン
- **HX711**: 8チャンネル
- **ADS1115**: I2Cインターフェース（アドレス0x48, 0x49）
- **GP8403**: I2Cインターフェース（アドレス0x58, 0x59, 0x5A, 0x5B）

### 配線図
配線の詳細については、提供された説明書の7ページにある*配線図*を参照してください。

---

## 使用方法
### Modbus RTU設定
ファームウェアはModbus RTUをサポートしています。  
ASCIIについては別途対応となり、デフォルトではRTUが有効になっています。  

#### デフォルト通信設定
- **モード**: RTU
- **スレーブID**: 1
- **ボーレート**: 38400(なんでも)
- **データビット**: 8
- **ストップビット**: 1
- **パリティ**: なし

### 入力レジスタ
| アドレス | レジスタ | 説明                     |
|----------|----------|--------------------------|
| 0-7      | HX711    | ADCチャンネル0-7（int16_t）|
| 8-15     | ADS1115  | ADCチャンネル8-15（int16_t）|

### ホールディングレジスタ
| アドレス | レジスタ | 説明                     |
|----------|----------|--------------------------|
| 0-5      | GP8403   | DACチャンネル（Trio, uint16_t）|
| 0-7      | GP8403   | DACチャンネル（Quartet, uint16_t）|

### Pythonサンプルコード
`pymodbus`を使用したModbus通信のテスト用Pythonスクリプト例:

```python
from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(port='/dev/ttyUSB0', baudrate=38400, method='rtu')
if client.connect():
    # 入力レジスタの読み取り（HX711およびADS1115）
    response = client.read_input_registers(0, 16, unit=1)
    print("ADC Values:", response.registers)

    # ホールディングレジスタへの書き込み（GP8403）
    client.write_registers(0, [0, 2000, 4000, 6000, 8000, 10000])
    print("DAC Values Updated")
    client.close()
```

---

## 高度な設定
### HX711のゲインおよびサンプリングレート
HX711チャンネルは、ゲイン128および80Hzのサンプリングレートで設定されています。　　
読み出したタイミングに応じ、適切に信号処理された最新のAD変換値が取得されます。  

### ADS1115の電圧範囲およびサンプリングレート
ADS1115は、±6.114Vの電圧範囲およびチャンネルあたり実効32Hzのサンプリングレートで設定されています。

---

## 参考文献
詳細なハードウェア仕様および追加情報については、提供された説明書を参照してください。