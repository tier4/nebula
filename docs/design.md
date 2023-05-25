# Nebula requirements

The driver follows the following functional requirements:
ドライバは、次の機能的な要求に応えなければならない。

## Hardware interface independent

The data acquisition and communication with the sensor must not be bound to a specific hardware interface.
センサでのデータ取得と通信は、特定のハードウェアに縛られてはならない。

### Why?

Writing a general-purpose driver independent of the interface will support different types of hardware interfaces.
インターフェイスに依存しない汎用ドライバを書くことで、異なったタイプのハードウェアインターフェイスに対応することができる。

## Sensor Control

The driver should obtain, set, and confirm the desired sensor configuration at launch, i.e., scan frequency, synchronization methods, etc.
ドライバは起動時に、望ましいセンサ設定（スキャン周波数、同期方法等）を取得、設定、確認できる必要がある。

### Why?

The sensor control will ensure that the sensor works in the expected mode as it was initially intended.
これによって、センサが当初より意図していたモードで動いていることが確認できる。

## Configurable output cloud

The ROS wrapper should be able to define the desired output format of the point cloud. i.e., customize the fields to be contained in the final output cloud.
ROS ラッパーは点群の望ましい出力フォーマットで定義できなければならない（最終的な出力クラウドに含まれるフィールドをカスタマイズできる）
In addition, it should be able to generate the corresponding 2D range image for the output point cloud.

さらに、出力された点群を対応する二次元レンジ画像に生成することができなければならない。
If the sensor allows it, have an option to add a minimum and maximum range distance as an option. For instance, if the user launches the driver with a range limit set to 0, the driver will not perform any filtering.
センサが許可している場合は、オプションとして最小値と最大値のレンジ距離を追加することができる。例えば、ユーザーが範囲制限を 0 に設定してドライバーを起動した場合、ドライバはフィルタリングを行わない。
The driver should have an option to define if the output cloud is of a fixed size or dynamic size. If a fixed size is selected, the output cloud must include NaN values for those lasers without any return. If a dynamic size is selected, the lasers with no valid returns are to be removed.
ドライバは、出力クラウドを固定されたサイズか、もしくはダイナミックサイズにするかを定義するオプションが必要である。固定されたサイズを選択した場合、出力クラウドは、リターンのないレーザの NaN 値を含まなければならない。ダイナミックサイズを選択した場合、リターンのないレーザは削除される。
Default behavior: Dynamic, remove invalid laser returns (NaN).
デフォルトの動作：ダイナミック、無効なレーザリターンを削除。（NaN)

### Why?

The configurable output will allow the generation of the point cloud according to the expected use case application.
これによって、予定したユースケースアプリケーションにより必要な出力を生成することができる。

## Sensor Metadata

The driver should include the following metadata for each generated point cloud:
ドライバは、生成された点群それぞれに、次のメタデータが含まれなければならない：

### Calibration data キャリブレーションデータ

Contains the sensor calibration parameters used to generate the point cloud from the raw data.
生データから点群を生成するのに使用したセンサキャリブレーションパラメータ

### Sensor settings センサ設定

The configuration mode in which the sensor is being executed:
センサが実行された設定モード

### Synchronization mode (PTP, PPS/NMEA)　同期モード(PTP, PPS/NMEA)

Sensor Type
センサタイプ
Sensor Model
センサモデル
Scan frequency
スキャン周波数

### Why?

The processed point cloud or the raw data does not always contain the state on which the sensor was run. Moreover, having this information at hand will help identify, classify and understand the situation of the recorded data.
処理した点群、つまり生データは、常にセンサが作動した状態を含んでいるとは限らない。手元に情報があることは、記録データの状況を素早く把握し、分類し、理解することの助けになる。

## Multi echo compatible

The sensor should consider the possibility of the future inclusion of more than two echos.
センサは２つ以上のエコーが将来的に含まれる可能性があることを考慮すべきである。

### Why?

Multi-echo support will help to future-proof the driver according to new features included or used in new sensors. In addition, multiple echo support has been proven to improve sensor resilience against weather conditions such as rain, fog, and snow.
これによって、新しいセンサに搭載された、もしくは、開発された新機能によって、ドライバの将来性を高めることができる。複数のエコーに対応することで、雨や霧、雪などの天候条件に対して、センサの回復力が改善されることが証明されている。

## ROS independent

The objects used inside the driver must be ROS independent.
ドライバ内に使用されている API 等の対象物は、ROS に依存してはならない。

### Why?

The third-party dependency reduction allows any software to be quickly updated without waiting for external dependencies to be updated.
これにより、どんなソフトウェアでも、アップデートのための外部依存を待つ必要がなく、素早くアップデートすることができる。

## Offline ready

The data parser API inside the driver should not be designed to expect the data to be received in a real-time stream.
ドライバ内のデータパーサ API は、データをリアルタイムストリームで受け取るように設計されるべきではない。

### Why?

Offline processing will help process the data faster than in real-time.
これにより、データがセンサの代わりにログファイルから発信された時にデータ処理をリアルタイムより速く処理することができる。
