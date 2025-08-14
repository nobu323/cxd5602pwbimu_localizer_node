# CXD5602PWBIMU Localizer Node

## 概要

このROS 2ノードは、シリアルポートを介してSpresenseボード（`cxd5602pwbimu_localizer_arduino`ファームウェアが書き込まれたもの）からIMUデータを受信し、ROS 2トピックとしてパブリッシュします。また、TF変換もブロードキャストします。

## 依存関係

このパッケージは以下のライブラリ・パッケージに依存します。

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`
- `std_srvs`
- `libserial-dev` (システム依存)

## パラメータ

- `serial_port` (string, default: "/dev/ttyUSB0"): Spresenseが接続されているシリアルポート。
- `baud_rate` (int, default: 1152000): シリアル通信のボーレート。
- `tf_parent_frame` (string, default: "world"): TF変換の親フレームID。
- `tf_child_frame` (string, default: "sensor"): TF変換の子フレームID。

## パブリッシュするトピック

- `imu/data` (`sensor_msgs/msg/Imu`):
  補正済みのIMUデータ（重力加速度除去後の加速度、推定された姿勢など）。
- `imu/data_raw` (`sensor_msgs/msg/Imu`):
  IMUセンサからの生データに近い加速度データ。
- `pose` (`geometry_msgs/msg/Pose`):
  Spresense上で推定された位置と姿勢。

## ブロードキャストするTF

- `tf_parent_frame` -> `tf_child_frame`:
  `pose`トピックと同じ位置・姿勢情報を持つTF変換。

## 提供するサービス

- `/cxd5602pwbimu_localizer_node/reset_pose` (`std_srvs/srv/Trigger`):
  このサービスを呼び出すと、Spresenseボードにリセットコマンドが送信され、ボード上の姿勢・位置推定がリセットされます。

  **呼び出し例:**
  ```bash
  ros2 service call /cxd5602pwbimu_localizer_node/reset_pose std_srvs/srv/Trigger
  ```

## 実行方法

このノードは、通常、Launchファイルから他のノードと共に起動されます。
個別に実行する場合は、`ros2 run`コマンドを使用します。

```bash
ros2 run cxd5602pwbimu_localizer_node localizer_node --ros-args -p serial_port:=/dev/your_spresense_port
```