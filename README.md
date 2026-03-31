# メカナムローバーVer.3.0 ROS2パッケージ

<p align="center">
  <img src="./images/mecanum3.png" width="600" />
</p>

ヴイストン株式会社より発売されている全方向移動台車「[メカナムローバーVer.3.0](https://www.vstone.co.jp/products/wheelrobot/ver.3.0_mecanum.html)」をROS 2で制御するためのパッケージです。別途Linux搭載のPC及びロボット実機が必要になります。

## 概要

このパッケージは、メカナムローバーVer.3.0台車ロボット用のROS 2パッケージを提供します。  
実機制御、SLAM、Navigation2、Gazeboシミュレーションに必要な launch ファイルと設定ファイルを含みます。

## 必要機器 & 開発環境

- メカナムローバーVer.3.0
  - 製品ページ: [https://www.vstone.co.jp/products/wheelrobot/ver.3.0_mecanum.html](https://www.vstone.co.jp/products/wheelrobot/ver.3.0_mecanum.html)
  - 販売ページ: [https://www.vstone.co.jp/robotshop/index.php?main_page=product_info&products_id=5345](https://www.vstone.co.jp/robotshop/index.php?main_page=product_info&products_id=5345)
- Ubuntu Linux - Jammy Jellyfish (22.04)
- ROS 2 Humble Hawksbill

## ファイルの構成

```text
ros2_ws/src
├ mecanumrover3_ros2
│  ├ mecanumrover3
│  ├ mecanumrover3_bringup
│  ├ mecanumrover3_navigation
│  ├ mecanumrover3_gazebo
│  └ mecanumrover_description
├ slam_gmapping
├ vs_rover_options_description
└ ydlidar_ros2_driver
```

## パッケージ構成

- `mecanumrover3`: メカナムローバーVer.3.0のメタパッケージです。
- `mecanumrover3_bringup`: 実機起動や遠隔操作に関する launch を提供します。
- `mecanumrover3_navigation`: SLAM と Navigation2 に関する launch と設定を提供します。
- `mecanumrover_description`: URDF と Gazebo 用のロボットモデルを提供します。
- `mecanumrover3_gazebo`: Gazebo 起動用の launch を提供します。

## インストール方法

このパッケージをインストールするには、以下の手順に従ってください。

[こちら](https://docs.ros.org/en/humble/Installation.html)の手順に従って、ROS 2 Humbleをインストールしてください。

1. [micro-ROS](https://micro.ros.org/) Agent のセットアップ: *(実機を動かす場合のみ必要)*

   ```bash
   mkdir -p ~/uros_ws/src
   cd ~/uros_ws/src
   git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
   cd ~/uros_ws
   rosdep update && rosdep install --from-paths src --ignore-src -y
   colcon build
   source install/local_setup.bash

   ros2 run micro_ros_setup create_agent_ws.sh
   ros2 run micro_ros_setup build_agent.sh
   source install/local_setup.bash
   ```

2. このリポジトリをワークスペースにクローンしてください。

   ```bash
   mkdir -p "$HOME/ros2_ws/src"
   cd "$HOME/ros2_ws/src"
   git clone -b humble_gazebo https://github.com/vstoneofficial/mecanumrover3_ros2.git --recurse-submodules
   git clone -b $ROS_DISTRO https://github.com/vstoneofficial/vs_rover_options_description.git
   sudo xargs -a mecanumrover3_ros2/packages.txt apt install -y
   rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
   ```

3. ワークスペースをビルドします。

   ```bash
   cd "$HOME/ros2_ws"
   colcon build --symlink-install
   ```

4. ワークスペースをオーバーレイします。

   ```bash
   source "$HOME/ros2_ws/install/local_setup.bash"
   ```

5. シェル起動時にワークスペースがオーバーレイされるように設定します。

   ```bash
   echo "source \$HOME/uros_ws/install/local_setup.bash" >> ~/.bashrc
   echo "source \$HOME/ros2_ws/install/local_setup.bash" >> ~/.bashrc
   ```

## 使用方法

このパッケージには、以下の主要な機能が含まれています。詳細は各ファイルを確認してください。

### URDFモデルの表示

以下のコマンドを実行して、メカナムローバーのURDFモデルを表示します。`rover:=` の部分は使用しているロボットに合わせて変更してください。対応モデルは `mecanum3` / `g120a` / `g40a_lb` です。

```bash
ros2 launch mecanumrover_description display.launch.py rover:=mecanum3
```

### メカナムローバー（実機）との通信

ROS 2 と Micro-ROS を統合するためのエージェントノードを起動します。

#### 有線シリアル接続の場合

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 921600 -v4
```

#### Wi-Fi 接続の場合

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### ROS 2トピックとメッセージ

本パッケージで使用している主な ROS 2 トピックとメッセージは以下の通りです。

```text
/rover_twist
  役割: ROS 2 側からロボットへ送る速度指令トピックです。

/rover_odo
  役割: マイコン側から ROS 2 側へ送信されるロボットの自己運動情報です。
  線速度・角速度が含まれます。

/rover_sensor
  役割: マイコン側から ROS 2 側へ送信されるセンサ情報です。
  配列要素:
    - data[0]: デジタル入力の値
    - data[1]: バッテリー電圧値
```

### odometryをpublish

`nav_robot.launch.py` を起動すると、pub_odom と RViz 上の可視化に必要なノードが起動します。

```bash
ros2 launch mecanumrover3_bringup nav_robot.launch.py rover:=mecanum3
```

### 台車ロボットをROS 2経由で遠隔操作

#### キーボードで操作

キーボードを使用してロボットを操作するためのノードを起動します。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=rover_twist
```

#### マウスで操作

マウスを使用してロボットを操作するためのノードを起動します。

```bash
ros2 launch mecanumrover3_bringup mouse_teleop.launch.py
```

### URDFモデルのオプション有効化方法

`mecanumrover_description/urdf/mecanum3.xacro` に定義されている各種オプションは、コメントの切り替えによって有効／無効を設定できます。ここでは、LRF TG30 オプションを例に設定手順を説明します。

1. オプションを有効化します。

   ```xml
   <!-- OPTION: LRF TG30 -->
   <xacro:include filename="$(find vs_rover_options_description)/urdf/lrf/tg30_mecanum3.xacro" />
   ```

2. オプションを無効化します。

   ```xml
   <!-- OPTION: LRF TG30 -->
   <!-- <xacro:include filename="$(find vs_rover_options_description)/urdf/lrf/tg30_mecanum3.xacro" /> -->
   ```

3. 反映を確認します。

   設定を保存後、以下のコマンドでURDFモデルを確認できます。

   ```bash
   ros2 launch mecanumrover_description display.launch.py rover:=mecanum3
   ```

### SLAM

#### SLAM ToolboxでSLAM

SLAM Toolbox は ROS 2 の Navigation2 と一緒に使える標準的なパッケージです。現在も活発に更新されており、広い範囲での地図作成やループ修正にも対応しています。実際にロボットを動かす場面ではこのパッケージの利用を推奨します。

1. SLAM Toolbox をインストールします。

   ```bash
   sudo apt install ros-humble-slam-toolbox
   ```

2. 実機で SLAM する場合は、まずロボットとの通信と odom publish を起動します。

   ```bash
   ros2 launch mecanumrover3_bringup nav_robot.launch.py rover:=mecanum3
   ```

3. 新しいターミナルで SLAM を開始します。

   ```bash
   ros2 launch mecanumrover3_navigation slam.launch.py
   ```

4. 付属の VS-C3 無線コントローラ、または ROS 2 の teleop で動かして mapping します。

- Gazebo で SLAM する場合は、`gazebo_slam.launch.py` を使用します。

```bash
ros2 launch mecanumrover3_gazebo gazebo_slam.launch.py rover:=mecanum3 wall:=Wall.stl
```

- `mecanumrover3_navigation` パッケージの [`config`](./mecanumrover3_navigation/config/) フォルダに、SLAM Toolbox 用のパラメータがあります。
- Gazebo 系 launch では `use_sim_time=true` を使い、実機では `use_sim_time=false` を使います。

#### SLAM gmappingでSLAM

`gmapping` は ROS 1 時代から広く使われてきた SLAM の手法です。ROS 2 版も存在しますが、公式での開発はすでに終了しており、有志による移植パッケージが提供されています。学習や体験目的で、まずは地図作成の流れを試したい場合に使えます。

1. [SLAM gmapping](https://github.com/Project-MANAS/slam_gmapping) を `src` フォルダにクローンしてビルドします。

   ```bash
   git clone https://github.com/Project-MANAS/slam_gmapping.git
   ```

2. 実機で gmapping を使う場合は、まずロボットとの通信と odom publish を起動します。

   ```bash
   ros2 launch mecanumrover3_bringup nav_robot.launch.py rover:=mecanum3
   ```

3. 新しいターミナルで gmapping を開始します。

   ```bash
   ros2 launch mecanumrover3_navigation gmapping.launch.py
   ```

4. 付属の VS-C3 無線コントローラ、または ROS 2 の teleop で動かして mapping します。

- `slam_gmapping` のパラメータは、`slam_gmapping.cpp` にあります。

### 作成した地図の保存方法

下記のコマンドで地図を `mecanumrover3_ros2/mecanumrover3_navigation/maps/` フォルダ内に保存します。

```bash
cd "$HOME/ros2_ws/src/mecanumrover3_ros2"
ros2 run nav2_map_server map_saver_cli -f mecanumrover3_navigation/maps/YOUR_MAP_NAME
```

## Navigation2を使用したナビゲーション

### Nav2パッケージをインストールする

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-rviz-plugins
```

### Nav2でナビゲーションする

地図ファイルは `map:=` 引数で指定してください。launch ファイルの編集は不要です。`map:=` には YAML のフルパスを指定します。

1. 実機でナビゲーションする場合は、まずロボットとの通信と odom publish を起動します。

   ```bash
   ros2 launch mecanumrover3_bringup nav_robot.launch.py rover:=mecanum3
   ```

2. 新しいターミナルでナビゲーションを開始します。

   ```bash
   cd "$HOME/ros2_ws/src/mecanumrover3_ros2"
   ros2 launch mecanumrover3_navigation navigation.launch.py rover:=mecanum3 map:=mecanumrover3_navigation/maps/YOUR_MAP_NAME.yaml
   ```

- `mecanumrover3_navigation` パッケージの [`config`](./mecanumrover3_navigation/config/) フォルダに、Navigation2 用のパラメータがあります。
- rover ごとにパラメータファイルが分かれています。使用しているロボットに合わせて調整してください。

### Gazebo

#### gazeboシミュレーションの準備

- 初回の Gazebo 起動前に環境変数を読み込みます。

  ```bash
  GAZEBO_SETUP_PATH=<gazebo_setup.sh path>
  source "$GAZEBO_SETUP_PATH"
  ```

- シェル起動時にも反映されるようにします。

  ```bash
  echo "source \$GAZEBO_SETUP_PATH" >> "$HOME/.bashrc"
  ```

#### gazeboでシミュレーションする

Gazebo 系 launch では `use_sim_time=true` を使います。
- `gazebo_bringup.launch.py` は `gui` と `physics` を受け取ります。`gui:=false` で GUI を無効化し、`physics:=ode` などを切り替えられます。
- `gazebo_nav.launch.py` は `gui`、`verbose`、`wall`、`map` を受け取ります。`wall` は空文字でも起動できますが、壁モデルを使う場合は STL 名を指定してください。
- `gazebo_slam.launch.py` は `gui` と `wall` を受け取ります。

##### 空のワールドを起動する

床とロボットモデルのみ存在するベースワールドです。他プログラムと連携する際に利用することを想定した launch ファイルです。

1. 空のワールドを起動します。

   ```bash
   ros2 launch mecanumrover3_gazebo gazebo_bringup.launch.py rover:=mecanum3
   ```

2. 壁をシミュレータ内にインポートします。`wall:=Wall.stl` の部分は、`mecanumrover3_gazebo/models` 内にある STL ファイルに書き換えることで、壁としてシミュレータ内にインポートできます。

   ```bash
   ros2 launch mecanumrover3_gazebo spawn_wall.launch.py wall:=Wall.stl
   ```

3. ローバーをマウスで遠隔操作するには、新しいターミナルで以下のコマンドを使用して遠隔操作ノードを起動します。

   ```bash
   ros2 launch mecanumrover3_bringup mouse_teleop.launch.py
   ```

##### gazeboでSLAMする

Gazebo 内のワールドの地図データを作成できます。

1. Gazebo server と SLAM ノードを起動します。

   ```bash
   ros2 launch mecanumrover3_gazebo gazebo_slam.launch.py rover:=mecanum3 wall:=Wall.stl
   ```

2. ローバーをマウスで遠隔操作するには、新しいターミナルで以下のコマンドを使用して遠隔操作ノードを起動します。

   ```bash
   ros2 launch mecanumrover3_bringup mouse_teleop.launch.py
   ```

3. 下記のコマンドで地図を `mecanumrover3_ros2/mecanumrover3_gazebo/maps/` フォルダ内に保存します。

   ```bash
   cd "$HOME/ros2_ws/src/mecanumrover3_ros2"
   ros2 run nav2_map_server map_saver_cli -f mecanumrover3_gazebo/maps/YOUR_MAP_NAME
   ```

##### gazeboでナビゲーションする

使用する地図は `map:=` 引数で指定してください。launch ファイルの編集は不要です。`map:=` には YAML のフルパスを指定します。

```bash
cd "$HOME/ros2_ws/src/mecanumrover3_ros2"
ros2 launch mecanumrover3_gazebo gazebo_nav.launch.py rover:=mecanum3 wall:=Wall.stl map:=mecanumrover3_gazebo/maps/YOUR_MAP_NAME.yaml
```

- `gazebo_nav.launch.py` は `use_sim_time:=true` がデフォルトです。
- ナビゲーションを行う場合、移動命令の `rover_twist` トピックが競合しないように、`rover_twist` を publish するノードは起動しないでください。

## ライセンス

このパッケージは Apache 2.0 ライセンスの下で提供されています。詳細については、[LICENSE](./LICENSE) ファイルを参照してください。

## 貢献

バグの報告や機能の提案など、このパッケージへの貢献は大歓迎です。プルリクエストやイシューを使用して issue をご利用ください。
