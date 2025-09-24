# メカナムローバーVer.3.0 ROS2パッケージ

<p align="center">
  <img src="./images/mecanum3.png" width="600" />
</p>

ヴイストン株式会社より発売されている全方向移動台車「[メカナムローバーVer.3.0](https://www.vstone.co.jp/products/wheelrobot/ver.3.0_mecanum.html)」をROS 2で制御するためのパッケージです。別途Linux搭載のPC及びロボット実機が必要になります。

# 目次
<!-- TOC -->

- [概要](#%E6%A6%82%E8%A6%81)
- [必要機器 & 開発環境](#%E5%BF%85%E8%A6%81%E6%A9%9F%E5%99%A8--%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83)
- [パッケージ構成](#%E3%83%91%E3%83%83%E3%82%B1%E3%83%BC%E3%82%B8%E6%A7%8B%E6%88%90)
- [インストール方法](#%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB%E6%96%B9%E6%B3%95)
- [使用方法](#%E4%BD%BF%E7%94%A8%E6%96%B9%E6%B3%95)
    - [URDFモデルの表示](#urdf%E3%83%A2%E3%83%87%E3%83%AB%E3%81%AE%E8%A1%A8%E7%A4%BA)
    - [メカナムローバー（実機）との通信](#%E3%83%A1%E3%82%AB%E3%83%8A%E3%83%A0%E3%83%AD%E3%83%BC%E3%83%90%E3%83%BC%E5%AE%9F%E6%A9%9F%E3%81%A8%E3%81%AE%E9%80%9A%E4%BF%A1)
        - [有線シリアル接続の場合](#%E6%9C%89%E7%B7%9A%E3%82%B7%E3%83%AA%E3%82%A2%E3%83%AB%E6%8E%A5%E7%B6%9A%E3%81%AE%E5%A0%B4%E5%90%88)
        - [Wi-Fi 接続の場合](#wi-fi-%E6%8E%A5%E7%B6%9A%E3%81%AE%E5%A0%B4%E5%90%88)
    - [odometryをpublish](#odometry%E3%82%92publish)
    - [台車ロボットをROS 2経由で遠隔操作](#%E5%8F%B0%E8%BB%8A%E3%83%AD%E3%83%9C%E3%83%83%E3%83%88%E3%82%92ros-2%E7%B5%8C%E7%94%B1%E3%81%A7%E9%81%A0%E9%9A%94%E6%93%8D%E4%BD%9C)
        - [キーボードで操作](#%E3%82%AD%E3%83%BC%E3%83%9C%E3%83%BC%E3%83%89%E3%81%A7%E6%93%8D%E4%BD%9C)
        - [マウスで操作](#%E3%83%9E%E3%82%A6%E3%82%B9%E3%81%A7%E6%93%8D%E4%BD%9C)
    - [SLAM ToolboxでSLAM](#slam-toolbox%E3%81%A7slam)
    - [SLAM gmappingでSLAM](#slam-gmapping%E3%81%A7slam)
    - [作成した地図の保存方法](#%E4%BD%9C%E6%88%90%E3%81%97%E3%81%9F%E5%9C%B0%E5%9B%B3%E3%81%AE%E4%BF%9D%E5%AD%98%E6%96%B9%E6%B3%95)
    - [Navigation2を使用したナビゲーション](#navigation2%E3%82%92%E4%BD%BF%E7%94%A8%E3%81%97%E3%81%9F%E3%83%8A%E3%83%93%E3%82%B2%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3)
- [ライセンス](#%E3%83%A9%E3%82%A4%E3%82%BB%E3%83%B3%E3%82%B9)
- [貢献](#%E8%B2%A2%E7%8C%AE)

<!-- /TOC -->

## 概要

メカナムローバーは、前後・左右・回転を組み合わせた自由度の高い移動ができることが大きな特徴で、細かい位置制御が求められる搬送・自律移動などの研究開発プロジェクトに好適です。このパッケージでは、メカナムローバーの制御を実現するためのROS2関連の機能を提供します。

## 必要機器 & 開発環境
- メカナムローバーVer.3.0:
  - 製品ページ: [https://www.vstone.co.jp/products/wheelrobot/ver.3.0_mecanum.html](https://www.vstone.co.jp/products/wheelrobot/ver.3.0_mecanum.html)
  - 販売ページ: [https://www.vstone.co.jp/robotshop/index.php?main_page=product_info&products_id=5345](https://www.vstone.co.jp/robotshop/index.php?main_page=product_info&products_id=5345)
- Ubuntu Linux - Jammy Jellyfish (22.04)
- ROS 2 Humble Hawksbill

## ファイルの構成
   ```
    ros2_ws/src
    ├ mecanumrover3_ros2
    │　├ mecanumrover3
    │　├ mecanumrover3_bringup
    │　├ mecanumrover3_navigation
    │　└ mecanumrover_description
    ├ slam_gmapping
    ├ vs_rover_options_description
    └ ydlidar_ros2_driver
   ```

## パッケージ構成
- `mecanumrover3`: メカナムローバーVer.3.0のメタパッケージ。
- `mecanumrover3_bringup`: メカナムローバーVer.3.0の起動に関連するノードやlaunchファイルを提供します。
- `mecanumrover3_navigation`: メカナムローバーVer.3.0のSLAMやnavigationに関連するノードやlaunchファイルを含んでいるパッケージです。
- `mecanumrover_description`: メカナムローバーVer.3.0の物理モデルやURDFモデルを含んでいるパッケージです。

## インストール方法

このパッケージをインストールするには、以下の手順に従ってください。

1. [こちら](https://docs.ros.org/en/humble/Installation.html)の手順に従って、ROS 2 Humbleをインストールしてください。
2. [micro-ROS](https://micro.ros.org/) Agent のセットアップ: *(実機を動かす場合のみ必要)*

   ```bash
   $ mkdir -p ~/uros_ws/src
   $ cd ~/uros_ws/src
   $ git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
   $ cd ~/uros_ws
   $ rosdep update && rosdep install --from-paths src --ignore-src -y
   $ colcon build
   $ source install/local_setup.bash

   $ ros2 run micro_ros_setup create_agent_ws.sh
   $ ros2 run micro_ros_setup build_agent.sh
   $ source install/local_setup.bash
   ```

3. このリポジトリをワークスペースにクローンしてください:

   ```bash
   $ mkdir -p ~/ros2_ws/src
   $ cd ~/ros2_ws/src
   $ git clone https://github.com/vstoneofficial/mecanumrover3_ros2.git --recurse-submodules
   $ git clone -b $ROS_DISTRO https://github.com/vstoneofficial/vs_rover_options_description.git  # オプションを表示するため
   $ rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
   ```

4. ワークスペースをビルド:

   ```bash
   $ cd ~/ros2_ws
   $ colcon build --symlink-install
   ```

5. ワークスペースのオーバレイ作業:

   ```bash
   $ source ~/ros2_ws/install/local_setup.bash
   ```

6. シェルを起動時にワークスペースがオーバーレイされるように設定します。

   ```bash
   $ echo "source ~/uros_ws/install/local_setup.bash" >> ~/.bashrc 
   $ echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
   ```

## 使用方法

このパッケージには、以下の主要な機能が含まれています。（詳細は各ファイルを確認してください）

### URDFモデルの表示
以下のコマンドを実行して、メカナムローバーのURDFモデルを表示します。
   ```
   ros2 launch mecanumrover_description display.launch.py
   ```

### メカナムローバー（実機）との通信
  ROS 2とMicro-ROSを統合するためのエージェントノードを起動。
  
#### 有線シリアル接続の場合
   ```
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 921600 -v4
   ```
      
#### Wi-Fi 接続の場合
   ```
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

### odometryをpublish
pub_odomノードとrviz上可視化
   ```
   ros2 launch mecanumrover3_bringup robot.launch.py
   ```

### 台車ロボットをROS 2経由で遠隔操作
#### キーボードで操作
キーボードを使用してロボットを操作するためのノードを起動。
   ```
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=rover_twist
   ```

#### マウスで操作
マウスを使用してロボットを操作するためのノードを起動。
   ```
   ros2 launch mecanumrover3_bringup mouse_teleop.launch.py
   ```

### SLAM ToolboxでSLAM
  ToolboxはROS2のナビゲーション機（Nav2）と一緒に使える標準的なパッケージです。  
  現在も活発に更新が続いており、広い範囲での地図作成やループ修正にも対応しています。  
  実際にロボットを動かす場面ではこのパッケージの利用を推奨します。
  
1. LRFオプションTG30
   - [ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver.git)を`src`フォルダにクローンして、buildしてください。
      ```
      git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
      ```
   - LRFに関するパラメータは[TG30.yaml](./mecanumrover3_bringup/params/TG30.yaml)にあります。

2. SLAM Toolboxの準備
   下記のコマンドでSLAM Toolboxをインストールする。
      ```
      sudo apt install ros-humble-slam-toolbox
      ```

3. SLAM ToolboxでSLAMする。
   - ロボット実機と通信できたら、以下のコマンドでodom publisher、ロボットのurdf表示、LiDAR関連のlaunchファイルを起動します。
      ```
      ros2 launch mecanumrover3_bringup nav_robot.launch.py
      ```

   - 新しいターミナルで以下のコマンドを使用してSLAMを開始します。
      ```
      ros2 launch mecanumrover3_navigation slam.launch.py
      ```
      付属のVS-C3無線コントローラもしくはROS 2のteleopで動かして、mappingする。

   - メカナムローバーVer.3.0用のSLAM Toolboxのパラメータは`mecanumrover3_navigation`パッケージの[`config`](./mecanumrover3_navigation/config/) フォルダにあります。

### SLAM gmappingでSLAM
  gmappingはROS1時代から広く使われてきたSLAMの手法です。  
  ROS2版も存在しますが公式での開発はすでに終了しており、有志による移植パッケージが提供されています。  
  そのため十分なサポートは期待できず、実用利用にはあまり適していません。
  ただし学習や体験を目的に「まずは地図を作る流れを試してみたい」といった場合には活用できます。

1. [SLAM gmapping](https://github.com/Project-MANAS/slam_gmapping)を`src`フォルダにクローンして、buildしてください。

      ```
      git clone https://github.com/Project-MANAS/slam_gmapping.git
      ```

2. SLAM gmappingでSLAMする。
   - ロボット実機と通信できたら、以下のコマンドでodom publisher、ロボットのurdf表示、LiDAR関連のlaunchファイルを起動します。
      ```
      ros2 launch mecanumrover3_bringup nav_robot.launch.py
      ```

   - 新しいターミナルで以下のコマンドを使用してSLAMを開始します。
      ```
      ros2 launch mecanumrover3_navigation gmapping.launch.py
      ```
      付属のVS-C3無線コントローラもしくはROS 2のteleopで動かして、mappingする。

   - メカナムローバーVer.3.0用のSLAM gmappingのパラメータは[`slam_gmapping.cpp`](https://github.com/Project-MANAS/slam_gmapping/blob/eloquent-devel/slam_gmapping/src/slam_gmapping.cpp)にあります。\
   [ROS 2 workspaceにクローンした場合](../slam_gmapping/slam_gmapping/src/slam_gmapping.cpp)
   - 変更下パラメータは下記のようになります。
      ```cpp
      base_frame_ = "base_footprint";
      maxUrange_ = 29.9;  maxRange_ = 30.0;
      minimum_score_ = 500;
      ```

### 作成した地図の保存方法
下記のコマンドで地図を`mecanumrover3_ros2/mecanumrover3_navigation/maps/`フォルダ内に保存する。
   ```
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/mecanumrover3_ros2/mecanumrover3_navigation/maps/YOUR_MAP_NAME
   ```

### Navigation2を使用したナビゲーション
1. Nav2パッケージをインストールする。
   ```
   $ sudo apt install ros-humble-navigation2
   $ sudo apt install ros-humble-nav2-bringup
   ```

2. Nav2でナビゲーションする。
   - [`navigation.launch.py`](./mecanumrover3_navigation/launch/navigation.launch.py#L35) の行35のマップ名を使用したいマップ名に変更する。
   　 次の行を探し、“ファイル名.yaml”を、先ほど保存，移動させた地図ファイルのファイル名に合わせて設定します。

   ```
      get_package_share_directory('mecanumrover3_navigation'),
      'maps',
      'YOUR_MAP_NAME.yaml'))
   ```

   - ロボット実機と通信できたら、以下のコマンドでpub_odom、ロボットのurdf表示、LiDAR関連のlaunchファイルを起動します。
      ```
      ros2 launch mecanumrover3_bringup nav_robot.launch.py
      ```
   
   - 新しいターミナルで以下のコマンドを使用してナビゲーションを開始します。
      ```
      ros2 launch mecanumrover3_navigation navigation.launch.py
      ```
   - メカナムローバーVer.3.0用のNavigation2のパラメータは`mecanumrover3_navigation`パッケージの[`config/nav2_params.yaml`](./mecanumrover3_navigation/config/nav2_params.yaml) フォルダにあります。

## ライセンス

このパッケージはApache 2.0ライセンスの下で提供されています。詳細については、[LICENSE](./LICENSE)ファイルを参照してください。

## 貢献

バグの報告や機能の提案など、このパッケージへの貢献は大歓迎です。issueまたはメールでのお問い合わせをご利用ください。
