# maqueen_ros
スイッチサイエンスマガジンの、[こちらの記事](https://www.switch-science.com/blogs/magazine/maqueen-m5stack-ros2 "ssci magazine")にて使用されているコードです。

joystick_controlとmaqueen_auto、maqueen_tfはros2パッケージであるため、ros2ワークスペースのsrcフォルダ内にクローンしてください。
また、joystick_controlをサブモジュールとして含んでいるため、--recursiveオプションをつけてクローンすることをお勧めします。

第1回では、Atomディレクトリ内のコードを使用しています。他ディレクトリは作業中です。気長にお待ちください。

## 対応マイコンとコード
対応マイコンボードとして、ATOM LiteとATOM S3を想定しています。

ATOM Liteをご使用の際には、Atom/maqueen_urosを、ATOM S3をご使用の際には、Atom/maqueen_uros_AtomS3をそれぞれご使用ください。

## 対応ロボットとコード
対応ロボットとして、Maqueen Lite、Maqueen Plus V1、Maqueen Plus V2を想定しています。

それぞれ、maqueen_lite.hまたはmaqueen_plus.hをインクルードし、MaqueenLiteクラスまたはMaqueenPlusクラスのインスタンスを生成して、使用します。

それぞれの違いは、モータの回転方向を表すCW・CCWに格納されている数値の違いです。

最近発売された、Maqueen Plus V3、Maqueen Lite V5については動作確認を行っておりませんが、
従来製品と同様、CW・CCWの数値を変えることでモータを制御することができるものと考えられます。

## インストール
まずROS2ワークスペースのsrcフォルダへ移動します。

```
cd ~/ros2_ws/src
```

リポジトリをクローンします。

```
git clone --recursive https://github.com/VIMe531/maqueen_ros.git
```

ROS2パッケージをビルドします。使用するのは、joystick_controlなので、joystick_controlを指定します。

```
cd ~/ros2_ws
colcon build --packages-select joystick_control
```

## 使い方
micro-ROS関連の設定を行ったら、ロボットに電源を入れ、以下のコマンドを打ち込みます。

```
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
ros2 run joy joy_node
ros2 run joystick_control joy_to_cmdvel
```


## 関連ドキュメント
[micro:Maqueen PLUS Wiki](https://wiki.dfrobot.com/SKU_MBT0021-EN_Maqueen_Plus_STEAM_Programming_Educational_Robot "maqueen plus") 

[micro:Maqueen micro:bit Educational Programming Robot Platform Wiki](https://wiki.dfrobot.com/micro_Maqueen_for_micro_bit_SKU_ROB0148-EN "maqueen lite")

[micro:bit Hardware Details](https://tech.microbit.org/hardware/ "micro:bit hardware")

[ROS2 Documentation](https://docs.ros.org/en/humble/index.html "ros2 docs")

[micro-ROS Documentation](https://micro.ros.org/ "micro-ros docs")
