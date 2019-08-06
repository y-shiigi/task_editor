# task_editor

# 概要
当文書では[SEED Platform Robots](https://www.seed-solutions.net/?q=seed_jp/node/73)を用いて、
一連の作業（以下タスク）およびタスク群（以下シナリオ）を記述/実行するためのシステムSEED Grand Station(以下、SGS)について記述する。

# システム構成
SGSの構成としては下図の通りである。  
<img src="https://user-images.githubusercontent.com/12426780/58168973-877f0f80-7cca-11e9-9703-97a55ca0180d.png" width=50%>


まず、遠隔操作PC/クラウド側では、タスク記述用GUI"Task Editor"を用いてロボットのタスクを記述する。  
次に記述したタスク群を１つのシナリオとして、ロボット側へ転送する。  
ロボット側は、遠隔操作PC/クラウド側から発せられるキューに応じてシナリオを実行する仕組みとなっている。  
なお、ロボット自身および周囲の環境情報は常時、遠隔操作PC/クラウド側へ送られ、データ可視化ツール"Rviz"に表示される。  

ロボット側にはソフトウェア群として[ROS](http://www.ros.org/about-ros/)のパッケージである
``seed_r7_ros_pkg``、``seed_smartactuator_sdk``と``task_editor``が入っている。  
``seed_r7_ros_pkg``については[Github](https://github.com/seed-solutions/seed_r7_ros_pkg)を参照することとし、
次節以降では``task_editor``について記述する。


# task_editor ディレクトリ構成

``task_editor``のディレクトリ構成は下図の通りである

<div style="line-height: 1em">
<pre>
/task_editor
├── CMakeLists.txt
├── README.md
├── TaskEditor-GUI              //GUIの実行ファイル
├── TaskEditor-GUI.ini          //GUIの初期設定ファイル
├── /config                     //設定データ
│ ├── /maps
│ │ ├── map.pgm               //地図データの画像ファイル
│ │ └── map.yaml              //地図データの設定ファイル
│ ├── ps3-holonomic.config.yaml//DUALSHOCK操作時の設定ファイル
│ ├── scenario.yaml            //シナリオファイル
│ └── waypoints.yaml           //waypointsファイル
├── /launch                     //複数のノードを起動するためのlaunchファイル 
│ ├── make_map.launch          //地図作成用
│ ├── map_saver.launch         //地図保存用
│ ├── robot_bringup.launch     //ロボットのシリアル通信起動用
│ ├── simulation.launch        //実機無しでの検証用
│ ├── static_map.launch        //地図利用（SLAM）用
│ ├── view.launch              //Rvizの表示用
│ └── wheel_bringup.launch     //URGの起動、DUALSHOCKによる走行のための設定用
├── package.xml
├── /scripts                    //シェルスクリプトおよびpythonスクリプト
│ ├── bringup.sh               //GUIからのコマンド呼び出し
│ ├── scp.exp                  //ロボットPC<->遠隔操作PCのファイル移行
│ ├── ssh.exp                  //SSH接続設定
│ └── start.py                 <span style="color: red; ">//シナリオ実行</span>
├── /src                        //ソースファイル
│ └── WaypointsEditor.cpp      //waypointsの設定/保存
└── view.rviz                   //rvizの設定ファイル
</pre>
</div>


# task_editor 仕様

シナリオ作成〜実行までの一連の流れは下記のとおりである。  

1. GUIを用いて作成されたシナリオは"保存ボタン"を押すと/configに.yamlファイルとして作成され、ロボット側のPCに転送される  
2. GUIにて"シナリオ実行"ボタンを押すとロボット側のPCにて start.py が実行される。  
start.py は主にステートマシンの作成と実行を担っている。  
ステートマシンとしては[smach](http://wiki.ros.org/smach)を用いており、詳細は[O'REILYの書籍](https://www.oreilly.co.jp/books/9784873118093/)が参考となる。  
3. ステートがムーバー移動の場合は、[アクション](http://wiki.ros.org/actionlib)という通信の仕組みで/move_baseとやり取りを行う。  
ステートがリフター移動の場合は、[サービス](http://wiki.ros.org/Services)という通信の仕組みで/task_controllerとやり取りを行う。
詳細は[工学社の書籍](https://www.kohgakusha.co.jp/support/ros_robot/index.html)が参考となる。


シナリオ実行時のフローチャートは下図の通りである。なお、赤文字は実際の関数等を表している。

<img src="https://user-images.githubusercontent.com/12426780/58219653-a9b67300-7d46-11e9-9c72-8a54869f62f5.png" width=80%>

# クラスドキュメントの作成方法

クラスドキュメントは[rosdoc_liteパッケージ](http://wiki.ros.org/rosdoc_lite)を用いて作成されている。
ローカルPCにて最新版に更新する場合は下記手順を実行のこと。
```
sudo apt-get install ros-kinetic-rosdoc-lite 
roscd task_editor
rosdoc_lite .
```
/task_editor直下に/doc/htmlが作成されるので、Firefoxなどで``index.html``を開けば良い。

