# task_editor

# 概要
当文書では[SEED Platform Robots](https://www.seed-solutions.net/?q=seed_jp/node/73)を用いて、
一連の作業（以下タスク）およびタスク群（以下シナリオ）を記述/実行するためのシステムSEED Grand Station(以下、SGS)について記述する。

# システム構成
SGSの構成としては下図の通りである。  
<img src="https://user-images.githubusercontent.com/12426780/58168973-877f0f80-7cca-11e9-9703-97a55ca0180d.png" width=50%>


まず、遠隔操作PC/クラウド側では、タスク記述用GUI"Task Editor"(**開発中であり、未配布**)を用いてロボットのタスクを記述する。  
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

# 操作方法
## メインコントローラの起動    
1. ros_controller用のlaunchを起動する    
``roslaunch task_editor robot_bringup.launch``
2. GamePadの起動    
[Elecom製GamePad](https://www.elecom.co.jp/products/JC-U4113SBK.html)の場合、上部スイッチをD側にスライドする    
LB(L1)を押しながら下記スティック操作にてMoverを動かす。    
　左ジョイスティック　：　前後、旋回    
　右ジョイスティック　：　左右     
LB(L1)の代わりにLT(L2)を押すと移動速度がUPする。    

## 地図の作成と保存    
1. 地図作成用のlaunchを起動する    
``roslaunch task_editor make_map.launch``    
2. RVizを表示する    
``roslaunch task_editor view.launch``    
3. GamePadで動かして地図が作成できたら保存する    
``roslaunch task_editor map_saver.launch``    
![image](https://user-images.githubusercontent.com/12426780/75102914-ef3a0080-5636-11ea-8ecf-549d1c20d217.png)

## 保存した地図を利用した自律移動    
1. 地図利用のlaunchを起動する    
``roslaunch task_editor static_map.launch``    
**``robot_bringup.launch``は再起動した方が良い**
2. 初期位置合わせをする    
RViz上の“2D Pose Estimate”を選択し、画面上をクリック＆ドラッグ＆ドロップして現在位置を設定する。この時、保存した地図とLiDARの点がおおよそ重なるまで繰り返すこと。     
![image](https://user-images.githubusercontent.com/12426780/75102966-8901ad80-5637-11ea-9a66-cefb3ef4d624.png)
3. 自律移動させる    
RViz上の“2D Nav Goal”ボタンを選択し、画面上をクリック＆ドラッグ＆ドロップして目標位置を設定する。**ドロップした瞬間に、SEED-Moverは移動を開始する。**    
![image](https://user-images.githubusercontent.com/12426780/75102981-bcdcd300-5637-11ea-9102-728a3a83f124.png)    

## シナリオの作成と実行方法    
1. waypointsの登録 
GamePadを使用して、下記ボタンにて登録/削除を行う。    
　登録	: “start”ボタン     
　参照	: “select”ボタン    
　削除	: “LB(L1) + LT(L2) + RB(R1) + RT(R2)”同時押し    
ボタン名は[ElecomのHP](https://www.elecom.co.jp/products/JC-U4113SBK.html)を参照のこと    
![image](https://user-images.githubusercontent.com/12426780/75103036-923f4a00-5638-11ea-9744-29be235197e0.png)    
2. シナリオの編集    
``emacs ~/ros/kinetic/src/task_editor/config/scenario.yaml``    
**任意のテキストエディタで問題ない**    
**各タスク内容は[API一覧](#API)参照のこと**
3. シナリオの実行    
``rosrun task_editor start.py`` : scenario.yamlが実行される。    
もし別のyamlファイルを実行したい場合、下記のようにすること。    
``rosrun task_editor start.py test.yaml``   : test.yamlが実行される。

# <a name = "API"> API一覧

| タスク名 | 引数名 | 引数の内容 | 引数の例 | 備考 |
| ------------- | ------------- | ------------- | ------------- | ------------- |
| move | place | waypoints番号 | place : 0 | waypointsへ自律移動する |
| via | place | waypoints番号 | place : 0 | waypoints付近まで自律移動する |
| vel_move | velocity | x速度[m/sec],y速度[m/sec],theta速度[rad/sec],時間[sec] | velocity: 0.1,0,0,3 | 指定速度で指定時間だけ移動する|
| lifter | position | base_linkからのリフターx位置[m],z位置[m],速度係数 | position: 0.0,0.5,1.0 | SEED-Lifterを移動させる |
| wait | time | 時間[msec] | wait : 1000 | 指定時間待つ。-1を入力すると rosparam``/task_editor/wait_task``が``False``になるまで待つ |
| loop | jump | タスク番号, ループ回数 | jump : 1,-1 | 別タスクにジャンプ/ループする。ループ回数を-1にすると無限ループする |
| led | led | 対象のSendNo, スクリプト番号 | led : 3,1 | 台車LEDを制御する。左の例だと右前LEDのスクリプト１を実行する |
| set_inflation | value | inflationサイズ[m] | value : 0.2 | costmapのinflationを設定する |
| set_max_vel | value | x速度[m/sec],y速度[m/sec],theta速度[rad/sec] | value : 0.5,0.5,0.8 | 自律移動時の最大速度を設定する |




# クラスドキュメントの作成方法

クラスドキュメントは[rosdoc_liteパッケージ](http://wiki.ros.org/rosdoc_lite)を用いて作成されている。
ローカルPCにて最新版に更新する場合は下記手順を実行のこと。
```
sudo apt-get install ros-kinetic-rosdoc-lite 
roscd task_editor
rosdoc_lite .
```
/task_editor直下に/doc/htmlが作成されるので、Firefoxなどで``index.html``を開けば良い。

