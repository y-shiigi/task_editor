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


# 環境構築
1. ROS、seed_smartactuator_skd、seed_r7_ros_pkgのインストール    
[Github](https://github.com/seed-solutions/seed_r7_ros_pkg)のREADME.md通りにインストールし、UDEVルールの設定、動作確認まで行う  
2. Realsenseドライバのインストール    
下記は参考程度とし、必ず[公式HP](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)を確認してください
```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver $ hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 
```
Ubuntu16の場合    
``
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u 
``    
Ubuntu18の場合    
``
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
``
```
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
```
Realsenseを接続し、 ``$ realsense-viewer``と入力して画像が表示されたら正常にインストールされている。
3. realsense-rosのインストール    
```
sudo apt-get install ros-${ROS_DISTRO}-realsense2-* ros-${ROS_DISTRO}-ddynamic-reconfigure ros-${ROS_DISTRO}-rgbd-launch 
```
4. 各種ツールのインストール
```
sudo apt-get install emacs byobu openssh-server chrony expect setserial xterm
sudo apt-get install ros-${ROS_DISTRO}-jsk-common ros-${ROS_DISTRO}-teleop-twist-joy ros-${ROS_DISTRO}-uvc-camera ros-${ROS_DISTRO}-jsk-visualization
```
5. task_editorのインストール
```
cd ~/ros/${ROS_DISTRO}/src
git clone https://github.com/y-shiigi/task_editor
cd ~/ros/${ROS_DISTRO}
rosdep install -y -r --from-paths src --ignore-src
catkin build task_editor
```
6. (**任意**)フォルダ名を英語にする
```
LANG=C xdg-user-dirs-gtk-update
```
7. (**任意**)電源ボタンを押したらシャットダウンさせる
```
sudo gedit /etc/acpi/events/powerbtn
```
上記``powerbtn``のファイルにて、下記変更を加える
```
#action=/etc/acpi/powerbtn.sh
action=shutdown -h now
```


# 操作方法
以下では１つのPCで作業する場合を記載する。
複数台PC（遠隔操作含む）で作業する場合は、下記を参考にすること。    
　[ROS_環境設定](https://qiita.com/srs/items/7d4aeb5e44138f97c770)    
　[jsk_common(ROS環境設定の簡易化)](https://jsk-common.readthedocs.io/en/latest/jsk_tools/cltools/setup_env_for_ros.html?highlight=rossetip)    
　[byobu(仮想ターミナル)](https://linuxfan.info/terminal-with-byobu)    

## メインコントローラの起動    
1. ros_controller用のlaunchを起動する    
``roslaunch task_editor robot_bringup.launch``
2. GamePadの起動    
[Elecom製GamePad](https://www.elecom.co.jp/products/JC-U4113SBK.html)の場合、上部スイッチをD側にスライドし、LBボタンを押した状態で、SEED-Moverの車輪にサーボが入るまで待つ。    
![image](https://user-images.githubusercontent.com/12426780/78964339-cdd49b80-7b34-11ea-8275-036b8071d5db.png)    
LBボタンを押したままジョイスティックを操作し、SEED-Moverが動く事を確認する。動かし方は下記を参照のこと。なお、LBの代わりにLTボタンを押すと移動速度がアップする。     
![image](https://user-images.githubusercontent.com/12426780/78964574-a3cfa900-7b35-11ea-8678-e59b2b37e37f.png)   
・左ジョイスティック　：　前後、旋回    
・右ジョイスティック　：　左右移動    
SEED-Moverがガタガタと振動する場合、GamePadの連射モードがONになっている可能性があるため、[マニュアル](https://www.elecom.co.jp/support/manual/peripheral/gamepad/jc-u4113s/jc-u4113s_v2.pdf)を参考に連射モードをOFFにすること。

## 地図の作成と保存    
1. 地図作成用のlaunchを起動する    
``roslaunch task_editor make_map.launch``    
2. RVizを表示する    
``roslaunch task_editor view.launch``    
3. GamePadで動かして地図が作成できたら保存する    
``roslaunch task_editor map_saver.launch``    
![image](https://user-images.githubusercontent.com/12426780/75102914-ef3a0080-5636-11ea-8ecf-549d1c20d217.png)    
地図は``/task_editor/config/maps"に保存される

## 保存した地図を利用した自律移動    
1. 地図利用のlaunchを起動する    
``roslaunch task_editor static_map.launch``    
**このとき、``robot_bringup.launch``は再起動した方が良い**
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
　参照	: “back”ボタン    
　削除	: “LB(L1) + LT(L2) + RB(R1) + RT(R2)”同時押し    
ボタン名は[ElecomのHP](https://www.elecom.co.jp/products/JC-U4113SBK.html)を参照のこと    
![image](https://user-images.githubusercontent.com/12426780/75103036-923f4a00-5638-11ea-9744-29be235197e0.png)    
2. シナリオの編集    
``emacs ~/ros/${ROS_DISTRO}/src/task_editor/config/scenario.yaml``    
**編集は任意のテキストエディタで問題ない**    
**各タスク内容は[API一覧](https://github.com/y-shiigi/task_editor/wiki/API%E4%B8%80%E8%A6%A7)参照のこと**

3. シナリオの実行    
``rosrun task_editor start.py`` : scenario.yamlが実行される。    
もし別のyamlファイルを実行したい場合、下記のようにすること。    
``rosrun task_editor start.py test.yaml``   : test.yamlが実行される。

## Realsenseの利用
### ARマーカー検出として利用したい場合    
```
roslaunch task_editor ar_track_realsense.launch
```    
カメラ前方にARマーカーをかざし、Rviz上にTF(``ar_marker_*``)が出力されることを確認する。また、使用するRealsenseやマーカーによって、``ar_track_realsense.launch``の下記パラメータを変更する。  
* serial_no_camera : realsenseの固有シリアル番号    
→``realsense-viewer``で確認できます
* marker_size　：　ARマーカーの1辺の長さ[cm]    

その他設定は[wiki](http://wiki.ros.org/ar_track_alvar)を参照のこと

### 障害物検知として使用したい場合
```
roslaunch task_editor realsense_laser.launch
```
Rviz上でSEED-Mover後方に黄色レーザーが出力される。出力されない場合、RealsenseのUSBを抜き差しするか、PCを再起動すること。

**USBの通信帯域の問題で、現状は2台同時に起動できません!**