#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import rospy
##-- for smach
from smach import State,StateMachine
import smach_ros
##-- for navigation
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import yaml
import tf
import math
##-- for find pkg
import rospkg
##-- for moveit
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped

###########################################
## @brief ナビゲーション関連のクラス
class NaviAction:
  ## @brief コンストラクタ。waypointsの読込とmove_baseのアクションクライアントの定義
  def __init__(self):
    rospack = rospkg.RosPack()
    rospack.list() 
    path = rospack.get_path('task_editor')
    ## @brief 読み込まれたwaypointsのデータ
    self.config = yaml.load(file(path + "/config/waypoints.yaml"))
    rospy.on_shutdown(self.shutdown)
    ## @brief /move_baseアクションクライアント
    self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while not self.ac.wait_for_server(rospy.Duration(5)):
      rospy.loginfo("Waiting for the move_base action server to come up")
    rospy.loginfo("The server comes up");
    ## @brief MoveBaseGoal型のゴール
    self.goal = MoveBaseGoal()

  ## @brief ゴールポジションの設定と移動の開始
  # @param _number waypointsの番号(0以上の数値）
  # @return ゴールに到着したか否か（succeeded or aborted）
  def set_goal(self,_number):
    rospy.on_shutdown(self.shutdown)

    rev = dict(self.config[_number]) #List to Dictionary

    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    self.goal.target_pose.pose.position.x = rev['pose']['position']['x']
    self.goal.target_pose.pose.position.y = rev['pose']['position']['y']
    self.goal.target_pose.pose.position.z = rev['pose']['position']['z']
    self.goal.target_pose.pose.orientation.x = rev['pose']['orientation']['x']
    self.goal.target_pose.pose.orientation.y = rev['pose']['orientation']['y']
    self.goal.target_pose.pose.orientation.z = rev['pose']['orientation']['z']
    self.goal.target_pose.pose.orientation.w = rev['pose']['orientation']['w']

    rospy.loginfo('Sending goal')
    self.ac.send_goal(self.goal)
    succeeded = self.ac.wait_for_result(rospy.Duration(60));
    state = self.ac.get_state();
    if succeeded:
      rospy.loginfo("Succeed")
      return 'succeeded'
    else:
      rospy.loginfo("Failed")
      return 'aborted'

  ## @brief 経由地点の設定と移動の開始
  # @param _number waypointsの番号(0以上の数値）
  # @return ゴール付近に到着したか否か（succeeded or aborted）
  def set_via_point(self,_number):
    rospy.on_shutdown(self.shutdown)

    rev = dict(self.config[_number]) #List to Dictionary

    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    self.goal.target_pose.pose.position.x = rev['pose']['position']['x']
    self.goal.target_pose.pose.position.y = rev['pose']['position']['y']
    self.goal.target_pose.pose.position.z = rev['pose']['position']['z']
    self.goal.target_pose.pose.orientation.x = rev['pose']['orientation']['x']
    self.goal.target_pose.pose.orientation.y = rev['pose']['orientation']['y']
    self.goal.target_pose.pose.orientation.z = rev['pose']['orientation']['z']
    self.goal.target_pose.pose.orientation.w = rev['pose']['orientation']['w']

    rospy.loginfo('Sending goal')
    self.ac.send_goal(self.goal)
    listener = tf.TransformListener()

    timeout = time.time() + 10; #[sec]

    while True:
      try:
        (position, quaternion) = listener.lookupTransform('map', 'base_link', rospy.Time(0) )
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      
      #ゴールから0.5[m]以内なら、succeededを返す
      if(math.sqrt((position[0]-self.goal.target_pose.pose.position.x)**2 
        + (position[1]-self.goal.target_pose.pose.position.y)**2 ) <= 0.5):
        
        rospy.loginfo("Succeed")
        return 'succeeded'

      elif (time.time() > timeout):
        rospy.loginfo("Timeout")
        return 'aborted' 

      else:
        rospy.sleep(0.5)

  ## @brief move_baseの終了
  def shutdown(self):
    rospy.loginfo("The robot was terminated")
    self.ac.cancel_goal()
#--------------------------------
## @brief ”ムーバー移動”ステート
# @param State smachのステートクラスを継承
class GO_TO_PLACE(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _place  waypointsの番号
  def __init__(self,_place):
    State.__init__(self, outcomes=['succeeded','aborted'])
    ## @brief waypointsの番号
    self.place_ = _place

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return ゴールに到着したか否か（succeeded or aborted）
  def execute(self, userdata):
    print 'Going to Place'+str(self.place_)
    if(na.set_goal(self.place_) == 'succeeded'):return 'succeeded'
    else: return 'aborted' 

## @brief ”経由点移動”ステート
# @param State smachのステートクラスを継承
class GO_TO_VIA_POINT(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _place  waypointsの番号
  def __init__(self,_place):
    State.__init__(self, outcomes=['succeeded','aborted'])
    ## @brief waypointsの番号
    self.place_ = _place

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return ゴールに付近に到着したか否か（succeeded or aborted）
  def execute(self, userdata):
    print 'Going to Place'+str(self.place_)
    if(na.set_via_point(self.place_) == 'succeeded'):return 'succeeded'
    else: return 'aborted' 
#---------------------------------

###########################################
## @brief Moveit Commander用のクラス
# moveit_commanderのライブラリを読みだす
class MoveitCommand:
  ## @brief コンストラクタ。モデルの読み出し、グループの定義
  def __init__(self):
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.dis_body_lifter = 1.065 - 0.92



  ## @brief lifterのIK算出と動作
  # @param _x リフター上部のx座標[m]
  # @param _z リフター上部のz座標[m]
  # @param _vel 速度のスケーリング : 0〜１
  # @return IKが算出できて、移動が完了したか否か（succeeded or aborted）
  def set_lifter_position(self, _x, _z, _vel=1.0):
    self.group = moveit_commander.MoveGroupCommander("torso")
    self.group.set_pose_reference_frame("base_link")

    #print self.group.get_planning_frame

    listener = tf.TransformListener()

    while True:
      try:
        (position, quaternion) = listener.lookupTransform('base_link', 'body_link', rospy.Time(0) )
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      
      if(len(position)>0): break

    current_pose = PoseStamped()
    current_pose = self.group.get_current_pose()
    target_pose = Pose()
    target_pose = current_pose.pose

    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]

    target_pose.position.x = _x
    target_pose.position.y = 0
    target_pose.position.z = _z + self.dis_body_lifter

    self.group.set_pose_target(target_pose)
    self.group.set_max_velocity_scaling_factor(_vel)
    plan = self.group.plan()

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("can't be solved lifter ik")
      self.group.clear_pose_targets()
      return 'aborted'
    else: 
      self.group.execute(plan)
      return 'succeeded'

#---------------------------------
## @brief ”リフター移動”ステート 
# @param State smachのステートクラスを継承
class LIFTER(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _position リフターの絶対位置(x,y)[m]と速度スケーリング(0~1)
  def __init__(self,_position):
    State.__init__(self, outcomes=['succeeded','aborted'])
    position_string = _position.split(',')
    ## @brief リフターの絶対位置(x,y)[m]と速度スケーリング(0~1)
    self.position_ = [float(s) for s in position_string]

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return サービスの呼び出し結果（succeeded or aborted）
  def execute(self, userdata):
    print 'Move Lifter at (' + str(self.position_[0]) +',' + \
      str(self.position_[1]) +') in scale velocity ' + str(self.position_[2])
    if(mc.set_lifter_position(self.position_[0],self.position_[1],self.position_[2]) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

'''
###########################################
## @brief ナビゲーション以外のタスク実行クラス
# task_controllerサーバー( TaskController.hh )とサービスコールで通信を行う
class TaskAction:
  ## @brief コンストラクタ。task_controllerサーバーの起動を待つ
  def __init__(self):
    rospy.loginfo('waiting service')
    rospy.wait_for_service('task_controller')

  ## @brief タスクの設定と実行を行う。task_controllerクライアントを作成し、サーバーへサービスを呼び出す。@n
  # 型の定義は TaskController.srv を参照のこと
  # @warning タスク名以外の引数は不要であっても何か（0など）を入力すること
  # @param _task タスク名(mover, lifter, wait)
  # @param _place waypoints番号
  # @param _lifter_position リフターの位置と移動時間
  # @param _time 待ち時間
  # @return サービスの結果
  def set_action(self,_task,_place,_lifter_position, _time):
    try:
        service = rospy.ServiceProxy('task_controller', TaskController)
        response = service(_task,_place,_lifter_position, _time)
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#---------------------------------
## @brief ”初期化”ステート 
# @param State smachのステートクラスを継承
class INITIALIZE(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return サービスの呼び出し結果（succeeded or aborted）
  def execute(self, userdata):
    print 'Initialize'
    if(ta.set_action("init",0,0,0) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

#---------------------------------
## @brief ”リフター移動”ステート 
# @param State smachのステートクラスを継承
class LIFTER(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _position リフターの絶対位置(x,y)[m]と移動時間[msec]
  def __init__(self,_position):
    State.__init__(self, outcomes=['succeeded','aborted'])
    position_string = _position.split(',')
    ## @brief リフターの絶対位置(x,y)[m]と移動時間[msec]
    self.position_ = [float(s) for s in position_string]

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return サービスの呼び出し結果（succeeded or aborted）
  def execute(self, userdata):
    print 'Move Lifter at (' + str(self.position_[0]) +',' + str(self.position_[1]) +') in time of ' + str(self.position_[2])
    if(ta.set_action("lifter",0,self.position_,0) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

#---------------------------------
'''

##########################################
## @brief ”待ち”ステート 
# @param State smachのステートクラスを継承
class WAIT(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _time 待ち時間[msec]
  def __init__(self,_time):
    State.__init__(self, outcomes=['succeeded','aborted'])
    ## @brief 待ち時間[msec]
    self.time_ = int(_time)

  ## @brief 遷移実行
  # @warning 待ち時間が-1の時はrosparam'/task_editor/wait_task'がTrueになるまで無限待機する
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return succeededのみ
  def execute(self, userdata):
    if(self.time_ >= 0):
      print 'wait ' + str(self.time_) + 'msec'
      rospy.sleep(self.time_ * 0.001)
    else :
      print 'wait for /task_editor/wait_task is False'
      rospy.set_param('/task_editor/wait_task',True)
      while(rospy.get_param('/task_editor/wait_task') == True): pass

    rospy.set_param('/task_editor/wait_task',False)

    return 'succeeded'

##########################################
## @brief ”ループ”ステート 
# @param State smachのステートクラスを継承
class LOOP(State):
  ## @brief コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  # @param _count ループ回数。負の値だと、無限ループ
  def __init__(self,_count):
    State.__init__(self, outcomes=['succeeded','aborted'])
    ## @brief ループ回数。負の値だと、無限ループ
    self.count_ = int(_count)

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return ループがある場合はsucceeded、無い場合はaborted
  def execute(self, userdata):
    self.count_ -= 1
    if(self.count_ > 0):
      print 'count ' + str(self.count_) + ' times'
      return 'succeeded'
    elif(self.count_ < 0):
      print 'infinity loop'
      return 'succeeded'
    else:
      return 'aborted'

##########################################
## @brief ”終了”ステート 
# @param State smachのステートクラスを継承
class FINISH(State):
  ## コンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return succeededのみ
  def execute(self, userdata):
    print 'FINISHED'
    return 'succeeded'

##########################################
## @brief ”何もしない”ステート 実装されていないタスクが実行された場合のみ使用
# @param State smachのステートクラスを継承
class NONE(State):
  ## @briefコンストラクタ。ステートの振る舞い(succeeded or aborted)定義
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return succeededのみ
  def execute(self, userdata):
    return 'succeeded'

############################################
## @brief シナリオの読込クラス
class Scenario:
  ## @brief コンストラクタ。引数のシナリオファイル名を読み込む（デフォルトはscenario.yaml)
  def __init__(self):

    if(len(sys.argv) != 2): file_name = "scenario.yaml"
    else: file_name = str(sys.argv[1])

    rospack = rospkg.RosPack()
    rospack.list() 
    path = rospack.get_path('task_editor')
    ## @brief 読み込まれたシナリオのデータ
    self.scenario = yaml.load(file(path + "/config/" + file_name))
    ## @brief タスクの数
    self.scenario_size = len(self.scenario)

    rospy.set_param('/task_editor/wait_task',False)

  ## @brief タスクの読込
  # @param _number scenario.yamlのデータ行
  # @return タスク名(move,lifter,time,endなど)
  def read_task(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['task']

  ## @brief waypointsの読込
  # @param _number scenario.yamlのデータ行
  # @return waypoints名(0,1,2,3など)
  def read_place(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['place']

  ## @brief リフターの姿勢読込
  # @param _number scenario.yamlのデータ行
  # @return リフターの絶対位置(x,z[m])と移動時間[msec]
  def read_position(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['position']

  ## @brief 待ち時間の読込
  # @param _number scenario.yamlのデータ行
  # @return 待ち時間[msec]
  def read_time(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['time']

  ## @brief ループ先の読込
  # @param _number scenario.yamlのデータ行
  # @return ジャンプ先
  def read_jump(self, _number):
    rev = dict(self.scenario[_number])
    jump_number = int(rev['action']['jump'].split(',')[0])
    if jump_number > 0:
      jump_number = jump_number - 1
    else :
      jump_number = 0
    return jump_number

  ## @brief ループ回数
  # @param _number scenario.yamlのデータ行
  # @return ループ回数
  def read_count(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['jump'].split(',')[1]

#==================================
#==================================
if __name__ == '__main__':
  rospy.init_node('aero_scenario_node')

  mc = MoveitCommand()
  na = NaviAction()
  #ta = TaskAction()
  sn = Scenario()

  # scneario_playというステートマシンのインスタンスを作成
  scenario_play = StateMachine(outcomes=['succeeded','aborted'])
  # scneario_playにステートを追加
  with scenario_play:

    for i in range(sn.scenario_size):
      if sn.read_task(i) == 'init':
       StateMachine.add('ACTION ' + str(i), INITIALIZE(), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'move':
       StateMachine.add('ACTION ' + str(i), GO_TO_PLACE(sn.read_place(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'via':
       StateMachine.add('ACTION ' + str(i), GO_TO_VIA_POINT(sn.read_place(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'lifter':
       StateMachine.add('ACTION ' + str(i), LIFTER(sn.read_position(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'wait':
       StateMachine.add('ACTION ' + str(i), WAIT(sn.read_time(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'loop':
       StateMachine.add('ACTION ' + str(i), LOOP(sn.read_count(i)), \
          transitions={'succeeded':'ACTION '+ str(sn.read_jump(i)),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'end':
       StateMachine.add('ACTION ' + str(i), FINISH(), \
          transitions={'succeeded':'succeeded','aborted':'aborted'})
      else :
       StateMachine.add('ACTION ' + str(i), NONE(), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})

  ## @brief デバッグ出力(smach_viewerで見れるようにする) @sa http://wiki.ros.org/smach_viewer
  sis = smach_ros.IntrospectionServer('server_name',scenario_play,'/SEED-Noid-Mover Scenario Play')
  sis.start()
  # ステートマシンの実行
  scenario_play.execute()
  sis.stop()

