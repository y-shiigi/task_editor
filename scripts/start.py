#!/usr/bin/env python
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
##-- for find pkg
import rospkg
##-- for task
from task_editor.srv import *
import sys

############################################
class NaviAction:
  def __init__(self):
    rospack = rospkg.RosPack()
    rospack.list() 
    path = rospack.get_path('task_editor')
    self.config = yaml.load(file(path + "/config/waypoints.yaml"))
    rospy.on_shutdown(self.shutdown)
    self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while not self.ac.wait_for_server(rospy.Duration(5)):
      rospy.loginfo("Waiting for the move_base action server to come up")
    rospy.loginfo("The server comes up");
    self.goal = MoveBaseGoal()

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

  def shutdown(self):
    rospy.loginfo("The robot was terminated")
    self.ac.cancel_goal()
#--------------------------------
class GO_TO_PLACE(State):
  def __init__(self,_place):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.place_ = _place

  def execute(self, userdata):
    print 'Going to Place'+str(self.place_)
    if(na.set_goal(self.place_) == 'succeeded'):return 'succeeded'
    else: return 'aborted' 
#---------------------------------

###########################################
class TaskAction:
  def __init__(self):
    rospy.loginfo('waiting service')
    rospy.wait_for_service('task_controller')

  def set_action(self,_task,_place,_lifter_position, _time):
    try:
        service = rospy.ServiceProxy('task_controller', TaskController)
        response = service(_task,_place,_lifter_position, _time)
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#---------------------------------
class INITIALIZE(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  def execute(self, userdata):
    print 'Initialize'
    if(ta.set_action("init",0,0,0) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

#---------------------------------
class LIFTER(State):
  def __init__(self,_position):
    State.__init__(self, outcomes=['succeeded','aborted'])
    position_string = _position.split(',')
    self.position_ = [float(s) for s in position_string]

  def execute(self, userdata):
    print 'Move Lifter at (' + str(self.position_[0]) +',' + str(self.position_[1]) +') in time of ' + str(self.position_[2])
    if(ta.set_action("lifter",0,self.position_,0) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

#---------------------------------

##########################################
class WAIT(State):
  def __init__(self,_time):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.time_ = int(_time)

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
class FINISH(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  def execute(self, userdata):
    print 'FINISHED'
    return 'succeeded'

##########################################
class NONE(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  def execute(self, userdata):
    return 'succeeded'

############################################
class Scenario:
  def __init__(self):

    if(len(sys.argv) != 2): file_name = "scenario.yaml"
    else: file_name = str(sys.argv[1])

    rospack = rospkg.RosPack()
    rospack.list() 
    path = rospack.get_path('task_editor')
    self.scenario = yaml.load(file(path + "/config/" + file_name))
    self.scenario_size = len(self.scenario)

    rospy.set_param('/task_editor/wait_task',False)

  def read_task(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['task']

  def read_place(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['place']

  def read_position(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['position']

  def read_time(self, _number):
    rev = dict(self.scenario[_number])
    return rev['action']['time']


#==================================
#==================================
if __name__ == '__main__':
  rospy.init_node('aero_scenario_node')

  na = NaviAction()
  ta = TaskAction()
  sn = Scenario()

  scenario_play = StateMachine(outcomes=['succeeded','aborted'])
  with scenario_play:

    for i in range(sn.scenario_size):
      if sn.read_task(i) == 'init':
       StateMachine.add('ACTION ' + str(i), INITIALIZE(), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'move':
       StateMachine.add('ACTION ' + str(i), GO_TO_PLACE(sn.read_place(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'lifter':
       StateMachine.add('ACTION ' + str(i), LIFTER(sn.read_position(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'wait':
       StateMachine.add('ACTION ' + str(i), WAIT(sn.read_time(i)), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
      elif sn.read_task(i) == 'end':
       StateMachine.add('ACTION ' + str(i), FINISH(), \
          transitions={'succeeded':'succeeded','aborted':'aborted'})
      else :
       StateMachine.add('ACTION ' + str(i), NONE(), \
          transitions={'succeeded':'ACTION '+ str(i+1),'aborted':'ACTION '+str(i+1)})
        
  sis = smach_ros.IntrospectionServer('server_name',scenario_play,'/SEED-Noid Scenario Play')
  sis.start()
  scenario_play.execute()
  sis.stop()

