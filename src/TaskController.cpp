#include "TaskController.hh"

TaskController::TaskController(const ros::NodeHandle _nh) :
  nh_(_nh)
{ 
  task_controller_server_ = nh_.advertiseService("task_controller", &TaskController::taskControllerCallback, this);
  robot_.reset(new aero::interface::AeroMoveitInterface(nh_));
}

bool TaskController::initMotion(){
  robot_->sendLifter(0.0, -0.4, 3000);
  robot_->waitInterpolation();
 
  return true;
}

bool TaskController::moveLifter(float _x,float _z,float _time){
  robot_->sendLifter(_x, _z, _time);
  robot_->waitInterpolation();
 
  return true;
}

bool TaskController::taskControllerCallback(task_editor::TaskController::Request &_req, task_editor::TaskController::Response &_res)
{
  ROS_INFO("call back start");

  if(_req.task == "init"){  //サービスのタスク名が"init"の時
    if(initMotion() == true) _res.result = "succeeded";
    else _res.result = "aborted"; 
  }
  else if(_req.task == "lifter"){ //サービスのタスク名が"lifter"の時
    if(moveLifter(_req.lifter_position[0],_req.lifter_position[1],_req.lifter_position[2]) == true) _res.result = "succeeded";
    else _res.result = "aborted"; 
  }


}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"task_controller_node");
  ros::NodeHandle nh;

  TaskController tc(nh);  

  ros::spin();

  return 0;
}
