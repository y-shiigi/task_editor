#ifndef TASK_CONTROLLER_H_
#define TASK_CONTROLLER_H_

#include <aero_std/AeroMoveitInterface.hh>
#include "task_editor/TaskController.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
//////////////////////////////
class TaskController{
public:
  TaskController(ros::NodeHandle _nh);
  bool taskControllerCallback(task_editor::TaskController::Request &_req, task_editor::TaskController::Response &_res);
  bool initMotion();
  bool moveLifter(float _x,float _z,float _time);
  bool downLifter();
  bool upLifter();

private:
  ros::NodeHandle nh_;
  ros::ServiceServer task_controller_server_;
  aero::interface::AeroMoveitInterface::Ptr robot_;
  std::map<aero::joint, double> joint_angles_;
};

#endif
