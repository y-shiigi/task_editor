#include "TaskController.hh"

TaskController::TaskController(const ros::NodeHandle _nh) :
  nh_(_nh)
{
  task_controller_server_ = nh_.advertiseService("task_controller", &TaskController::taskControllerCallback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
}

bool TaskController::moveMarker(std::string _marker){
  tf::StampedTransform marker_coord;
  tf::StampedTransform camera_coord;
  tf::StampedTransform cd_tf;
  tf::Quaternion rotate;

  double dx,dy,dth;

  bool reached_x,reached_y,reached_th;
  reached_x = reached_y = reached_th = false;
  rotate.setRPY(0,1.57,0);

  while(1){
    //marker_coord = GetTransformObject("ar_marker_0");
    marker_coord = GetTransformObject(_marker);
    camera_coord = GetTransformObject("camera_link");

    dx = marker_coord.getOrigin().x() - camera_coord.getOrigin().x();
    dy = marker_coord.getOrigin().y() - camera_coord.getOrigin().y();
    dth = tf::getYaw(marker_coord.getRotation() * rotate);

    //std::cout << dx << "\t" << dy << "\t" << fabs(dx) << "\t" << fabs(dy) << std::endl;
    //std::cout << "\t" << dth << std::endl;

    if(fabs(dx) < 0.20) reached_x = true;
    else reached_x = false;
    if(fabs(dy) < 0.01) reached_y = true;
    else reached_y = false;
    if(fabs(dth) < 0.02) reached_th = true;
    else reached_th = false;


    if((marker_coord.getOrigin() == tf::Vector3(0.0,0.0,0.0)) || (reached_x == true && reached_y == true && reached_th == true)) break;
    else{
      if(reached_x == false) cmd_vel_.linear.x = dx * 0.1;
      else cmd_vel_.linear.x = 0;

      if(reached_y == false) cmd_vel_.linear.y = dy * 0.3;
      else cmd_vel_.linear.y = 0;

      if(reached_th == false) cmd_vel_.angular.z= dth * 0.2;
      else cmd_vel_.angular.z= 0;

      cmd_vel_pub_.publish(cmd_vel_);
    }
  }
  cmd_vel_.linear.x = 0;
  cmd_vel_.linear.y = 0.0;
  cmd_vel_.angular.z= 0;

  cmd_vel_pub_.publish(cmd_vel_);

  return true;
}

tf::StampedTransform TaskController::GetTransformObject(std::string object){
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // char *command;

  //sprintf(command,"mplayer %s/scripts/sounds/no_tray.wav",pkg_path_.c_str());

  try{
    listener.waitForTransform("/base_link",object,
			      ros::Time(0), ros::Duration(2.0));
    listener.lookupTransform("/base_link",object,
			     ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    //system(command);
    ROS_INFO("No getting point");
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  }

  return transform;
}


bool TaskController::taskControllerCallback(task_editor::TaskController::Request &_req, task_editor::TaskController::Response &_res)
{
  ROS_INFO("call back start");

  if(_req.task == "marker"){
    if(moveMarker(_req.marker) == true) _res.result = "succeeded";
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
