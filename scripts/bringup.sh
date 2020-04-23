#!/bin/bash
source /home/seed/ros/kinetic/devel/setup.bash
roscd task_editor/scripts

ip_address=$1
password=$2
command=$3

if [ ${command} = "controller" ]; then 
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"export ROS_IP='${ip_address}'\" \"roslaunch task_editor robot_bringup.launch;exit\" "'

elif [ ${command} = "make_map" ]; then 
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"sleep 1;export ROS_IP='${ip_address}'\" \"roslaunch --wait task_editor make_map.launch ;exit\" "'

elif [ ${command} = "save_map" ]; then
  map_name=$4
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"export ROS_IP='${ip_address}'\" \"roslaunch task_editor map_saver.launch map_name:='${map_name}' ;exit\" "'

elif [ ${command} = "static_map" ]; then 
  map_name=$4
  gnome-terminal --zoom=0.5 \
    --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"sleep 1;export ROS_IP='${ip_address}'\" \"roslaunch --wait task_editor static_map.launch localization_map_name:='${map_name}';exit\" "' \
    --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"sleep 1;export ROS_IP='${ip_address}'\" \"roslaunch task_editor --wait realsense_laser.launch;exit\" "'


elif [ ${command} = "scenario_start" ]; then 
  scenario_name=$4
  gnome-terminal --zoom=0.5 --geometry=+0+0 --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"export ROS_IP='${ip_address}'\" \"rosrun task_editor start.py '${scenario_name}';exit\" "'

elif [ ${command} = "scenario_stop" ]; then
    local_ip=$4
    export ROS_MASTER_URI=http://${ip_address}:11311;export ROS_IP=${local_ip};
    rosnode kill /aero_scenario_node;

elif [ ${command} = "scenario_restart" ]; then 
  local_ip=$4
  gnome-terminal --zoom=0.5 --geometry=+0+0 --tab -e 'bash -c "export ROS_MASTER_URI=http://'${ip_address}':11311;export ROS_IP='${local_ip}';rosparam set /task_editor/wait_task False ;exit"'

elif [ ${command} = "rviz" ]; then 
  local_ip=$4
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "export ROS_MASTER_URI=http://'${ip_address}':11311;export ROS_IP='${local_ip}';roslaunch --wait task_editor view.launch;exit"'

elif [ ${command} = "ps3" ]; then 
  expect ssh.exp ${ip_address} ${password} "export DISPLAY=:0.0; gnome-terminal --tab -e 'bash -c \"expect -c \\\" spawn env LANG=C sudo sixad --start ; expect password ; send seed\n ; interact ;exec /bin/bash \\\" \" ' "
#gnome-terminal --tab -e 'bash -c "expect -c \"spawn env LANG=C sudo sixad --start ; expect password ; send seed\n ; interact \" "' 

elif [ ${command} = "operation" ]; then 
  local_ip=$4
  action=$5
  scenario_name=$6
  map_name=$7
  if [ ${action} = "start" ]; then 
    gnome-terminal --zoom=0.5 \
      --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"export ROS_IP='${ip_address}'\" \"roslaunch task_editor robot_bringup.launch;exit\" "' \
      --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"sleep 1;export ROS_IP='${ip_address}'\" \"roslaunch --wait  task_editor static_map.launch  localization_map_name:='${map_name}';exit\" "' \
      --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"sleep 1;export ROS_IP='${ip_address}'\" \"roslaunch task_editor --wait realsense_laser.launch;exit\" "' \
      --tab -e 'bash -c "export ROS_MASTER_URI=http://'${ip_address}':11311;export ROS_IP='${local_ip}';sleep 5;roslaunch --wait task_editor view.launch;exit"' \
      --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"export ROS_IP='${ip_address}'\" \"sleep 5;rosrun task_editor start.py '${scenario_name}';exit\" "'
    

  elif [ ${action} = "previous" ]; then 
    export ROS_MASTER_URI=http://${ip_address}:11311;export ROS_IP=${local_ip};
    rosparam set /task_editor/jump -1;
    rosparam set /task_editor/wait_task False ;
  elif [ ${action} = "next" ]; then 
    export ROS_MASTER_URI=http://${ip_address}:11311;export ROS_IP=${local_ip};
    rosparam set /task_editor/jump 1;
    rosparam set /task_editor/wait_task False ;
  elif [ ${action} = "pause" ]; then 
    export ROS_MASTER_URI=http://${ip_address}:11311;export ROS_IP=${local_ip};
    rosrun task_editor go.py
  elif [ ${action} = "go_waypoint" ]; then 
    export ROS_MASTER_URI=http://${ip_address}:11311;export ROS_IP=${local_ip};
    rosrun task_editor go.py $6
  elif [ ${action} = "stop" ]; then 
    killall gnome-terminal-server
  fi

elif [ $1 = "kill" ]; then
  killall gnome-terminal-server

elif [ ${command} = "shutdown" ]; then 
  expect ssh.exp ${ip_address} ${password} "expect -c \" spawn env LANG=C sudo shutdown -h now; expect password; send seed\n; interact \" "

elif [ ${command} = "reboot" ]; then 
  expect ssh.exp ${ip_address} ${password} "expect -c \" spawn env LANG=C sudo reboot; expect password; send seed\n; interact \" "

elif [ ${command} = "save_scenario" ]; then
  file_name=$4
  directory_name=$5
  expect scp.exp ${ip_address} ${password} send ${file_name} ${directory_name}

elif [ ${command} = "load_scenario" ]; then 
  file_name=$4
  directory_name=$5
  expect scp.exp ${ip_address} ${password} load ${file_name} ${directory_name}

elif [ ${command} = "load_waypoint" ]; then 
  file_name=$4
  directory_name=$5
  expect scp.exp ${ip_address} ${password} load ${file_name} ${directory_name}

elif [ ${command} = "check_wifi" ]; then 
  while true
  do
    ping -q -c5 $ip_address > /dev/null
    if [ $? -eq 0 ]; then
      echo "Connection Succeeded"
      break;
    fi
  done

fi
