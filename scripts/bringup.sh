#!/bin/bash
source /home/seed/ros/kinetic/devel/setup.bash
roscd task_editor/scripts

ip_address=$1
password=$2

if [ $3 = "controller" ]; then 
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"export ROS_IP='${ip_address}'\" \"roslaunch aero_startup aero_bringup.launch;exit\" "'

elif [ $3 = "make_map" ]; then 
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"sleep 1;export ROS_IP='${ip_address}'\" \"roslaunch task_editor make_map.launch;exit\" "'

elif [ $3 = "save_map" ]; then 
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"export ROS_IP='${ip_address}'\" \"roslaunch task_editor map_saver.launch;exit\" "'

elif [ $3 = "static_map" ]; then 
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"sleep 1;export ROS_IP='${ip_address}'\" \"roslaunch task_editor static_map.launch;exit\" "'

elif [ $3 = "scenario_start" ]; then 
  scenario_name=$4
  gnome-terminal --tab -e 'bash -c "expect ssh.exp '${ip_address}' '${password}' \"export ROS_IP='${ip_address}'\" \"rosrun task_editor start.py '${scenario_name}';exit\" "'

elif [ $3 = "rviz" ]; then 
  gnome-terminal --zoom=0.5 --tab -e 'bash -c "export ROS_MASTER_URI=http://'${ip_address}':11311;export ROS_IP=192.168.10.200;roslaunch task_editor view.launch;exit"'

elif [ $3 = "ps3" ]; then 
  expect ssh.exp ${ip_address} ${password} "export DISPLAY=:0.0; gnome-terminal --tab -e 'bash -c \"expect -c \\\" spawn env LANG=C sudo sixad --start ; expect password ; send seed\n ; interact ;exec /bin/bash \\\" \" ' "
#gnome-terminal --tab -e 'bash -c "expect -c \"spawn env LANG=C sudo sixad --start ; expect password ; send seed\n ; interact \" "' 

elif [ $1 = "kill" ]; then
  killall gnome-terminal-server

elif [ $3 = "shutdown" ]; then 
  expect ssh.exp ${ip_address} ${password} "expect -c \" spawn env LANG=C sudo shutdown -h now; expect password; send seed\n; interact \" "

elif [ $3 = "reboot" ]; then 
  expect ssh.exp ${ip_address} ${password} "expect -c \" spawn env LANG=C sudo reboot; expect password; send seed\n; interact \" "

elif [ $3 = "save_scenario" ]; then
  file_name=$4
  directory_name=$5
  expect scp.exp ${ip_address} ${password} send ${file_name} ${directory_name}

elif [ $3 = "load_scenario" ]; then 
  file_name=$4
  directory_name=$5
  expect scp.exp ${ip_address} ${password} load ${file_name} ${directory_name}

elif [ $3 = "load_waypoint" ]; then 
  file_name=$4
  directory_name=$5
  expect scp.exp ${ip_address} ${password} load ${file_name} ${directory_name}

elif [ $3 = "check_wifi" ]; then 
  while true
  do
    ping -q -c5 $ip_address > /dev/null
    if [ $? -eq 0 ]; then
      echo "Connection Succeeded"
      break;
    fi
  done

fi
