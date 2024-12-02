# Start sensor 
source /home/nx/F1TENTH_WS/devel/setup.bash
roslaunch racecar sensors.launch &
roslaunch racecar teleop.launch &


python3 /home/nx/F1TENTH_WS/src/f1tenth_control/CS498-MP/src/project/studentVision.py &
python3 /home/nx/F1TENTH_WS/src/f1tenth_control/CS498-MP/src/project/car_follower.py
