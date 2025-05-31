# ros-drone
ROS communication for drones

SETUP
1. Install Dependecies
   - For ROS Melodic
   
        sudo apt update 
     
        sudo apt install ros-melodic-cv-bridge ros-melodic-sensor-msgs ros-melodic-geometry-msgs ros-melodic-image-transport python-rospy python-catkin-tools python-numpy python-opencv
     
        pip install torch torchvision
     
   - For ROS Noetic
   
        sudo apt update 
     
        sudo apt install ros-noetic-cv-bridge ros-noetic-sensor-msgs ros-noetic-geometry-msgs ros-noetic-image-transport python3-rospy python3-catkin-tools python3-numpy python3-opencv
     
        pip3 install torch torchvision

3.  mkdir -p <your_catkin_ws>/src
   
4.  cd <your_catkin_ws>/src
   
5.  git clone https://github.com/Wallxtr/drone_package.git
      
6.  cd ~/<your_catkin_ws>

7.  catkin_make
   
8.  echo "source ~/<your_catkin_ws>/devel/setup.bash" >> ~/.bashrc
   
9.  source ~/.bashrc
   
10.  Inıtialize roscore in one terminal
   
11.  You can use scripts like that: \
           rosrun drone_package publisher_main_machine.py  _image_dir:=<your_image_directory> \
           rosrun drone_package publisher_drone_machine.py _image_dir:=<your_image_directory> _model_path:=<your_model_path> \
           rosrun drone_package subscriber_main_machine.py _save_dir:=<your_save_directory> _model_path:=<your_model_path> \
           rosrun drone_package publisher_drone_machine.py _save_dir:=<your_save_directory>
   
12. Note that those scripts test two different scenorio:
       - human detection model running on main machine -> publisher_main_machine.py , subscriber_main_machine.py
       - human detection model running on drone machines -> publisher_drone_machine.py , subscriber_drone_machine.py

13.  Note that you must only run one subscriber nodes (GCS) but you can run many publisher nodes (drones).


   
