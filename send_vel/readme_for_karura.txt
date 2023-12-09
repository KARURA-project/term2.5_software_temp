how to launch
mkdir -p ~/${karura}/src && cd ~/${karura}
source /opt/ros/melodic/setup.bash
cd ..
git clone the repo
rosdep install --from-paths src --ignore-src
catkin_make
source devel/setup.bash
cd to osr/src
chmod +x rover.py
roslaunch osr_bringup osr.launch
