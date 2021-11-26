# DVRK_robot_project
This seemed to be the easiest installation process using catkin build and rosinstall. I used Ubuntu 20.04. 
1. install a ROS version, I installed ROS Noetic 
2. run the following command to install useful packages along with catkin-tools ```sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev python3-wstool python3-catkin-tools python3-osrf-pycommon```
3. run the following command with the appropriate ros version, use melodic if you have melodic instead of noetic or similiar ```source /opt/ros/noetic/setup.bash```
4. create a directory on the home directory ```
mkdir ~/catkin_ws```
5. use wstool to pull all the code from github ```wstool init src```
6. create files for catkin build tool ```catkin init```
7. all code should be compiled in release mode ```catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release```
8. go in source directory to pull code ```cd src```
9. get code from github, here I used the devel branch not the master branch, ```wstool merge https://raw.githubusercontent.com/jhu-dvrk/dvrk-ros/devel/dvrk_ros.rosinstall```
10. now wstool knows which repositories to pull ```wstool up```
11. compile everything use catkin build command ```catkin build --summary```
12. setting up environment variables for ROS, open the ```.bashrc``` file in the home directory. It is a hidden folder so **Cntrl+H** will show the hidden folders. 
```
# for ROS
if [ -f ~/catkin_ws/devel/setup.bash ]; then
  . ~/catkin_ws/devel/setup.bash
fi
# for cisst (optional)
if [ -f ~/catkin_ws/devel/cisstvars.sh ]; then
  . ~/catkin_ws/devel/cisstvars.sh
fi
```

# Troubleshoot 
When following step 11, catkin build was showing few errors and cmake was not working. I found ways that seemed to work for me. 
1. error 1, something like this (empy error)
```-- Could NOT find PY_em (missing: PY_EM) CMake Error at cmake/empy.cmake:30 (message): Unable to find either executable 'empy' or Python module 'em'...  try installing the package 'python-empy'```

solution: ```pip install empy```

2. error 2, python path cataking package error
```ImportError: "from catkin_pkg.package import parse_package failed: No module named 'catkin_pkg' Make sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH```

solution: open the .bashrc file and add the following line
```export PYTHONPATH=$PYTHONPATH:/usr/lib/python3/dist-packages```
Then restart the terminal. 
3. error 3: uuid error, this may have something to do with the anaconda that I had on my machine, 

solution: 
```ls ~/anaconda3/lib/libuuid*```
```mkdir ~/anaconda3/libuuid```
```mv ~/anaconda3/lib/libuuid* ~/anaconda3/libuuid```
