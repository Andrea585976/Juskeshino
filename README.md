## Configuration and installation
Execute in a command prompt the next commands:
```shell
cd
git clone https://github.com/Andrea585976/Juskeshino.git
cd Juskeshino
./Setup.sh
cd catkin_ws
catkin_make -j2 -l2
```
You will notice catkin_make needs the graph-msgs package to compile without errors, you can resolve it with the next command:
```shell
sudo apt install ros-noetic-graph-msgs
```

Continue executing:
```shell
echo "source ~/Juskeshino/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Test
```shell
cd Juskeshino/catkin_ws
source devel/setup.bash
roslaunch surge_et_ambula justina_gazebo.launch
```

## Aditional packages
To use YOLO:
```shell
pip install yolov5
```
