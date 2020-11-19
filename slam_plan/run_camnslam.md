
Need to install ROS for any of this to work.  

Install drivers for realsense2 camera:

```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt-get install librealsense2-dkms -y
sudo apt-get install librealsense2-utils -y
```

Alternate?
```
sudo apt-get install ros-melodic-realsense2-camera -y
sudo apt-get install ros-$ROS_VER-realsense2-description -y
```

To launch camera and see view of camera topics:
```
roslaunch realsense2_camera rs_camera.launch
rosrun image_view image_view image:=/camera/color/image_raw __name:=asdf
```


#To run orbslam2 in ros:

Install:
Create workspace with source folder and clone the repo into the source folder.  Install any other dependencies.  Then build and source the build to run.  
```
git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git
sudo apt install libeigen3-dev -y
cd ../
catkin build
source devel/setup.bash
```

Then run with:
```
roslaunch orb_slam2_ros orb_slam2_d435_mono.launch
```
