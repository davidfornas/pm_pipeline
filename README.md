pm_pipeline
=====================

This packages contain the _PERCEPTION_ and _MANIPULATION_ pipeline componets developed by **IRSLab** researchers. There are packages dealing with robotic vision and manipulation, mainly dealing with underwater environments.

 * **pm_perception**: Package about perception: from low level sensing
to interpretation, 3D processing, segmentation ...

 * **pm_tools**: Classes that interface to PCL, ViSP and OpenCV, among others, to expose a simplified interface.

 * **pm_measure**: Classes used to measure times.

 * **pm_manipulation**: Arm control packages used to object manipulation and arm control in environments
already perceived with pm_perception.

 * **pm_examples**: Stand alone examples of the usage of this classes.

 * **pm_grasp_visualization**: Grasp specification for MERBOTS and visual planning.


CHANGES MERBOTS: Specification GUI for MERBOTS
==============================================

**roslaunch pm_grasp_visualization server.launch**

In launch set:

* scene is arm5e_arm.xml which used the model: gripper_for_spec.urdf and cloud_source.urdf to reference the cloud...
* rosrun merbots_gui merbots_gui

**PUBLISH INPUT CLOUD FROM BAG OR PCD **

STEREO
dfornas@asusIrslab:~/ros_ws/src/pm_pipeline/pm_grasp_visualization/launch$ rosrun pcl_ros pcd_to_pointcloud cloud2.pcd cloud_pcd:=/stereo_down/points2 1
LASER
rosbag play ~/ros_ws/data/final_experiments/anfora_2017-02-24-09-35-50.bag -l --start=145 --duration=12 --clock 

Copy files in pm_grasp_planning/launch/uwsim to /uwsim/data/scenes

![alt tag](https://raw.githubusercontent.com/davidfornas/pm_pipeline/merbots/gui.png)

Instructions:

* Press get Grasping position
* Wait for gripper to position
* (optional) Edit position
* Press Execute grasping
