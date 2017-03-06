Specification GUI for MERBOTS
=====================

**roslaunch pm_grasp_visualization server.launch**

In launch set:

* scene is arm5e_arm.xml which used the model: gripper_for_spec.urdf and cloud_source.urdf to reference the cloud...
* rosrun merbots_gui merbots_gui

**PUBLISH INPUT CLOUD FROM BAG OR PCD **
STEREO
dfornas@asusIrslab:~/ros_ws/src/pm_pipeline/pm_grasp_visualization/launch$ rosrun pcl_ros pcd_to_pointcloud cloud2.pcd cloud_pcd:=/stereo_down/points2 1
LASER
rosbag play ~/ros_ws/data/final_experiments/anfora_2017-02-24-09-35-50.bag -l --start=145 --duration=12 --clock 

Copiar los archivos de pm_grasp_planning/launch/uwsim en /uwsim/data/scenes



![alt tag](https://raw.githubusercontent.com/davidfornas/pm_pipeline/merbots/gui.png)


Instructions:

* Press get Grasping position
* Wait for gripper to position
* (optional) Edit position
* Press Execute grasping
