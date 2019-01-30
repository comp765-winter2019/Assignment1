# Assignment1: localization using visual imagery

## Objective

To provide practice implementing Recursive Bayesian Estimation to track the belief of a robot's position over time using quasi-realistic motion and sensing models. The most common choice to solve this assignment will be to implement a particle filter that is composed of propagation, measurement, and re-sampling steps.

## Installation of Pre-requisites

- The first step is to be able to run the desktop-full verison of ROS Melodic (www.ros.org) as well as Gazebo v9. 
  - If you are installing ROS locally on your machine, dont forget the final step which is to source ROS's setup.bash. 
  - To verify your install, each of the following should run without error:
     - roscore
     - roscd (changes your directory to /opt/ros/melodic)
     - rosrun gazebo_ros gazebo (brings up the GUI for the simulator of an empty world)
     - rqt_image_view

## Getting started with the provided code 

- Obtain and compile the assignment code and add it to your ROS environment: 
  - git clone https://github.com/comp765-winter2019/Assignment1.git
  - cd Assignment1 
  - catkin_make (if problems, try again with -j1 flag)
  - echo source $(pwd)/devel/setup.bash >> ~/.bashrc 
  - source ~/.bashrc  ( must also be run in any previously opened terminals... or just close and re-open them )
- Launch the simulation:
  - roslaunch comp765_assign1 control_helpers.launch
- In a new terminal window, run your localizer node and one to deal with ground-truth and compute errors:
  - roslaunch comp765_assign1 localizer_and_gt_nodes.launch
- In a new terminal window, start an image viewer:
  - rqt_image_view
  - In the top drop-down menu select "/assign1/result_image" to see your method's comparison to ground truth
  - Select "/assign1/localization_debug_image" instead, to see an image you can customize to help development 
- (optional) Launch rviz to view the grouth truth trajectory and the estimated trajectory:
  - rosrun rviz rviz -d `rospack find comp765_assign1`/cfg/config.rviz
- In a (FINAL yaaaay) new terminal window: launch a simple keyboard interface to drive the robot:
  - rosrun aquajoy aqua_key3D
  - Practice driving around the robot. While the window focus is on the terminal, try these key mappings:
    - 0 turns off the auto-pilot (small delay)
    - 1 turns on the auto-pilot in orientation-only mode 
    - 4 turns on the auto-pilot in orientation plus depth mode
    - "w/s" increase/decrease forward speed
    - "j/l" turns left/right
    - "q" tells the robot to stop where it is (some drift will happen)
  
## Starting your own code

You only have to add code to one file: comp765_assign1/src/localizer_node.cpp. The comments there have instructions to get you going, so editing this file is a good place to start. Try to make a small change to start off and ensure you can get this to take effect using the workflow:

- catkin_make
- Reset the robot to the desired state by:
  - Stop the keyboard control (press 0 or kill the node)
  - In Gazebo select the menu Edit-Reset Model Poses
- Stop and start your localizer code (ctrl-c, up, enter)
- Start the keyboard control again (press 4 or re-launch) and make a test swim
  
## Aims for your code:

1. Basic (full marks if these three work about as well as my solution)
  1. Track the robot's position when it starts from the known initial position (0,0,0)
  2. Global localization (aka kidnapped-robot) where you move the robot to some location unknown to your code before beginning
  3. Ensure you can handle a variety of forward speeds and turn patterns: lawn-mower, square, circle, triangle
2. Advanced: change the simulator to 3D mode
  1. Enable by editting aqua_gazebo/urdf/aqua_plugins.urdf.xacro
    1. Search for element "planar" set value to 1
  2. Additional key mappings for 3D:
    - "a/d" deeper/shallower
    - "u/o" roll left/right
    - "i/k" pitch down/up
    - "e/c" heave up/down
  3. Hint: a sufficiently robust observation model for planar can handle some angle changes without explicitly modeling the robot's orientation
3. Brutal mode: kidnapped-robot from arbitrary 6DOF starting location (please be done your project before trying this)

## Hints and Information:

You may find it speeds development to create an "overlay" catkin workspace that contains only the comp765_assign1 package, avoiding unnecessary recompiling of the support packages:

- Start in a terminal that has sourced the setup.bash from the full assignment code
- Make a fresh directory outside any other catkin workspace (perhaps called a1_overlay_ws) 
- cd a1_overlay_ws
- mkdir src
- cd src 
- catkin_init_workspace
- Move only the comp765_assign1 package to this src folder, so it is the only package there 
- cd .. 
- catkin_make (you will see only the one package builds)
- echo source $(pwd)/devel/setup.bash >> ~/.bashrc
- source ~/.bashrc
    
You can now run, code, and rebuild this package alone and it should be quicker. Just remember always to source the devel/setup.bash in the overlay workspace when you start new terminals etc. 

Useful C++ tools:
- std::normal_distribution
- std::uniform_real_distribution
- cv::Mat robot_image = cv_bridge::toCvCopy(robo_img_msg, sensor_msgs::image_encodings::BGR8)->image;
- cv::Vec3b predicted_color = robot_image.at<cv::Vec3b>(probe_pixel_y, probe_pixel_x);
 
