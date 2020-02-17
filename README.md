# Capstone Project
This is an individiaul submit of Capstone project.

Talat Buyukakin

talat.buyukakin@tofas.com.tr

talatb.akin@hotmail.com

GitHub: TBuyukakin

## Modules
### Waypoint updater
Subscriber: /current_pose, /base_waypoints, /traffic_waypoint, /obstacle_waypoint
Publisher : /final_waypoints

The waypoint updater node is responsible for selecting a given number of waypoints ahead of the vehicle that describe the desired route and publish the list with target velocities to the /final_waypoints topic. 

- Find closest waypoint
	Search closest waypoint using KDTree
    Ensure it is in front of the vehicle using dot product
    Check if traffic light is further than waypoints
    	If there is no traffic light stopline, then target speed is set to the maximum
        If yes, arrenge velocity
- Arrenge velocity
	Find stopping waypoint index
    To be on time consider earlier index 
    Arrenge velocity based on distance and deceleration
- Define callback functions of subscribers

### DBW - Drive By Wire
#### DBW Node
Subscriber:/current_velocity, /twist_cmd, /vehicle/dbw_enabled
Publisher: /vehicle/steering_cmd, /vehicle/throttle_cmd, /vehicle/brake_cmd

DBW Node publishes the vehicle's throttle, steering and brakes commands based on current velocity and target velocity provided by “Waypoint Follower node”.

- Define specifications
- Publish commands to
	Throttle Control: Takes the target and current velocity and calculates the error and adjustment to be published to 		/vehicle/throttle_cmd
    Steering Control: Based on linear and angular velocities, calculates target steering angle using the vehicle’s 			steering ratio and wheel-base data, it is published to the /vehicle/steering_cmd topic.
    Braking Control: Calculates target deceleration torque based on vehicle’s velocity error, vehicle weight, and wheel 	radius, it is published to the /vehicle/brake_cmd topic.

#### Twist Controller
- Define PID Gain, Derivative and Integrator Controller Parameters (Kp, Kd, Ki)
- PID implementation for velocity control
	Define velocity error (command - current)
    Get velocity correction from PID
    Calculate steering angle
 	Calculate maximum breaking torque

### TL Detector
Subscriber: /current_pose, /base_waypoints, /vehicle/traffic_lights, /image_color
Publisher : /traffic_waypoint

- Define callback functions for subscribers
- Find closest visible traffic light using KDTree
- Obtain the state of the light
- Pubslish upcoming red light state waypoints 

#### TL Classifier
TL Classifier is adapted from [TensorflowModel API]( https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb)

- Import Trained Model: ssd_inception_coco
- Load the previously created list of label strings
- Obtain the categories & take the indexes
- Input and output Tensors for detectGraph obtain scores
- Visualize boxes & labels on image array
- Assign scores on categories

### Conclusion
#### Submission Checklist
- All code is uploaded to GitHub. Here it can be found at ()[]
- README is created and included.
- Email and name is included on "Notes to Reviewer" section

#### Requirements
It is able to :
- be launched correctly using the launch files provided in the capstone repo.
- follow the waypoints smoothly in the simulator.
- arrenge the target speed for the waypoints with respect to different values of kph velocity parameter 
- stop at traffic lights when needed.
- publishes throttle, steering, and brake commands at 50hz.


------------------------------------------------------------------------------------------------------------------------------------------------

Original README of the project can be found below.

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
