/**
 * @file main.cpp
 * @author Doug Summerlin, Rajan Pande
 * @brief The main cpp file for our final_project node for the ENPM809Y Final Project
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <atomic>
#include <cmath>
#include "../include/bot_controller.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::atomic<bool> found_marker;
int ID;
float x_coor;
float y_coor;
float z_coor;
float x_rot;
float y_rot;
float z_rot;
float w_rot;

double trans_x;
double trans_y;
double trans_z;


/**
 * @brief Struct for bot_controller compatability
 * 
 */
struct controller_options {
    std::string drive_type;
    double value;
    bool direction;
    double angle;
    double goal_x;
    double goal_y;
};

/**
 * @brief Listener function that reads frames from /tf and computes transform
 * 
 * @param tfBuffer 
 */
void listen(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "my_frame", ros::Time(0), ros::Duration(4.0));
    trans_x = transformStamped.transform.translation.x;
    trans_y = transformStamped.transform.translation.y;
    trans_z = transformStamped.transform.translation.z;
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

/**
 * @brief Squares and input
 * 
 * @param x 
 * @return double 
 */
double sq(double x){
  return x*x;
}

/**
 * @brief Computes marker location for follower bot with tolerance
 * 
 * @param marker_location 
 * @param explorer_location 
 * @param tolerance 
 * @return std::pair<double,double> 
 */
std::pair<double,double> find_point (std::pair<double, double> marker_location, std::pair<double, double> explorer_location, double tolerance){
  double dist1 = sqrt(sq(marker_location.first-explorer_location.first)+sq(marker_location.second-explorer_location.second));
  double ratio = tolerance/dist1;
  if (ratio < 0){
    ratio = ratio*(-1);
  }
  else if (ratio > 1){
    ratio = 1/ratio;
  }
  else {
    ratio = ratio;
  }
  std::pair<double, double> follower_location;
  follower_location.first = (((1-ratio)*explorer_location.first)+(ratio*marker_location.first));
  follower_location.second = (((1-ratio)*explorer_location.second)+(ratio*marker_location.second));
  return follower_location;
}

/**
 * @brief This is the callback function that will get called when a new message
 * has arrived on the Topic /fiducial_transforms.
 * 
 * @param msg 
 */
void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg){
  if (!msg->transforms.empty()) { //check marker is detected
    found_marker = true;     
    ID = msg->transforms.at(0).fiducial_id;

    //broadcaster object
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    

    // broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "my_frame"; //name of the frame
    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    x_coor = transformStamped.transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    y_coor = transformStamped.transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
    z_coor = transformStamped.transform.translation.z;

    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    x_rot = transformStamped.transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    y_rot = transformStamped.transform.rotation.x;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    z_rot = transformStamped.transform.rotation.x;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
    w_rot = transformStamped.transform.rotation.x;

    br.sendTransform(transformStamped); //broadcast the transform on /tf Topic

    
  }
  else{
    found_marker=false;
  }
}

/**
 * @brief Main loop where node activities are executed
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
  std::array<std::pair<double, double>, 4> follower_locations;
  float ang_vel=0.5;  
  bool boolean = true;
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  Bot_Controller controller(&nh);
  controller_options options;
  ros::Subscriber sub_fiducial = nh.subscribe("fiducial_transforms", 1000, &fiducial_callback);

  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  // intialize storage variables for array locations
  XmlRpc::XmlRpcValue targ_1_array;
  XmlRpc::XmlRpcValue targ_2_array;
  XmlRpc::XmlRpcValue targ_3_array;
  XmlRpc::XmlRpcValue targ_4_array;

  // get aruco marker locations with getParam() and print them to terminal
  nh.getParam("/aruco_lookup_locations/target_1", targ_1_array);
  ROS_INFO_STREAM("First target type: " << targ_1_array.getType()); //type 7 is equivalent to array in XmlRpc::XmlRpcValue class documentation 
  ROS_ASSERT(targ_1_array.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_INFO_STREAM("First target: " << targ_1_array[0] << " , " << targ_1_array[1]);

  nh.getParam("/aruco_lookup_locations/target_2", targ_2_array);
  ROS_INFO_STREAM("Second target type: " << targ_2_array.getType());
  ROS_ASSERT(targ_2_array.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_INFO_STREAM("Second target: " << targ_2_array[0] << " , " << targ_2_array[1]);

  nh.getParam("/aruco_lookup_locations/target_3", targ_3_array);
  ROS_INFO_STREAM("Third target type: " << targ_3_array.getType());
  ROS_ASSERT(targ_3_array.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_INFO_STREAM("Third target: " << targ_3_array[0] << " , " << targ_3_array[1]);

  nh.getParam("/aruco_lookup_locations/target_4", targ_4_array);
  ROS_INFO_STREAM("Fourth target type: " << targ_4_array.getType());
  ROS_ASSERT(targ_4_array.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_INFO_STREAM("Fourth target: " << targ_4_array[0] << " , " << targ_4_array[1]);

  // type converson from XmlRpc::XmlRpcValue::TypeArray to primative double type
  std::pair<double, double> targ_1_pair(targ_1_array[0],targ_1_array[1]);
  std::pair<double, double> targ_2_pair(targ_2_array[0],targ_2_array[1]);
  std::pair<double, double> targ_3_pair(targ_3_array[0],targ_3_array[1]);
  std::pair<double, double> targ_4_pair(targ_4_array[0],targ_4_array[1]);

  // declare home location coordinate pairs for explorer and follower, respectively
  std::pair<double, double> explorerhome(-4.0, 2.5);
  std::pair<double, double> followerhome(-4.0, 3.5);

  // initialize 4 goals for explorer to search
  move_base_msgs::MoveBaseGoal explorer_goal_1;
  move_base_msgs::MoveBaseGoal explorer_goal_2;
  move_base_msgs::MoveBaseGoal explorer_goal_3;
  move_base_msgs::MoveBaseGoal explorer_goal_4;

  // initialize 4 goals for follower to search
  move_base_msgs::MoveBaseGoal follower_goal_1;
  move_base_msgs::MoveBaseGoal follower_goal_2;
  move_base_msgs::MoveBaseGoal follower_goal_3;
  move_base_msgs::MoveBaseGoal follower_goal_4;

  // initialize home goals for explorer and follower
  move_base_msgs::MoveBaseGoal explorer_home;
  move_base_msgs::MoveBaseGoal follower_home;

  // build goal 1 for explorer
  explorer_goal_1.target_pose.header.frame_id = "map";
  explorer_goal_1.target_pose.header.stamp = ros::Time::now();
  explorer_goal_1.target_pose.pose.position.x = targ_1_pair.first;
  explorer_goal_1.target_pose.pose.position.y = targ_1_pair.second;
  explorer_goal_1.target_pose.pose.orientation.w = 1.0;  
  
  // build goal 2 for explorer
  explorer_goal_2.target_pose.header.frame_id = "map";
  explorer_goal_2.target_pose.header.stamp = ros::Time::now();
  explorer_goal_2.target_pose.pose.position.x = targ_2_pair.first;
  explorer_goal_2.target_pose.pose.position.y = targ_2_pair.second;
  explorer_goal_2.target_pose.pose.orientation.w = 1.0;

  // build goal 3 for explorer
  explorer_goal_3.target_pose.header.frame_id = "map";
  explorer_goal_3.target_pose.header.stamp = ros::Time::now();
  explorer_goal_3.target_pose.pose.position.x = targ_3_pair.first;
  explorer_goal_3.target_pose.pose.position.y = targ_3_pair.second;
  explorer_goal_3.target_pose.pose.orientation.w = 1.0;

  // build goal 4 for explorer
  explorer_goal_4.target_pose.header.frame_id = "map";
  explorer_goal_4.target_pose.header.stamp = ros::Time::now();
  explorer_goal_4.target_pose.pose.position.x = targ_4_pair.first;
  explorer_goal_4.target_pose.pose.position.y = targ_4_pair.second;
  explorer_goal_4.target_pose.pose.orientation.w = 1.0;

  // build home goal for explorer
  explorer_home.target_pose.header.frame_id = "map";
  explorer_home.target_pose.header.stamp = ros::Time::now();
  explorer_home.target_pose.pose.position.x = explorerhome.first;
  explorer_home.target_pose.pose.position.y = explorerhome.second;
  explorer_home.target_pose.pose.orientation.w = 1.0;

  // build goal 1 for follower
  follower_goal_1.target_pose.header.frame_id = "map";
  follower_goal_1.target_pose.header.stamp = ros::Time::now();
  follower_goal_1.target_pose.pose.position.x = 0;
  follower_goal_1.target_pose.pose.position.y = 0;
  follower_goal_1.target_pose.pose.orientation.w = 1.0;

  // build goal 2 for follower
  follower_goal_2.target_pose.header.frame_id = "map";
  follower_goal_2.target_pose.header.stamp = ros::Time::now();
  follower_goal_2.target_pose.pose.position.x = 0;
  follower_goal_2.target_pose.pose.position.y = 0;
  follower_goal_2.target_pose.pose.orientation.w = 1.0;

  // build goal 3 for follower
  follower_goal_3.target_pose.header.frame_id = "map";
  follower_goal_3.target_pose.header.stamp = ros::Time::now();
  follower_goal_3.target_pose.pose.position.x = 0;
  follower_goal_3.target_pose.pose.position.y = 0;
  follower_goal_3.target_pose.pose.orientation.w = 1.0;

  // build goal 4 for follower
  follower_goal_4.target_pose.header.frame_id = "map";
  follower_goal_4.target_pose.header.stamp = ros::Time::now();
  follower_goal_4.target_pose.pose.position.x = 0;
  follower_goal_4.target_pose.pose.position.y = 0;
  follower_goal_4.target_pose.pose.orientation.w = 1.0;

  // build home goal for explorer
  follower_home.target_pose.header.frame_id = "map";
  follower_home.target_pose.header.stamp = ros::Time::now();
  follower_home.target_pose.pose.position.x = followerhome.first;
  follower_home.target_pose.pose.position.y = followerhome.second;
  follower_home.target_pose.pose.orientation.w = 1.0;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    // broadcast();

    bool robotswitch = 0;
    // send explorer to goal 1
    if (!explorer_goal_sent && robotswitch == 0) {
      ROS_INFO("Sending goal 1 for explorer");
      explorer_client.sendGoal(explorer_goal_1);//this should be sent only once
      explorer_goal_sent = true;
    }

    // wait for explorer to reach goal
    explorer_client.waitForResult();

    //when explorer reaches goal 1, do stuff
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, explorer robot reached goal 1");
      ROS_INFO("Beginning Rotation:");
      while(boolean){
        controller.m_move(0.0, ang_vel);
        ros::spinOnce();
        if(found_marker == true){
          controller.stop();
          boolean = false;
          ROS_INFO("Marker Found");
          ROS_INFO_STREAM("FIDUCIAL ID 1: "<<ID);
          // broadcast();
          listen(tfBuffer);
          std::pair<double, double> marker_location (trans_x, trans_y);
          follower_locations.at(ID) = find_point(marker_location, targ_1_pair, 0.4);
          // follower_locations.at(ID).first = trans_x;
          // follower_locations.at(ID).second = trans_y-0.5;
          ROS_INFO_STREAM("X_coor listener: "<<follower_locations.at(ID).first);
          ROS_INFO_STREAM("Y_coor listener: "<<follower_locations.at(ID).second);

        }
      }
    }

    explorer_goal_sent = false;
    boolean = true;
    found_marker = false;
     

    // send explorer to goal 2
    if (!explorer_goal_sent && robotswitch == 0) {
      ROS_INFO("Sending goal 2 for explorer");
      explorer_client.sendGoal(explorer_goal_2);//this should be sent only once
      explorer_goal_sent = true;
    }

    explorer_client.waitForResult();

    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, explorer robot reached goal 2");
      ROS_INFO("Beginning Rotation:");

      while(boolean){
        controller.m_move(0.0, ang_vel);
        ros::spinOnce();
        if(found_marker == true){
          controller.stop();
          boolean = false;
          ROS_INFO("Marker Found");
          ROS_INFO_STREAM("FIDUCIAL ID 2: "<<ID);
          // broadcast();
          listen(tfBuffer);
          std::pair<double, double> marker_location (trans_x, trans_y);
          follower_locations.at(ID) = find_point(marker_location, targ_2_pair, 0.4);
          // follower_locations.at(ID).first = trans_x;
          // follower_locations.at(ID).second = trans_y-0.5;
          ROS_INFO_STREAM("X_coor listener: "<<follower_locations.at(ID).first);
          ROS_INFO_STREAM("Y_coor listener: "<<follower_locations.at(ID).second);
        }
      }
      
    }
    explorer_goal_sent = false;
    boolean = true;     

    // send explorer to goal 3
    if (!explorer_goal_sent && robotswitch == 0) {
      ROS_INFO("Sending goal 3 for explorer");
      explorer_client.sendGoal(explorer_goal_3);//this should be sent only once
      explorer_goal_sent = true;
    }

    explorer_client.waitForResult();

    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, explorer robot reached goal 3");
      ROS_INFO("Beginning Rotation:");
      while(boolean){
        controller.m_move(0.0, ang_vel);
        ros::spinOnce();
        if(found_marker == true){
          controller.stop();
          boolean = false;
          ROS_INFO("Marker Found");
          ROS_INFO_STREAM("Fiducial ID 3: "<<ID);
          // broadcast();
          listen(tfBuffer);
          std::pair<double, double> marker_location (trans_x, trans_y);
          follower_locations.at(ID) = find_point(marker_location, targ_3_pair, 0.4);
          // follower_locations.at(ID).first = trans_x;
          // follower_locations.at(ID).second = trans_y-0.5;
          ROS_INFO_STREAM("X_coor listener: "<<follower_locations.at(ID).first);
          ROS_INFO_STREAM("Y_coor listener: "<<follower_locations.at(ID).second);
        }
      }      
    }
    explorer_goal_sent = false;
    boolean = true;
    
    if (!explorer_goal_sent && robotswitch == 0) {
      ROS_INFO("Sending goal 4 for explorer");
      explorer_client.sendGoal(explorer_goal_4);//this should be sent only once
      explorer_goal_sent = true;
    }

    explorer_client.waitForResult();

    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, explorer robot reached goal 4");
      ROS_INFO("Beginning Rotation:");
      while(boolean){
        controller.m_move(0.0, ang_vel);
        ros::spinOnce();
        if(found_marker == true){
          controller.stop();
          boolean = false;
          ROS_INFO("Marker Found");
          ROS_INFO_STREAM("Fiducial ID 4: "<<ID);
          // broadcast();
          listen(tfBuffer);
          std::pair<double, double> marker_location (trans_x, trans_y);
          follower_locations.at(ID) = find_point(marker_location, targ_4_pair, 0.4);
          // follower_locations.at(ID).first = trans_x;
          // follower_locations.at(ID).second = trans_y-0.5;
          ROS_INFO_STREAM("X_coor listener: "<<follower_locations.at(ID).first);
          ROS_INFO_STREAM("Y_coor listener: "<<follower_locations.at(ID).second);
        }
      }
      
    }
    explorer_goal_sent = false;
    boolean = true; 

    if (!explorer_goal_sent && robotswitch == 0) {
      ROS_INFO("Sending explorer home");
      explorer_client.sendGoal(explorer_home);//this should be sent only once
      explorer_goal_sent = true;
    }

    explorer_client.waitForResult();

    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, explorer robot is home");
      explorer_goal_sent = false;
      robotswitch = 1;
    }

    // send follower to goal 1
    if (!explorer_goal_sent && robotswitch == 1) {
      ROS_INFO("Sending goal 1 for follower");
      follower_goal_1.target_pose.pose.position.x = follower_locations.at(0).first;
      follower_goal_1.target_pose.pose.position.y = follower_locations.at(0).second;
      follower_client.sendGoal(follower_goal_1);//this should be sent only once
      follower_goal_sent = true;
    }

    // wait for follower to reach goal
    follower_client.waitForResult();

    //when follower reaches goal 1, do stuff
    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, follower robot reached goal 1");
      follower_goal_sent = false;
    }

    // send follower to goal 2
    if (!explorer_goal_sent && robotswitch == 1) {
      ROS_INFO("Sending goal 2 for follower");

      follower_goal_2.target_pose.pose.position.x = follower_locations.at(1).first;
      follower_goal_2.target_pose.pose.position.y = follower_locations.at(1).second;
      follower_client.sendGoal(follower_goal_2);//this should be sent only once
      follower_goal_sent = true;
    }

    follower_client.waitForResult();

    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, follower robot reached goal 2");
      follower_goal_sent = false;
    }

    // send follower to goal 3
    if (!explorer_goal_sent && robotswitch == 1) {
      ROS_INFO("Sending goal 3 for follower");
      follower_goal_3.target_pose.pose.position.x = follower_locations.at(2).first;
      follower_goal_3.target_pose.pose.position.y = follower_locations.at(2).second;
      follower_client.sendGoal(follower_goal_3);//this should be sent only once
      follower_goal_sent = true;
    }

    follower_client.waitForResult();

    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, follower robot reached goal 3");
      follower_goal_sent = false;
    }

    // send follower to goal 4
    if (!explorer_goal_sent && robotswitch == 1) {
      ROS_INFO("Sending goal 4 for follower");
      follower_goal_4.target_pose.pose.position.x = follower_locations.at(3).first;
      follower_goal_4.target_pose.pose.position.y = follower_locations.at(3).second;
      follower_client.sendGoal(follower_goal_4);//this should be sent only once
      follower_goal_sent = true;
    }

    follower_client.waitForResult();

    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, follower robot reached goal 4");
      follower_goal_sent = false;
    }

    // send follower home
    if (!explorer_goal_sent && robotswitch == 1) {
      ROS_INFO("Sending follower home");
      follower_client.sendGoal(follower_home);//this should be sent only once
      follower_goal_sent = true;
    }

    follower_client.waitForResult();

    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, follower robot is home! ENPM809Y Final Project complete, ROS shutting down...");
      follower_goal_sent = false;
      ros::shutdown();
    }


    // broadcast();
    ros::spinOnce();
    listen(tfBuffer);    
    loop_rate.sleep();

  }


}