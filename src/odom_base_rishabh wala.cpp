#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

float gframe_distances[2]; 
double heading_angle=0, variance=0; 
double distance[2], times[2];

#define TARE 0.1

//THIS NODE IS TO BE USED IN CONJUCTION WITH THE "vect" EXECTUABLE IN THE "vectornav" PACKAGE (ROS DEVICE DRIVER)//


void gframe_components(double angle, double distance[]){
  double c = cos(angle), s = sin(angle);
  gframe_distances[0] += (c*(distance[0]) - s*(distance[1]));
  gframe_distances[1] += (s*(distance[0]) + s*(distance[1]));
}

void Angle(const sensor_msgs::MagneticField::ConstPtr& info){
  static double readings[2]={0, 0};
  readings[0]=readings[1];
  readings[1]=info->magnetic_field_covariance[1]; //this is actually the yaw angle (reference: "vectornav/src/standard.cpp")
  if(readings[0]!=0)heading_angle+=(readings[1]-readings[0])*(M_PI/180); //ensures 'angle' is not computed unless both readings[] are present. This is because the IMU gives values in absolute terms, not relative to the starting pose
  if(heading_angle>M_PI)heading_angle=(heading_angle-(2*M_PI));
  if(heading_angle<(-(M_PI)))heading_angle=((2*M_PI)+heading_angle);
}

void Linear(const sensor_msgs::Imu::ConstPtr& info){  
  //static double angle;
  static double acc[2], vels[4];
  
  //TIME//
  times[0] = times[1];
  times[1] = info->header.stamp.toSec();  
  times[0]=(times[1]-times[0]);
/*  
  //ANGLE//
  ang_vel=info->angular_velocity.z;
  angle+=(ang_vel)*(times[0]); 
*/  
  //ACCELERATIONS//
  if((((info->linear_acceleration.x)-acc[0])>TARE)||(((info->linear_acceleration.x)-acc[0])<(-TARE))) {acc[0]=-info->linear_acceleration.x;}
  if((((info->linear_acceleration.y)-acc[1])>TARE)||(((info->linear_acceleration.y)-acc[1])<(-TARE))) {acc[1]=-info->linear_acceleration.y;}
  //VELOCITIES//
  vels[0]=vels[1]; vels[2]=vels[3];
  vels[1]=(acc[0])*(times[0]); vels[3]=(acc[1])*(times[0]);
  //DISTANCES//
  distance[0]=((vels[0]+vels[1])/2)*(times[0]); distance[1]=((vels[2]+vels[3])/2)*(times[0]);
  
  gframe_components(heading_angle, distance);
  
  //PRINTING DEBUGGING INFO//
  ROS_INFO("X Acceleration: %f", acc[0]);//info->linear_acceleration.x);
  ROS_INFO("Y Acceleration: %f", acc[1]);//info->linear_acceleration.y);
  
  ROS_INFO("Distance moved in x: %f", gframe_distance[0]);
  ROS_INFO("Distance moved in y: %f", gframe_distances[1]);

//  ROS_INFO("Angular velocity about z-axis: %f", ang_vel);
  ROS_INFO("Heading info; angle: %f", heading_angle*(180/M_PI)); 
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Imu_Odom_Computation_Bitch");
  
  ros::NodeHandle imu_nh; 
  ros::CallbackQueue imu_callbacks; imu_nh.setCallbackQueue(imu_callbacks);
  ros::Subscriber imu_data_user = imu_nh.subscribe("/imu_data", 5, Linear);
  ros::Subscriber imu_mag_user = imu_nh.subscribe("/imu_mag", 5, Angle);
    
  ros::Publisher imu_poser = imu_nh.advertise<geometry_msgs::PoseStamped>("/imu_odom", 1);
  ros::Rate rt(1); 
  
  while(ros::ok()){ 
    imu_callbacks.callAvailable(ros::WallDuration(0));
    
    geometry_msgs::PoseStamped imu_pose;
    
    imu_pose.point.x=gframe_distances[0];
    imu_pose.point.y=gframe_distances[1];
    imu_pose.point.z=heading_angle;
    imu_pose.header.seq++;
    imu_pose.header.stamp=ros::Time::now();
    imu_pose.header.frame_id = "odom";
    imu_pose.child_frame_id = "base_link";
  
    imu_pose.publish(imu_pose);
    
    rt.sleep();
  }
    
  return 0;
}
