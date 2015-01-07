#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

  double x = 0.0;
  double x_prev = 0.0;
  double y = 0.0;
  double y_prev = 0.0;
  double th = 0.0;
  double th_prev = 0.0;
  double vx = 0.0;
  double vx_prev = 0.0;
  double vy = 0.0;
  double vy_prev = 0.0;
  double vth = 0.0;
  double ax;
  double ay;
//It is not advisable to use timestamps taken from the 'main' function. I've incorporated time-stamping so 
//that it's taken from the incoming messages itself, that reflects a truer picture of 
//when the measurments were taken
//  ros::Time current_time, last_time;
//  float time_diff = 0.0;


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	//The time stuff is in this block
	const float time[2] = {0.0, 0.0};
	time[0] = time[1];
	time[1] = msg -> header.stamp.toSec();
	time[0] = time[1] - time[0];
	
    ::ax = msg -> linear_acceleration.x;
    ::ay = msg -> linear_acceleration.y;
    
    //The IMU isn't giving you velocity values
    vx_prev = vx;
    vy_prev = vy;
    vx = vx_prev + ax *(time_diff);
    vy = vy_prev + ay *(time_diff);
    
    x_prev = x;
    y_prev = y;
    x = x_prev + 0.5*ax*(time[0])*(time[0]);
    y = y_prev + 0.5*ay*(time[0])*(time[0]);
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    th_prev = th;
    th = msg -> magnetic_field_covariance[1]; //th is a 'double', what it's being assigned is a float...pain ho sakta hai
  //  vth = (th - th_prev)/(::current_time - ::last_time);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  
  ::current_time = ros::Time::now();
  ::last_time = ros::Time::now();

  ros::Rate r(15.0);
  
  while(n.ok())
  {

    ros::spinOnce();               // check for incoming messages
    ::current_time = ros::Time::now();
    //time_diff = (::current_time.toSec() - ::last_time.toSec());
    ros::Subscriber mag_sub= n.subscribe("/imu_mag",1,magCallback);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = ::current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    nav_msgs::Odometry odom;

    ros::Subscriber imu_sub= n.subscribe("/imu_data",1,imuCallback);
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;

    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
    ::last_time = ::current_time;
    r.sleep();
  }
}
