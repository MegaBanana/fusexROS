

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sstream>
#include <math.h>


struct Quaterniond
{
   double x;
   double y;
   double z;
   double w;
};

struct Quaterniond toQuaternion(double pitch, double roll, double yaw)
   {
	   struct Quaterniond q;
	   double t0 = std::cos(yaw * 0.5);
	   double t1 = std::sin(yaw * 0.5);
	   double t2 = std::cos(roll * 0.5);
	   double t3 = std::sin(roll * 0.5);
	   double t4 = std::cos(pitch * 0.5);
	   double t5 = std::sin(pitch * 0.5);

	   q.w = t0 * t2 * t4 + t1 * t3 * t5;
	   q.x = t0 * t3 * t4 - t1 * t2 * t5;
	   q.y = t0 * t2 * t5 + t1 * t3 * t4;
	   q.z = t1 * t2 * t4 - t0 * t3 * t5;
	   return q;
   }


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "SendIMU");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 1000);

  ros::Rate loop_rate(10);


   //string arduinoPort "/dev/ttyACM3/
   
  // sendIMU();




  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    geometry_msgs::PoseWithCovarianceStamped msg;


    //msg.header = ros::Time::Header();
    msg.header.frame_id = "base_link";
   // msg.header.stamp = ros::Time::now();
    
    double pitch  = 0;
    double roll   = 0;
    double yaw    = M_PI;
    
    struct Quaterniond quat = toQuaternion(pitch, roll, yaw);
    
    msg.pose.pose.position.x = 0;
    msg.pose.pose.position.y = 0;
    msg.pose.pose.position.z = 0;
    msg.pose.pose.position.x = 0; 
    
    
    msg.pose.pose.orientation.x = quat.x;
    msg.pose.pose.orientation.y = quat.y;
    msg.pose.pose.orientation.z = quat.z;
    msg.pose.pose.orientation.x = quat.x; 


    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}


void sendIMU()
{

}
