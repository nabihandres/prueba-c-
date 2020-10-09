#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry> 
//#include <tf.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  int i,j;

 

    Eigen::Matrix4f Tm;
  	Tm <<     0.714191,   -0.54278,   0.441988,  0.0432322,
        0.409255,   0.836069,    0.36542,  0.0571429,
        -0.567861, -0.0800918,   0.819232,    1.22178,
            0,          0,          0,       1; 

    std::cout << Tm << std::endl;


  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    //std_msgs::String msg;

    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    //chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    //++count;
  }

  return 0;
}