#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <eigen3/Eigen/Dense>
#include <Eigen/Geometry>  hondae 
#include <cmath>
#include <vector>
//#include <tf.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  int i,j;
  double d = 0.5;
  double theta20= 0.8;
  double theta1= 0.2;
  double theta2= 0.3;
  double PI=3.14;
  double l1 = 0.164544, l2= 0.164544, l3=0.435295, l4=0.285, r1=0.125,r2=0.125,r3=0.125; 
  double gamma1 = 1.57 , gamma2 = 1.57, gamma3 = 1.57;


  double thetaR_P;
  double Beta;
  double d_P_A1x, d_P_A1z, dPA1;
  double d_P_A2x, d_P_A2z, dPA2;

  
  thetaR_P = PI/2+theta2-theta1+Beta;
  Beta = asin(l2*l2+l4*l4-l1*l1)/(2*l2*l4);
  dPA1 = (l4*l4+l1*l1-l2*l2)/(2*l4);
  d_P_A1x = dPA1;
  d_P_A1z = -sqrt (l1*l1-(dPA1*dPA1));
  dPA2 = (l2*l2+l4*l4-l1*l1)/(2*l4);
  d_P_A2x = -dPA2;
  d_P_A2z = -sqrt (l2*l2-(dPA2*dPA2));

  Eigen::Matrix4d O_A1, O_A2, O_A3;





    Eigen::Matrix4d O_R;

  	O_R <<     1,   0,   0,  -d*sin(theta20),
        	   0,   1,   0,       0,
        	   0,   0,   1,  -d*cos(theta20),
               0,   0,   0,       1; 

    Eigen::Matrix4d R_P;

  	R_P <<     cos(thetaR_P),   0,   sin(thetaR_P),  0,
        	   0,   			1, 		    0, 		 0,
        	   -sin(thetaR_P),  0,   cos(thetaR_P),  0,
               0,               0,   		0,     	 1; 

    Eigen::Matrix4d P_A1;

  	P_A1 <<    1,   0,   0,  d_P_A1x,
        	   0,   1,   0,     0,
        	   0,   0,   1,  d_P_A1z,
               0,   0,   0,     1; 

    Eigen::Matrix4d P_A2;

  	P_A2 <<    1,   0,   0,  d_P_A2x,
        	   0,   1,   0,     0,
        	   0,   0,   1,  d_P_A2z,
               0,   0,   0,     1; 

    Eigen::Matrix4d R_A3;

  	R_A3 <<    1,   0,   0,   -l3,
        	   0,   1,   0,     0,
        	   0,   0,   1,     0,
               0,   0,   0,     1; 

    Eigen::Matrix4d A1_W1;

  	A1_W1 <<   cos(PI/2-theta1-Beta),   0,   sin(PI/2-theta1-Beta),  0,
        	   0,   					1, 		    0, 		 		 0,
        	   -sin(PI/2-theta1-Beta),  0,   cos(PI/2-theta1-Beta),  0,
               0,               		0,   		0,     			 1; 

    Eigen::Matrix4d A2_W2;

  	A2_W2 <<   cos(PI/2-theta1-Beta),   0,   sin(PI/2-theta1-Beta),  0,
        	   0,   					1, 		    0, 		 		 0,
        	   -sin(PI/2-theta1-Beta),  0,   cos(PI/2-theta1-Beta),  0,
               0,               		0,   		0,     			 1; 

    Eigen::Matrix4d A3_W3;

  	A3_W3 <<   cos(theta2),    0,   sin(theta2),  0,
        	   0,   		   1, 	    0, 		  0,
        	   -sin(theta2),   0,   cos(theta2),  0,
               0,              0,   	0,     	  1; 

 	Eigen::Matrix4d W1_C1;

  	W1_C1 <<   1,    0,     0,   r1*sin(gamma1),
        	   0,    1, 	0, 		  0,
        	   0,    0,     0,  -r1*cos(gamma1),
               0,    0,   	0,     	  1; 

    Eigen::Matrix4d W2_C2;

  	W2_C2 <<   1,    0,     0,   r2*sin(gamma2),
        	   0,    1, 	0, 		  0,
        	   0,    0,     0,  -r2*cos(gamma2),
               0,    0,   	0,     	  1; 

    Eigen::Matrix4d W3_C3;

  	W3_C3 <<   1,    0,     0,   r3*sin(gamma3),
        	   0,    1, 	0, 		  0,
        	   0,    0,     0,  -r3*cos(gamma3),
               0,    0,   	0,     	  1; 

    O_A1 = O_R * R_P * P_A1;
    O_A2 = O_R * R_P * P_A2;
    O_A3 = O_R * R_A3;
    double dO_A1x=O_A1(0,3);
    double dO_A1z=O_A1(2,3);
    double dO_A2x=O_A1(0,3);
    double dO_A2z=O_A1(2,3);
    double dO_A3x=O_A1(0,3);
    double dO_A3z=O_A1(2,3);

    Eigen::Vector3d J1;

  	J1    <<   r1*(cos(gamma1)*cos(theta2)+sin(gamma1)*sin(theta2))-dO_A1z,
  			   r1*(cos(gamma1)*cos(theta2)+sin(gamma1)*sin(theta2))-dO_A1x,     
  										0; 

  	Eigen::Vector3d J2;

  	J2    <<   r2*(cos(gamma1)*cos(theta2)+sin(gamma2)*sin(theta2))-dO_A2z,
  			   r2*(cos(gamma1)*cos(theta2)+sin(gamma2)*sin(theta2))-dO_A2x,     
  										0; 

  	Eigen::Vector3d J3;

  	J3    <<   r3*(cos(gamma3)*cos(theta2)+sin(gamma3)*sin(theta2))-dO_A3z,
  			   r3*(cos(gamma3)*cos(theta2)+sin(gamma3)*sin(theta2))-dO_A3x,     
  										0; 

    Eigen::MatrixXd J(9, 3);

    J    <<    J1(0), 0, 0,
               J1(1), 0, 0,
               J1(2), 0, 0,
               0, J2(0), 0,
               0, J2(1), 0,
               0, J2(2), 0,
               0, 0, J3(0),
               0, 0, J3(1),
               0, 0, J3(2); 

    Eigen::MatrixXd J_T(3, 9);
    J_T=J.transpose();
    Eigen::Matrix4d J_I;
    J_I=W3_C3.inverse();

	Eigen::Matrix3d q;
	q=J_T*J;
	Eigen::Matrix3d q_I;
	q_I=q.inverse();	
	Eigen::MatrixXd q_I_J_T(3, 9);
	q_I_J_T= q_I*J_T;
	Eigen::MatrixXd identity_9_3(9, 3);
    identity_9_3   <<  1, 0, 0,
		               0, 1, 0,
		               0, 0, 1,
		               1, 0, 0,
		               0, 1, 0,
		               0, 0, 1,
		               1, 0, 0,
		               0, 1, 0,
		               0, 0, 1; 
	Eigen::Matrix3d q_1;
	q_1=q_I_J_T*identity_9_3;
	double V_Bx= 0.5, V_Bz=0.8, W_B=0.3;
	Eigen::Vector3d Bpb;
  	Bpb    <<   V_Bx,
  			   	V_Bz,     
				W_B; 
	Eigen::Vector3d qdot;
 	qdot=q_1*Bpb;




    
    //Eigen::Vector4d dO_Ai=O_A1.col(3);
 

    //std::cout << O_R << std::endl;
    //std::cout << R_P << std::endl;
    //std::cout << P_A1 << std::endl;
    //std::cout << P_A2 << std::endl;
    //std::cout << R_A3 << std::endl;
    //std::cout << A1_W1 << std::endl;
    //std::cout << A2_W2 << std::endl;
    //std::cout << A3_W3 << std::endl;
    //std::cout << W1_C1 << std::endl;
    //std::cout << W2_C2 << std::endl;
    //std::cout << W3_C3 << std::endl;
    std::cout << O_A1 << std::endl;
    std::cout << O_A2 << std::endl;
    std::cout << O_A3 << std::endl;
    std::cout << dO_A1x << std::endl;
    std::cout << dO_A1z << std::endl;
    std::cout << J1 << std::endl;
	std::cout << J2 << std::endl;
	std::cout << J3 << std::endl;
	std::cout << J << std::endl;
	std::cout << J_T << std::endl;
	std::cout << J_I << std::endl;
	std::cout << q << std::endl;
	std::cout << q_I << std::endl;
	std::cout << q_I_J_T << std::endl;
	std::cout << identity_9_3 << std::endl;
	std::cout << q_1 << std::endl;
	std::cout << qdot << std::endl;
	
	
	




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
