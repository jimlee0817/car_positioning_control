#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"
#include <cmath>
#include <sstream>

using namespace std;

ros::Publisher velocity_pub;
ros::Subscriber goal_sub;
ros::Subscriber pose_sub;


const double PI = 3.14159265359;
double x_goal;
double y_goal;
double theta_goal;
double x, y, z, w;
double x_pose, y_pose, theta_pose;
double constrainAngle(double x){
     
       //x = fmod(x+PI,PI*2);
       //if (x < 0)
          //x += PI*2;
       x = fmod(x,PI*2);
       if (x>=PI)
          x = -(PI*2-x);

       
       //return x-PI;
       return x;
}
double limitTurn(double x){
       if (x>PI)
          x = -(2*PI-x);
      
       else if(x<-PI)
          x = (2*PI+x);     
       //return x-PI;
       return x;
}
int goal_flag = 0;

void goal_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{

    x_goal = msg->pose.position.x;
    y_goal = msg->pose.position.y;
    x = msg->pose.orientation.x;
    y = msg->pose.orientation.y;
    z = msg->pose.orientation.z;
    w = msg->pose.orientation.w;
    /*********angle transformation**********/
    theta_goal = atan2(2*(w*z+x*y),1-2*(z*z+y*y));
    //theta_goal = constrainAngle(theta_goal);
    theta_goal = fmod(theta_goal,PI*2);
    //theta_goal = theta_goal - PI;
    ROS_INFO("theta_goal is %f", theta_goal/PI*180);
    //ROS_INFO("the position(x,y,theta) is %f , %f, %f",x_goal,y_goal,theta_goal);
    goal_flag = 1;    
}
    
void pose_Callback(const geometry_msgs::Twist::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{
    x_pose = msg->linear.x;
    y_pose = msg->linear.y;
    theta_pose =  msg->angular.z;
    theta_pose = constrainAngle(theta_pose);
    //ROS_INFO("theta_pose_callback is %f", theta_pose/PI*180);
    //theta_pose = fmod(theta_pose,PI*2);
}
int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlebot_controller");
	ros::NodeHandle n;
        goal_sub = n.subscribe("/move_base_simple/goal", 1000,goal_Callback);
        pose_sub = n.subscribe("/robot_pose", 1000,pose_Callback);
	velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        ros::Rate loop_rate(10);
        geometry_msgs::Twist vel_msg;
        double Ts, t, r, dTol, l, D, v, w, phiT, alpha, beta;

	while(ros::ok()){
                geometry_msgs::PoseStamped goal_msg; 
                //geometry_msgs::Twist pose_msg;
                //geometry_msgs::Twist vel_msg;
                

                dTol = 0.01; // Tolerance distance (to the intermediate point) for switch
                /*********Build the system spec**********/
                //l = 1.0; 
                //r = 1.0;

                double AngleD = theta_goal - theta_pose;
                double AngleT = 0.006;
                ROS_INFO ("AngleD: %f", AngleD/PI*180);
                //D = sqrt(pow(goal_msg.pose.position.x-pose_msg.linear.x,2) + pow(goal_msg.pose.position.y-pose_msg.linear.y,2));
                D = sqrt(pow(x_goal-x_pose,2) + pow(y_goal-y_pose,2));
                if ((D<dTol) || (goal_flag == 0))// Stop when close to the goal
                {
                   v = 0.0;
                   //w = 0.0;
                   vel_msg.linear.x = v;
                   vel_msg.angular.z = w;
                   velocity_pub.publish(vel_msg);
                   goal_flag = 0;
		   if(std::abs(AngleD) < AngleT)
		   {
		      w = 0;
		   }
                }
                else
                {
                   phiT = atan2(y_goal-y_pose, x_goal-x_pose);
                   //phiT = fmod(phiT,2*PI);
                   
                   double phiT_deg = phiT/PI*180;
                   ROS_INFO ("phiT: %f", phiT_deg);

                   alpha = phiT - theta_pose;
                   alpha = fmod(alpha,2*PI);
                   alpha = limitTurn(alpha);

                   beta = theta_goal-theta_pose-alpha;
                   //beta = theta_goal-theta_pose;
                   beta = fmod(beta,2*PI);
                   beta = limitTurn(beta);
                   
                   //ROS_INFO ("theta_goal_degree: %f", theta_goal/PI*180);
                   double theta_pose_degree =  theta_pose/PI*180;
                   //double alpha_degree =  alpha/PI*180;
                   //double theta_goal_degree =  theta_goal/PI*180;
                   //double beta_degree =  beta/PI*180;
                   ROS_INFO ("theta_pose: %f", theta_pose/PI*180);
                   //beta = theta_goal-theta_pose;
                   //ROS_INFO ("beta: %f", beta/PI*180);
                   /*********controller**********/
                   //ROS_INFO ("theta_pose_degree: %f", theta_pose_degree);
                   //ROS_INFO ("alpha: %f", alpha/PI*180);
                   //ROS_INFO ("theta_goal_degree: %f", theta_goal/PI*180);
                   //ROS_INFO ("theta_goal: %f", theta_goal);
                   v = D*0.3;
                   //ROS_INFO ("D: %f", D);
                   w = alpha*0.8+beta*(-0.5);
                   //ROS_INFO ("w: %f", w);
                   vel_msg.linear.x = v;
                   vel_msg.angular.z = w;
                   velocity_pub.publish(vel_msg); 
                }                  

                
                        
               
		/** run the clean application afer you implement it*/
		double t0 = ros::Time::now().toSec(); //get current time before cleaning
		//clean();
		double t1 = ros::Time::now().toSec(); //get current time after cleaning
		//ROS_INFO ("Cleaning execution time: %.2f", (t1-t0));
		ros::spinOnce();
                loop_rate.sleep();

	}

	

   return 0;
}
