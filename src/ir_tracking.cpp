#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#define MODE_TRACKING 1
#define MODE_STOP 2
#define MODE_FIND_PERSON_CW 3
#define MODE_FIND_PERSON_CCW 4
int MODE = MODE_STOP; 
double pre_v=0,pre_w=0;
double target_v=0,target_w=0;
double delta_v=0,delta_w=0;

double min_dist=25.0;
double keep_range_in=30.0,keep_range_out=50.0;
double track_dist=min_dist+keep_range_in;
double max_dist=min_dist+keep_range_in+keep_range_out;
double limit_dist=150;
double target_d,target_d_pre;
double target_th=0,target_th_pre=0;
double error_d=0,error_d_sum=0,error_th=0,error_th_sum=0;

double target_x,target_y,target_x_pre,target_y_pre;
double target_x_ir=track_dist;
double target_x_ir_pre=track_dist;
double target_y_ir=0,target_y_ir_pre=0;
double ml=0,mr=0;
double max_scale=0.6;
double track_mode=0;
double camera_x=0,camera_y=0,camera_x_pre=0,camera_y_pre=0;
double people_x=0,people_y=0;
double lim_dv,lim_dw,lim_tv,lim_tw;


double min( double arr[], int len )
{
    double min = arr[0];

    for ( int i = 1; i < len; i++ )
        if ( arr[i] < min )
            min = arr[i];

    return min;
}

class IR_Tracking
{
  public:
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_,target_xy_pub_ir_,people_pub_,mode_pub_;
  ros::Subscriber cmd_sub_,camera_sub_;
  IR_Tracking()
  {
    cmd_sub_ = nh_.subscribe("/sensor/ir_value", 1 ,&IR_Tracking::DistCB,this);
    camera_sub_ = nh_.subscribe("/target_xywh", 1 ,&IR_Tracking::CameraCB,this);


    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/andbot/cmd_vel", 1);
    target_xy_pub_ir_ = nh_.advertise<geometry_msgs::Vector3>("/target_xy_ir", 1);
    people_pub_ = nh_.advertise<geometry_msgs::Vector3>("/target_people", 1);
    mode_pub_ = nh_.advertise<std_msgs::Int8>("/sensor/ir_mode", 1);
 

  }

void CameraCB(const geometry_msgs::Vector3& msg){
  camera_x=0.3*msg.x+0.7*camera_x_pre;
  camera_y=0.3*msg.y+0.7*camera_y_pre;

}
void DistCB(const geometry_msgs::Vector3& msg){
  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
  geometry_msgs::Vector3Ptr target_xy_cmd(new geometry_msgs::Vector3());
  geometry_msgs::Vector3Ptr target_people_xy_cmd(new geometry_msgs::Vector3());
  std_msgs::Int8Ptr mode_msg(new std_msgs::Int8());


  double ir_value[3]={msg.x,msg.y,msg.z};
  double ir_pos[3]={-10.0,0.0,10.0};
  double m1=0,m2=0;
/*------------------------------
// limit range 160cm
-------------------------------*/
  for (int i=0;i<3;i++)
  {
	if (ir_value[i]>limit_dist)
	  ir_value[i]=limit_dist;
  }


///////////////////////////////////////////////////
//  sensor ir_value to decision target pos
//double d10=ir_value[1]-ir_value[0];
//double d20=ir_value[2]-ir_value[0];
//double d10=ir_value[1]-ir_value[0];
//double d20=ir_value[2]-ir_value[0];
//double d21=ir_value[2]-ir_value[1];


  if (ir_value[0]>max_dist && ir_value[1]>max_dist && ir_value[2]>max_dist){   
    target_x_ir=track_dist;
    target_y_ir=0;
    mode_msg->data=100;
  }
  if (ir_value[0]<max_dist && ir_value[1]>max_dist && ir_value[2]>max_dist){   
    target_x_ir=ir_value[0];
    target_y_ir=30;
    mode_msg->data=2;
  }
  if (ir_value[0]<max_dist && ir_value[1]<max_dist && ir_value[2]>max_dist){   
    target_x_ir=(ir_value[0]+ir_value[1])/2;
    target_y_ir=15;
    mode_msg->data=1;
  }
  if (ir_value[0]<max_dist && ir_value[1]<max_dist && ir_value[2]<max_dist){   
    target_x_ir=(ir_value[0]+ir_value[1]+ir_value[2])/3;
    target_y_ir=0;
    mode_msg->data=0;
  }
  if (ir_value[0]>max_scale*max_dist && ir_value[1]<max_dist && ir_value[2]<max_dist){   
    target_x_ir=(ir_value[1]+ir_value[2])/2;
    target_y_ir=-15;
    mode_msg->data=-1;
  }
  if (ir_value[0]>max_scale*max_dist && ir_value[1]>max_scale*max_dist && ir_value[2]<max_dist){   
    target_x_ir=ir_value[2];
    target_y_ir=-30;
    mode_msg->data=-2;
  }
    mode_pub_.publish(mode_msg);



/*
  target_x_ir=min(ir_value,3);
  ml=(float)(ir_value[0]-ir_value[1])/10;
  mr=(float)(ir_value[1]-ir_value[2])/10;

  target_y_ir=-(ml+mr)*3;
  if (target_y_ir >=20)
    target_y_ir=20;
  if (target_y_ir <=-20)
    target_y_ir=-20;
*/


//////////////////////////////////////////////////////
//  ir sensor target filter 
    target_x_ir=0.8*target_x_ir + 0.2*target_x_ir_pre;
    target_y_ir=0.8*target_y_ir + 0.2*target_y_ir_pre;

   target_x_ir_pre=target_x_ir;
    target_y_ir_pre=target_y_ir;

    target_xy_cmd->x=target_x_ir;
    target_xy_cmd->y=target_y_ir;
    target_xy_pub_ir_.publish(target_xy_cmd);
    
//////////////////////////////////////////////////////////
//  sensor fusion
    target_x=target_x_ir;
    target_y=1*target_y_ir+0*camera_x;
/////////////////////////////////////////////////////////
//  final position filter 
    target_x=1*target_x + 0*target_x_pre;
    target_y=1*target_y + 0*target_y_pre;
    target_x_pre=target_x;
    target_y_pre=target_y;

    target_people_xy_cmd->x=target_x;
    target_people_xy_cmd->y=target_y;
    people_pub_.publish(target_people_xy_cmd);
//////////////////////////////////////////////////////////
//  target_x target_y 
//  to 
//  target_d target_th

    target_th=atan2(target_y,target_x);
    target_d=sqrt(target_y*target_y+target_x*target_x);
/////////////////////////////////////////////////////////
//  PI Control to calculate target_v and target_w
    error_d=target_d-track_dist;
    error_d_sum=error_d_sum+error_d;
    if(error_d_sum>70) error_d_sum=70;
    if(error_d_sum<-70) error_d_sum=-70;

    target_v=0.03*(error_d)+0.001*error_d_sum;
    target_w=2*target_th;
    ROS_INFO("error_d %f error_d_sum %f",error_d,error_d_sum);
//////////////////////////////////////////////////////////
//  move limition

/*
    if (mode_msg->data==100)   { 
      lim_tv=0.5,lim_dv=(float)lim_tv/5;
      lim_tw=1,lim_dw=(float)lim_tw/5;
    } 
    else if(mode_msg->data==-2||mode_msg->data==-1||mode_msg->data==1||mode_msg->data==2){
      lim_tv=0.5,lim_dv=(float)lim_tv/5;
      lim_tw=1;lim_dw=(float)lim_tw/5;
    }
    else if(mode_msg->data==0){
      lim_tv=0.5,lim_dv=(float)lim_tv/5;
      lim_tw=1,lim_dw=(float)lim_tw/5;
    }
*/

    lim_tv=0.8,lim_dv=(float)lim_tv/5;
    lim_tw=0.2,lim_dw=(float)lim_tw/2;
    if(mode_msg->data==-2||mode_msg->data==-1||mode_msg->data==1||mode_msg->data==2){
      lim_tv=0.8,lim_dv=(float)lim_tv/5;
      lim_tw=1.5;lim_dw=(float)lim_tw/2;
    }


    delta_v=target_v-pre_v;

    if (delta_v>=lim_dv) delta_v=lim_dv;
    else if (delta_v<=-lim_dv) delta_v=-lim_dv;

    delta_w=target_w-pre_w;
    if (delta_w>=lim_dw) delta_w=lim_dw;
    else if (delta_w<=-lim_dw) delta_w=-lim_dw;

    target_v=pre_v+delta_v;
    target_w=pre_w+delta_w;
    
    if (target_v>=lim_tv) target_v=lim_tv;
    else if (target_v<=-lim_tv) target_v=-lim_tv;
    if (target_w>=lim_tw) target_w=lim_tw;
    else if (target_w<=-lim_tw) target_w=-lim_tw;

    cmd->linear.x=target_v;
    cmd->angular.z=target_w;
    cmd_pub_.publish(cmd);
    pre_v=target_v;
    pre_w=target_w;


//   ROS_INFO("v %f w %f",target_v,target_w);
//   ROS_INFO("x %f y %f d %f th %f",target_x,target_y,target_d,target_th);

//   ROS_INFO("ir_value %f %f %f ",ir_value[0],ir_value[1],ir_value[2]);
//   ROS_INFO("ir_value_f[1] %f",ir_value_f[1]);
//   ROS_INFO("ir_value_f[2] %f",ir_value_f[2]);
/*
  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());


  cmd->linear.x = msg.x;
  cmd->angular.z = 0;
  cmd_pub_.publish(cmd);
*/

}


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ir_tracking");
  IR_Tracking it;
  ros::spin();
  return 0;
}
