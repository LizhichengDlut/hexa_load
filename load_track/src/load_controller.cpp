//     2023.4.18
//Writen by Lizhicheng
//email:li_zhicheng@dlut.edu.cn
#include "ros/ros.h"

#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include "eigen3/Eigen/Dense"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mavros_msgs/PositionTarget.h>
#include <dynamic_reconfigure/server.h>
#include "load_track/load_trackConfig.h"
#include "gazebo_msgs/LinkStates.h"

#define TRAJ_CIRCLE 1
#define TRAJ_LAMNISCATE 2
#define TRAJ_STATIONARY 3

using namespace std;
using namespace Eigen;

const double g (9.8);
const double mq(1.2),ml(0.5);
const double l(0.5);
const Eigen::Vector3d e3(0,0,1);
const Eigen::Vector3d k(0.1,0.1,0.2);
const Eigen::Vector3d kx(0.5,0.5,1.0);
const Eigen::Vector3d kv(0.1,0.1,0.2);
double kq(0.4);
double k_omega(0.04);
double Kpos_x_(8), Kpos_y_(8), Kpos_z_(10), Kvel_x_(1.5), Kvel_y_(1.5), Kvel_z_(3.3), Ki_x_(0.01), Ki_y_(0.01), Ki_z_(0.5);
Eigen::Vector3d ex;
Eigen::Vector3d ev;
Eigen::Vector3d eq;                       //error of q;
Eigen::Vector3d edotq;                    //error of dot(q);
Eigen::Vector3d x,v,xl,vl,omega_l;
double traj_omega = 1.3;
const Eigen::Vector3d traj_axis(0,0,1);
const Eigen::Vector3d omega_d(traj_omega*e3);
const Eigen::Vector3d traj_radial(1,0,0);
const Eigen::Vector3d traj_origin(0,0,1.5);
const double radius (5);
int type(1);                              //traj_type
Eigen::Vector3d x_target(0,0,1),xd,vd,ad(0,0,0),qd(0,0,0),q_dotd(0,0,0);
Eigen::Vector3d q,dot_q;
Eigen::Vector3d A;
Eigen::Vector3d Fn,Fpd,Fff,F;

Eigen::Matrix3d hat(const Eigen::Vector3d x)
{
  Eigen::Matrix3d Mat3;
  Mat3 << 0, -x(2), x(1),
                  x(2), 0, -x(0),
                 -x(1), x(0), 0;
  return Mat3;
}

double innerProduct(Eigen::Matrix3d x1, Eigen::Matrix3d x2)
{
  double R = x1(0,0)*x2(0,0)+x1(0,1)*x2(0,1)+x1(0,2)*x2(0,2)+
             x1(1,0)*x2(1,0)+x1(1,1)*x2(1,1)+x1(1,2)*x2(1,2)+
             x1(2,0)*x2(2,0)+x1(2,1)*x2(2,1)+x1(2,2)*x2(2,2);

  return R;
}

Eigen::Vector3d toEigen(const geometry_msgs::Point& p)
{
    Eigen::Vector3d ev3(p.x,p.y,p.z);
    return ev3;
}

Eigen::Vector3d toEigen(const geometry_msgs::Vector3& v3)
{
    Eigen::Vector3d ev3(v3.x,v3.y,v3.z);
    return ev3;
}

Eigen::Vector3d toUnitVector(const geometry_msgs::Quaternion& q)
{
    Eigen::Matrix3d R;
    Eigen::Quaterniond Q(q.w, q.x, q.y, q.z );
  
    R = Q.toRotationMatrix();
    
    // R(0,0) = 1-2*q.y*q.y-2*q.z*q.z;
    // R(0,1) = 2*q.x*q.y+2*q.w*q.z;
    // R(0,2) = 2*q.x*q.z-2*q.w*q.y;
    // R(1,0) = 2*q.x*q.y-2*q.w*q.z;
    // R(1,1) = 1-2*q.x*q.x-2*q.z*q.z;
    // R(1,2) = 2*q.y*q.z+2*q.w*q.x;
    // R(2,0) = 2*q.x*q.z+2*q.w*q.y;
    // R(2,1) = 2*q.y*q.z-2*q.w*q.x;
    // R(2,2) = 1-2*q.x*q.x-2*q.y*q.y;

    Eigen::Vector3d unit_vector = R*e3;
  
    return unit_vector;
}

Eigen::Vector3d getPosition(double time)
{
    Eigen::Vector3d position;
    double theta;
    switch(type){
        case TRAJ_CIRCLE:
        theta = traj_omega *time;
        position = std::cos(theta) * traj_radial + std::sin(theta) * traj_axis.cross(traj_radial) +
                 (1 - std::cos(theta)) * traj_axis.dot(traj_radial) * traj_axis + traj_origin;
        break;
        case TRAJ_LAMNISCATE:
        position = x_target;
        // theta = traj_omega * time;
        // position = std::cos(theta) * traj_radial + std::sin(theta) * std::cos(theta) * traj_axis.cross(traj_radial) +
        //          (1 - std::cos(theta)) * traj_axis.dot(traj_radial) * traj_axis + traj_origin;
        break;
        case TRAJ_STATIONARY:  // Lemniscate of Genero

        position << 0.0, 0.0, 1.0;
        break;

    }

    return position;
}

Eigen::Vector3d getVelocity(double time) {
  Eigen::Vector3d velocity;
  double theta;

  switch (type) {
    case TRAJ_CIRCLE:

      velocity = traj_omega * traj_axis.cross(getPosition(time));
      break;
    case TRAJ_STATIONARY:

      velocity << 0.0, 0.0, 0.0;
      break;

    case TRAJ_LAMNISCATE:  // Lemniscate of Genero

    velocity <<0.0, 0.0, 0.0;

      // theta = traj_omega * time;
      // velocity = traj_omega *
      //            (-std::sin(theta) * traj_radial +
      //             (std::pow(std::cos(theta), 2) - std::pow(std::sin(theta), 2)) * traj_axis.cross(traj_radial) +
      //             (std::sin(theta)) * traj_axis.dot(traj_radial) * traj_axis);
      break;

    default:
      velocity << 0.0, 0.0, 0.0;
      break;
  }
  return velocity;
}

Eigen::Vector3d getAcceleration(double time) {
  Eigen::Vector3d acceleration;

  switch (type) {
    case TRAJ_CIRCLE:

      acceleration = traj_omega * traj_axis.cross(getVelocity(time));
      break;
    case TRAJ_STATIONARY:
    
      acceleration << 0.0, 0.0, 0.0;
      break;
    case TRAJ_LAMNISCATE:
      acceleration << 0.0, 0.0, 0.0;
    default:
      acceleration << 0.0, 0.0, 0.0;
      break;
  }
  return acceleration;
}

void link_0PoseCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{

    xl = toEigen(msg->pose[4].position);
    // ROS_INFO("[%f]",msg->pose[4].position.x);//subscribe position and velocity
    vl = toEigen(msg->twist[4].linear);
    omega_l = toEigen(msg->twist[4].angular);
    q = toUnitVector(msg->pose[4].orientation);
}

void posCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  x = toEigen(msg->pose.position);
}

void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  v = toEigen(msg->twist.linear);
}

void flagCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (msg->child_frame_id == "1")
    type = 3;
  else if (msg->child_frame_id == "2")
  {
    type = 2;
    x_target = toEigen(msg->pose.pose.position);
  }
  else if(msg->child_frame_id == "3")
  type = 1;
}

void ReconfigCallback(load_track::load_trackConfig &config, uint32_t level)
{
  if (traj_omega != config.omega_d) {
    traj_omega = config.omega_d;
    ROS_INFO("Reconfigure request : omega = %.2f ", config.omega_d);
  } else if (Kpos_x_ != config.Kp_x) {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
  } else if (Kpos_y_ != config.Kp_y) {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
  } else if (Kpos_z_ != config.Kp_z) {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
  } else if (Kvel_x_ != config.Kv_x) {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
  } else if (Kvel_y_ != config.Kv_y) {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y = %.2f  ", config.Kv_y);
  } else if (Kvel_z_ != config.Kv_z) {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z = %.2f  ", config.Kv_z);
  }else if (Ki_x_ != config.Ki_x) {
    Ki_x_ = config.Ki_x;
    ROS_INFO("Reconfigure request : Ki_x  = %.2f  ", config.Ki_x);
  } else if (Ki_y_ != config.Ki_y) {
    Ki_y_ = config.Ki_y;
    ROS_INFO("Reconfigure request : Ki_y = %.2f  ", config.Ki_y);
  } else if (Ki_z_ != config.Ki_z) {
    Ki_z_ = config.Ki_z;
    ROS_INFO("Reconfigure request : Ki_z = %.2f  ", config.Ki_z);
  }
}

/*
main function
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "load_controller");
    ros::NodeHandle n;


    // ros::Publisher accelPub = n.advertise<geometry_msgs::Vector3Stamped>("/mavros/setpoint_accel/accel",100);
    // ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped",100);
    // ros::Publisher forcePub = n.advertise<mav_msgs::RollPitchYawrateThrust>("/roll_pitch_yawrate_thrust_command",100);
    ros::Publisher setrawPub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",100);
    mavros_msgs::PositionTarget msg;
    msg.header.stamp = ros::Time::now();
    static int seq = 1;
    msg.header.seq = seq++;
    msg.header.frame_id = 1;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX +
                    mavros_msgs::PositionTarget::IGNORE_PY +
                    mavros_msgs::PositionTarget::IGNORE_PZ +
                    mavros_msgs::PositionTarget::IGNORE_VX +
                    mavros_msgs::PositionTarget::IGNORE_VY +
                    mavros_msgs::PositionTarget::IGNORE_VZ +
                    // mavros_msgs::PositionTarget::IGNORE_AFX +
                    // mavros_msgs::PositionTarget::IGNORE_AFY +
                    // mavros_msgs::PositionTarget::IGNORE_AFZ +
                    // mavros_msgs::PositionTarget::FORCE +
                    // mavros_msgs::PositionTarget::IGNORE_YAW;
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    

    ros::Rate loop_rate(100);
    ros::Subscriber sub = n.subscribe("/gazebo/link_states", 100, link_0PoseCallback);
    ros::Subscriber pos_sub = n.subscribe("/mavros/local_position/pose", 100, posCallback);
    ros::Subscriber vel_sub = n.subscribe("/mavros/local_position/velocity_local", 100, velCallback);
    ros::Subscriber flag_sub = n.subscribe("/cyb_flags", 100, flagCallback);
    dynamic_reconfigure::Server<load_track::load_trackConfig> server;
    dynamic_reconfigure::Server<load_track::load_trackConfig>::CallbackType f;
    f = boost::bind(&ReconfigCallback,_1,_2);
    server.setCallback(f);

    //compute e_x and e_v

    //preset trajectory 
    //compute A in paper
    //track trajectory

    const Eigen::Vector3d a_i_max {0.5, 0.5, 5};
    const double max_fb_acc_ = 9.0;
    const Eigen::Vector3d g_ {0, 0, -9.8};
    Eigen::Vector3d Ki_ {0.1, 0.1, 0.5};

    ros::Time start_time = ros::Time::now();

    while(ros::ok())
    {
    ros::Time curr_time = ros::Time::now();
    double time = (curr_time-start_time).toSec();


    xd = getPosition(time);
    vd = getVelocity(time);
    ad = getAcceleration(time);

    ex = xd - xl;   //use UAV position or load position
    // ex = xd - x;
    // ex.x() = 1 - xl.x();
    // ex.y() = 1 - xl.y(); 
    // ex.z() = 4 - xl.z();   
    
    // ex.x() = 1 - x.x();
    // ex.y() = 1 - x.y(); 
    // ex.z() = 4 - x.z();   
                            
    // vd =  1*ex;
    ev = vd -vl;
        // ev = vd - v;
    // ev.x() = vd.x() - vl.x();
    // ev.y() = vd.y() - vl.y();
    // ev.z() = vd.z() - vl.z();


    // ev.x() = vd.x() - v.x();
    // ev.y() = vd.y() - v.y();
    // ev.z() = vd.z() - v.z();

    Eigen::Vector3d Kpv_ = {0,0,0};
    Eigen::Vector3d e_i;
    Kpv_[0] = Kpos_x_/Kvel_x_;
    Kpv_[1] = Kpos_z_/Kvel_y_;
    Kpv_[2] = Kpos_z_/Kvel_z_;
    Eigen::Vector3d Kpos_{0,0,0};
    Kpos_ << Kpos_x_, Kpos_y_, Kpos_z_;
    Eigen::Vector3d Kvel_{0,0,0};
    Kvel_ << Kvel_x_, Kvel_y_, Kvel_z_;
    Ki_<<Ki_x_, Ki_y_, Ki_z_;

    dot_q = omega_l.cross(q);

    if(x.z()>=0.5){
      e_i += (Kpv_.asDiagonal() * ex - ev)*0.01;
    }
    Eigen::Vector3d a_i = Ki_.asDiagonal() * e_i;
    if (a_i[0] >= a_i_max[0]){
       a_i[0] = a_i_max[0];
    }
   else if (a_i[1] >= a_i_max[1]){
      a_i[1] = a_i_max[1];
   }
   else if (a_i[2] >= a_i_max[2]){
      a_i[2] = a_i_max[2];
    }
   else if (a_i[0] <= -a_i_max[0]){
      a_i[0] = -a_i_max[0];
   }
    else if (a_i[1] <= -a_i_max[1]){
       a_i[1] = -a_i_max[1];
   }
    else if (a_i[2] <= -a_i_max[2]){
      a_i[2] = -a_i_max[2];
   }

  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * ex + Kvel_.asDiagonal() * ev;  // feedforward term for trajectory error
  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  const Eigen:: Vector3d a_des = a_fb + ad + a_i - g_ + l*(dot_q.dot(dot_q))*q; 
    // const Eigen:: Vector3d a_des = a_fb + ad + a_i - g_; 
  // msg.acceleration_or_force.x = 0.01 * a_des.x();
  // msg.acceleration_or_force.y = 0.01 * a_des.y();
  // msg.acceleration_or_force.z = 0.05 * a_des.z(); 
  // msg.yaw = 0; 

  // setrawPub.publish(msg);

    // A = -1*kx.asDiagonal()*ex-kv.asDiagonal()*ev+(mq+ml)*(ad+e3*g)+mq*l*(dot_q.dot(dot_q))*q;//equation (37) in paper,what is the meaning of q and \dot(q)?
    // A.x() = Kpos_x_*ex.x()+Kvel_x_*ev.x()+(mq+ml)*ad.x();   //8 1.5
    // A.y() = Kpos_y_*ex.y()+Kvel_y_*ev.y()+(mq+ml)*ad.y();   //8 1.5
    // A.z() = Kpos_z_*ex.z()+Kvel_z_*ev.z()+(mq+ml)*(ad.z()+9.8);
    // A = 1.5*ev+(mq+ml)*(ad+e3*g);
    // A.z() = 3.3*ev.z()+(mq+ml)*9.8;
    // A.z() = 10*ex.z()+3.3*ev.z()+5;
    // q_last = qd;

    // A = 0.01 * a_des;
    // A.z() = 0.05 * a_des.z();

    // qd = 0.01*a_des;
    // qd.z() = 0.05*a_des.z();         //normlized
    qd = a_des/a_des.norm();//修改记录6月13日改为对范数归一化

    // qd = - A/A.norm();
    
    // qdotdlast = q_dotd;

    // q_dotd = 100*(qd - q_last); //  \dot{q}_d

    q_dotd = omega_d.cross(qd);

    // qdotdd = 100*(q_dotd - qdotdlast);  // \ddot{q}_d   can't use diffuse

    // Fn = (qd.dot(q))*q;

    Fn = 0.008*a_des.dot(q)*q;  //6.13改为（38）式在论文中
    Fn.z() = 0.05 *(a_des.dot(q)*q).z();


    ROS_INFO("xl.x[%f]",q.x());

    ROS_INFO("xl.y[%f]",q.y());

    ROS_INFO("xl.z[%f]",q.z());

    // Fn = (A.dot(q))*q;

    eq = hat(q)*hat(q)*qd;
    Eigen::Vector3d temp = qd.cross(omega_d.cross(qd));
    edotq = dot_q - temp.cross(q);   //from the paper

    Fpd = - kq * eq - k_omega * edotq;

    Eigen::Vector3d temp1 = qd.cross(q_dotd);//    the temp of q_d cross \dot{q}_d;

    // Fff = mq * l * q.dot(temp1) * q.cross(dot_q);//    < > stand for inner product
    Fff = l * q.dot(temp1) * q.cross(dot_q);
    F = Fn + Fpd - Fff;

    msg.acceleration_or_force.x = F.x();
    msg.acceleration_or_force.y = F.y();
    msg.acceleration_or_force.z = F.z(); 
    msg.yaw = 0; 

  setrawPub.publish(msg);
                                                                                                                                                                                                 

    // F = Fn - Fpd;


    // mav_msgs::RollPitchYawrateThrust msg;
    // msg.header.stamp = ros::Time::now();
    // msg.header.frame_id = "map";
    // msg.roll = 0.01; //姿态角还是力？
    // msg.pitch = 0.01;
    // msg.yaw_rate = 0;
    // msg.thrust.x = qd.x();
    // msg.thrust.y = qd.y();
    // msg.thrust.z = qd.z();
    // forcePub.publish(msg);
    // ROS_INFO("force published.");

    // geometry_msgs::Twist msg;
    // msg.linear.x = vd.x();
    // msg.linear.y = vd.y();
    // msg.linear.z = vd.z();
    // msg.angular.x = 0;
    // msg.angular.y = 0;
    // msg.angular.z = 0;

    // geometry_msgs::Twist msg;
    // msg.linear.x = -0.2*ex.x();
    // msg.linear.y = -0.2*ex.y();
    // msg.linear.z = -0.4*ex.z();
    // msg.angular.x = 0;
    // msg.angular.y = 0;
    // msg.angular.z = 0;
    // velPub.publish(msg);


    // geometry_msgs::Vector3Stamped msg;
    // msg.header.frame_id = "map";
    // msg.header.stamp = ros::Time::now();
    // msg.vector.x = 0.05*qd(0);
    // msg.vector.y = 0.05*qd(1);
    // msg.vector.z = qd(2);
    // accelPub.publish(msg);




    ros::spinOnce();

    loop_rate.sleep();

    }



    

    return 0;

}