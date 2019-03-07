#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
//outside forces
Vector3d gravity(0, 0, 9.81); // gravity, world g ppoint down is positive
//states
double t = 0;
VectorXd x = VectorXd::Zero(15); //state
MatrixXd cov = MatrixXd::Identity(15,15);//covariance matrix
bool upd = false;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
    double t_cur = msg->header.stamp.toSec();
    if (upd == false)
    {
      t = t_cur;
      upd = true;
      return;
    }
    double dt = t_cur - t;
    t = t_cur;

    //Eular Angle
    double phi = x(3);
    double theta = x(4);
    double psi = x(5);

    //acceleration
    Vector3d acceleration_i, acceleration;
    acceleration_i(0) = msg->linear_acceleration.x;
    acceleration_i(1) = msg->linear_acceleration.y;
    acceleration_i(2) = msg->linear_acceleration.z;
    acceleration(0) = acceleration_i(0) - x(12);
    acceleration(1) = acceleration_i(1) - x(13);
    acceleration(2) = acceleration_i(2) - x(14);

    //Angular Velocity
    Vector3d Omega_i, Omega;
    Omega_i(0) = msg->angular_velocity.x;
    Omega_i(1) = msg->angular_velocity.y;
    Omega_i(2) = msg->angular_velocity.z;
    Omega(0) = Omega_i(0) - x(9);
    Omega(1) = Omega_i(1) - x(10);
    Omega(2) = Omega_i(2) - x(11);

    //transfer matrices
    Matrix3d G;
    G <<  cos(theta), 0, -cos(phi)*sin(theta),
          0,          1,  sin(phi),
          sin(theta), 0,  cos(phi)*cos(theta);

    Matrix3d G_inv = G.inverse();
    Matrix3d R;
             R << cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
                  cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi),  sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi),
                 -cos(phi)*sin(theta),                              sin(phi),           cos(phi)*cos(theta);

	  Matrix3d df2_dx2;
            df2_dx2 << 0													   ,Omega(2)*cos(theta) - Omega(0)*sin(theta),							0,
						-(Omega(2)*cos(theta) - Omega(0)*sin(theta)) / pow(cos(phi), 2), (sin(phi)*(Omega(0)*cos(theta) + Omega(2)*sin(theta))) / cos(phi), 0,
						(sin(phi)*(Omega(2)*cos(theta) - Omega(0)*sin(theta))) / pow(cos(phi), 2), -(Omega(0)*cos(theta) + Omega(2)*sin(theta)) / cos(phi), 0;
	  Matrix3d df3_dx2;
            df3_dx2 << sin(psi)*(acceleration(1)*sin(phi) + acceleration(2)*cos(phi)*cos(theta) - acceleration(0)*cos(phi)*sin(theta)),	acceleration(2)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) - acceleration(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), -acceleration(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acceleration(2)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)) - acceleration(1)*cos(phi)*cos(psi),
						          -cos(psi)*(acceleration(1)*sin(phi) + acceleration(2)*cos(phi)*cos(theta) - acceleration(0)*cos(phi)*sin(theta)),	acceleration(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - acceleration(0)*(sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)), acceleration(0)*(cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)) + acceleration(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - acceleration(1)*cos(phi)*sin(psi),
						           acceleration(1)*cos(phi) - acceleration(2)*cos(theta)*sin(phi) + acceleration(0)*sin(phi)*sin(theta),				-cos(phi)*(acceleration(0)*cos(theta) + acceleration(2)*sin(theta)),																		   0;


    //At, Ut
    MatrixXd At = MatrixXd::Zero(15,15);
	At.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3); //dx3/dx
	At.block<3, 3>(3, 9) = -G_inv; //-dG(X2)^-1*X4/X4
	At.block<3, 3>(3, 3) = df2_dx2;//dG(X2)^-1(Wm-X4)/dX2
	At.block<3, 3>(6, 3) = df3_dx2;//dR(X2)(am-X2)/dX2
  At.block<3, 3>(6, 12) = -R; //dR(X2)(am-X5)/dX5


  MatrixXd Ut = MatrixXd::Zero(15,12);
	Ut.block<3, 3>(3, 0) = -G_inv;
  Ut.block<3, 3>(6, 3) = -R;
  Ut.block<6, 6>(9, 6) = MatrixXd::Identity(6,6);

  MatrixXd Ft = MatrixXd::Identity(15,15) + dt * At;
  MatrixXd Vt = dt * Ut;
  cov = Ft * cov * (Ft.transpose()) + Vt * Q * (Vt.transpose());
  VectorXd f = VectorXd::Zero(15);
  f.block<3, 1>(0,0) = x.block<3,1>(6,0);
  f.block<3, 1>(3,0) = G_inv * Omega;
  f.block<3, 1>(6,0) = gravity + R * acceleration;

  x = x + dt * f;
  nav_msgs::Odometry odom;
  MatrixXd Rwm = MatrixXd::Identity(3,3);

  Vector3d w_p = Rwm * x.block<3,1>(0,0);
  Vector3d w_v = Rwm * x.block<3,1>(6,0);

  odom.header.frame_id = "world";
  odom.header.stamp = msg->header.stamp;
  odom.pose.pose.position.x = w_p(0,0);
  odom.pose.pose.position.y = w_p(1,0);
  odom.pose.pose.position.z = w_p(2,0);

  odom.twist.twist.linear.x = w_v(0);
  odom.twist.twist.linear.y = w_v(1);
  odom.twist.twist.linear.z = w_v(2);

  double phit = x(3,0);
  double thetat = x(4,0);
  double psit = x(5,0);
  Matrix3d Rt;
  Rt << cos(psit)*cos(thetat)-sin(phit)*sin(psit)*sin(thetat), -cos(phit)*sin(psit), cos(psit)*sin(thetat)+cos(thetat)*sin(phit)*sin(psit),
        cos(thetat)*sin(psit)+cos(psit)*sin(phit)*sin(thetat),  cos(phit)*cos(psit), sin(psit)*sin(thetat)-cos(psit)*cos(thetat)*sin(phit),
       -cos(phit)*sin(thetat), sin(phit), cos(phit)*cos(thetat);

  Rt = Rwm * Rt;

  Quaterniond ori(Rt);

  odom.pose.pose.orientation.x = ori.x();
  odom.pose.pose.orientation.y = ori.y();
  odom.pose.pose.orientation.z = ori.z();
  odom.pose.pose.orientation.w = ori.w();

  odom_pub.publish(odom);

}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0.05, 0.05, 0)
    //camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							                 0, -1, 0,
    //                               0, 0, -1;
    Quaterniond r(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    Matrix3d Rcw = r.toRotationMatrix();
    Vector3d Tcw(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    Matrix3d R = Rcw.transpose() * Rcam.transpose();
    Vector3d Piw = -Rcw.transpose() * (Rcam.transpose() * Vector3d(0.05, 0.05, 0) + Tcw);

    double phi = asin(R(2,1));
    double psi = atan2(-R(0,1)/cos(phi),R(1,1)/cos(phi));
    double theta = atan2(-R(2,0)/cos(phi),R(2,2)/cos(phi));

    MatrixXd Ct = MatrixXd::Zero(6,15);
    Ct.block<3,3>(0,0) = MatrixXd::Identity(3,3);
    Ct.block<3,3>(3,3) = MatrixXd::Identity(3,3);

    Vector3d Omega(phi, theta, psi);

    MatrixXd Wt = MatrixXd::Identity(6,6);

    MatrixXd Kt = cov * (Ct.transpose()) * ((Ct * cov * (Ct.transpose()) + Wt * Rt * (Wt.transpose())).inverse());

    VectorXd IMUpos(6);
    IMUpos.block<3,1>(0,0) = Piw;
    IMUpos.block<3,1>(3,0) = Omega;

    VectorXd odom_diff = IMUpos - Ct * x;
    if(odom_diff(5) > M_PI)
    {
      odom_diff(5) = odom_diff(5) - 2*M_PI;
    }
    else
    if(odom_diff(5) < -M_PI)
    {
      odom_diff(5) = odom_diff(5) + M_PI*2;
    }

    if(odom_diff(4) > M_PI)
    {
      odom_diff(4) = odom_diff(4) - 2*M_PI;
    }
    else
    if(odom_diff(4) < -M_PI)
    {
      odom_diff(4) = odom_diff(4) + M_PI*2;
    }

    if(odom_diff(3) > M_PI)
    {
      odom_diff(3) = odom_diff(3) - 2*M_PI;
    }
    else if(odom_diff(3) < -M_PI)
    {
      odom_diff(3) = odom_diff(3) + M_PI*2;
    }

    x = x + Kt * odom_diff;
    cov = cov - Kt * Ct * cov;


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.1 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
// 　　　　　　　 ┏┓     ┏┓+ +
// 　　　　　　　┏┛┻━━━━━┛┻┓ + +
// 　　　　　　　┃　　　　  ┃ 　             NO COST TOO GREAT
// 　　　　　　　┃　　　━　 ┃ ++ + + +       NO MIND TO THINK
// 　　　　　　 ████━████  ┃+               NO WILL TO BREAK
// 　　　　　　　┃　　　　　┃ +              NO VOICE TO CRY SUFFERING
// 　　　　　　　┃　┻　　　 ┃                BORN OF GOD AND VOID
// 　　　　　　　┃　　　　  ┃ + +            YOU SHALL SEAL THE BLINDING LIGHT THAT PLAGUES THEIR DREAMS
// 　　　　　　　┗━┓　　　┏━┛                YOU ARE THE VESSEL
// 　　　　　　　  ┃　　 ┃　　　　　　　　　  YOU ARE THE HOLLOW KNIGHT
// 　　　　　　　  ┃　　 ┃ + + + +
// 　　　　　　　  ┃　　 ┃　　　　Code is far away from bug with the animal protecting　　　　　　　
// 　　　　　　　  ┃　　 ┃ + 　　　　神兽保佑,代码无bug　　
// 　　　　　　　  ┃　　 ┃
// 　　　　　　　  ┃　　 ┃　　+　　　　　　　　　
// 　　　　　　　  ┃　 　 ┗━━━━┓ + +
// 　　　　　　　  ┃ 　　　　　 ┣┓
// 　　　　　　　  ┃ 　　　　___┏┛
// 　　　　　　　  ┗┓┓┏━┳┓┏┛ + + + +
// 　　　　　　　   ┃┫┫ ┃┫┫
// 　　　　　　　   ┗┻┛ ┗┻┛+ + + +
