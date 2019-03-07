#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <math.h>
//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace cv;
using namespace aruco;
using namespace Eigen;

double rmsT = 0;
double rmsR = 0;
double counter = 0;
//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat K, D;
// test function, can be used to verify your estimation
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
               pts_3[i].x, pts_3[i].y, pts_3[i].z,
               un_pts_2[i].x, un_pts_2[i].y,
               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
    }
    puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);

    // version 2, your work
    Matrix3d R;
    Vector3d T;
    R.setIdentity();
    T.setZero();
    ROS_INFO("write your code here!");
    //...
    vector<cv::Point2f> udpts_2;
    cv::undistortPoints(pts_2, udpts_2, K, D);
    // Matrix<double, 2*pts_3.length(), 9> A;
    MatrixXd A(2*pts_3.size(), 9);

    for (unsigned int dots = 0; dots < pts_3.size(); dots++)
    {
      A(dots*2, 0) = pts_3[dots].x;
      A(dots*2, 1) = pts_3[dots].y;
      A(dots*2, 2) = 1;
      A(dots*2, 3) = 0;
      A(dots*2, 4) = 0;
      A(dots*2, 5) = 0;
      A(dots*2, 6) = -pts_3[dots].x * udpts_2[dots].x;
      A(dots*2, 7) = -pts_3[dots].y * udpts_2[dots].x;
      A(dots*2, 8) = -udpts_2[dots].x;

      A(dots*2 + 1, 0) = 0;
      A(dots*2 + 1, 1) = 0;
      A(dots*2 + 1, 2) = 0;
      A(dots*2 + 1, 3) = pts_3[dots].x;
      A(dots*2 + 1, 4) = pts_3[dots].y;
      A(dots*2 + 1, 5) = 1;
      A(dots*2 + 1, 6) = -pts_3[dots].x * udpts_2[dots].y;
      A(dots*2 + 1, 7) = -pts_3[dots].y * udpts_2[dots].y;
      A(dots*2 + 1, 8) = -udpts_2[dots].y;
    }

    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

    MatrixXd V(9,9);
    V = svd.matrixV();

    Vector3d h1e;
    Vector3d h2e;
    Vector3d h3e;

    Matrix3d H;

    H(0,0) = V(0,8);
    H(0,1) = V(1,8);
    H(0,2) = V(2,8);
    H(1,0) = V(3,8);
    H(1,1) = V(4,8);
    H(1,2) = V(5,8);
    H(2,0) = V(6,8);
    H(2,1) = V(7,8);
    H(2,2) = V(8,8);

    // cout << "H= " << H << endl;

    // Matrix3d KE;
    // for(int i=0; i<3; i++)
    // {
    //   for(int j=0; j<3; j++)
    //   {
    //     KE(i,j) = K.at<double>(i,j);
    //   }
    // }
    //
    // Matrix3d Hh = KE.inverse() * H;


    h1e = H.col(0);
    h2e = H.col(1);
    h3e = H.col(2);

    Matrix3d HF;
    HF << h1e,h2e,h1e.cross(h2e);

    JacobiSVD<MatrixXd> svd2(HF, ComputeThinU | ComputeThinV);

    // cout << "he1= " << h1e << endl;
    // cout << "he3= " << h3e << endl;

    R = svd2.matrixU() * svd2.matrixV().transpose();
    // cout << "R= " << R << endl;
    T = h3e / h1e.norm();
    //  cout << "T= " << T ;
    //...
    //...
    if (T(2) < 0)
    {
      T = -T;
      R.col(0) = -R.col(0);
      R.col(1) = -R.col(1);
    }

    counter = counter + 1;
    double refx, refy, refz;
    refx = t.at<double>(0,0);
    refy = t.at<double>(1,0);
    refz = t.at<double>(2,0);




    rmsT = sqrt ((pow(rmsT,2)*(counter-1) + pow(T(0) - refx, 2) + pow(T(1) - refy, 2) + pow(T(2) - refz, 2)) / counter);

    rmsR = sqrt ((pow(rmsR,2)*(counter-1) + pow(R(0,0) - R_ref(0,0), 2) + pow(R(0,1) - R_ref(0,1), 2) + pow(R(0,2) - R_ref(0,2), 2)
                                          + pow(R(1,0) - R_ref(1,0), 2) + pow(R(1,1) - R_ref(1,1), 2) + pow(R(1,2) - R_ref(1,2), 2)
                                          + pow(R(2,0) - R_ref(2,0), 2) + pow(R(2,1) - R_ref(2,1), 2) + pow(R(2,2) - R_ref(2,2), 2)) / (3*counter)); // 3 directions so 3 * counter in denomenator

    cout << "rmsT = " << rmsT << endl;
    cout << "rmsR = " << rmsR << endl;

    Quaterniond Q_yourwork;
    Q_yourwork = R;
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "world";
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    pub_odom_yourwork.publish(odom_yourwork);
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);

    cv::imshow("in", bridge_ptr->image);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;



    //init window for visualization
    cv::namedWindow("in", 1);

    ros::spin();
}
