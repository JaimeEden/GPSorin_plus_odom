#ifndef ENU_TO_GPS
#define ENU_TO_GPS

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <flc_interface/fcc.h>
#include <tf2/utils.h>
#include <geometry_msgs/Quaternion.h>

class NavFixSatConverter {
public:
    NavFixSatConverter();

    boost::posix_time::time_duration gps_to_date(int gps_week, int gps_wins);

    void orien_callback(const flc_interface::fcc::ConstPtr& orienMsg);

    void navFixSatCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void trans_pub(double roll, double pitch, double yaw, double x, double y, double z);

    void odom_callback(const nav_msgs::Odometry::ConstPtr& odomMsg);

    void ENU_TO_WGS(double &lat, double &lon, double &h, double lat0, double lon0, double h0, double xEast,double yNorth,double zUp );


private:
    ros::NodeHandle nh;
    ros::Subscriber sub; //first gps sub
    ros::Subscriber sub_odom;
    ros::Subscriber sub_orien;
    ros::Publisher odom_pub;

    bool first_frame_received = false;
    double first_frame_latitude;
    double first_frame_longitude;
    double first_frame_altitude;

    double roll,pitch,yaw;  //rad
    double pre_roll = 0.0,pre_pitch = 0.0,pre_yaw = 0.0;
    double delta_roll,delta_pitch,delta_yaw;

    ros::Time targetTime;
    double first_angle_time;
    double first_odom_time;
    double duration;


    double ans_x;
    double ans_y;
    double ans_z;

    double pre_x = 0;
    double pre_y = 0;
    double pre_z = 0;
    double pre_t = 0;


    double v_x;
    double v_y;
    double v_z;

    double latitude;
    double longitude;
    double altitude;

    bool first_angle = false;
    bool first_odom = false;

};

#endif
