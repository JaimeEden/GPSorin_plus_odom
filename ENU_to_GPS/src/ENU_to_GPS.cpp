#include "ENU_to_GPS/ENU_to_GPS.hpp"

NavFixSatConverter::NavFixSatConverter() : nh("~") , first_frame_received(false)
{
    // 订阅NavFixSat话题
    sub = nh.subscribe("/gps", 10, &NavFixSatConverter::navFixSatCallback, this);
    sub_odom = nh.subscribe("/Odometry", 10, &NavFixSatConverter::odom_callback, this);
    sub_orien = nh.subscribe("/FligtControler_info", 10, &NavFixSatConverter::orien_callback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/v_angle_pub",10);

    // 发布ENU坐标
    //enu_pub = nh.advertise<nav_msgs::Odometry>("enu_position", 10);
}

void NavFixSatConverter::odom_callback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{

    // if(!first_odom)
    // {
    //     first_odom_time = odomMsg->header.stamp.toSec();
    //     first_odom = true;
    // }

    // if((!first_odom)&&(!first_angle))  return;
    // else
    // {
    //     duration = first_angle_time - first_odom_time;
    // }

    //if(targetTime.toSec() - (odomMsg->header.stamp.toSec() + duration) < 0.2 || (odomMsg->header.stamp.toSec() + duration) - targetTime.toSec() < 0.2)
    if(targetTime.toSec() - odomMsg->header.stamp.toSec() < 0.2 || odomMsg->header.stamp.toSec() - targetTime.toSec() < 0.2)
    {
        double roll_0,pitch_0,yaw_0;
        double yaw_updated;
        geometry_msgs::Quaternion orientation = odomMsg->pose.pose.orientation;
        tf2::Quaternion tf_orientation;
        tf2::fromMsg(orientation,tf_orientation);
        tf2::Matrix3x3(tf_orientation).getRPY(roll_0,pitch_0,yaw_0);

        pitch = -pitch_0-30/180*3.14159265359;
        roll = -roll_0;
        yaw_updated = yaw - yaw_0;  //yaw为北向夹角，yaw——0为偏航。

        trans_pub(roll,pitch,yaw_updated,odomMsg->pose.pose.position.x,odomMsg->pose.pose.position.y,odomMsg->pose.pose.position.z);

        ENU_TO_WGS(latitude,longitude,altitude,first_frame_latitude,first_frame_longitude,first_frame_altitude,ans_x,ans_y,ans_z);

        double dt = odomMsg->header.stamp.toSec() - pre_t;

        v_x = (ans_x - pre_x)/dt;
        v_y = (ans_y - pre_y)/dt;
        v_z = (ans_z - pre_z)/dt;

        // delta_roll = (roll - pre_roll)/dt;
        // delta_pitch = (pitch - pre_pitch)/dt;
        // delta_yaw = (yaw - pre_yaw)/dt;

        pre_t = odomMsg->header.stamp.toSec();
        pre_x = ans_x;
        pre_y = ans_y;
        pre_z = ans_z;
        // pre_roll = roll;
        // pre_pitch = pitch;
        // pre_yaw = yaw;

        nav_msgs::Odometry v_angle;
        v_angle.header = odomMsg->header;
        v_angle.pose.pose.position.x = latitude;
        v_angle.pose.pose.position.y = longitude;
        v_angle.pose.pose.position.z = altitude;
        v_angle.twist.twist.linear.x = v_x;
        v_angle.twist.twist.linear.y = v_y;
        v_angle.twist.twist.linear.z = v_z;
        v_angle.twist.twist.angular.x = roll_0;
        v_angle.twist.twist.angular.y = pitch_0;
        v_angle.twist.twist.angular.z = yaw_updated;

        

        // 发布enu坐标与速度角速度
        odom_pub.publish(v_angle);
        ros::spinOnce();

        return;


    }

}

void NavFixSatConverter::trans_pub(double roll, double pitch, double yaw, double x, double y, double z)
{

    ans_x = cos(pitch)*cos(yaw)*x + cos(pitch)*sin(yaw)*y - sin(pitch)*z;
    ans_y = (sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw))*x + (cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw))*y + sin(roll)*cos(pitch)*z;
    ans_z = (sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw))*x + (cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw))*y + cos(roll)*cos(pitch)*z;

}

void NavFixSatConverter::navFixSatCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    if (!first_frame_received) {
        // 存储经纬高信息
        first_frame_latitude = msg->latitude;
        first_frame_longitude = msg->longitude;
        first_frame_altitude = msg->altitude;

        // 设置标志表示第一帧已经接收
        first_frame_received = true;

    }
}

void NavFixSatConverter::orien_callback(const flc_interface::fcc::ConstPtr& orienMsg)
{
    auto time_utc = gps_to_date(orienMsg->gps_cycle.data, orienMsg->tic_toc.data);
    uint32_t seconds = time_utc.total_seconds();
    ros::Time finalTime(seconds-18, uint32_t(orienMsg->tic_toc.data % 1000)*1e6);

    //时间戳对齐
    // if(!first_angle)
    // {
    //     first_angle_time = finalTime.toSec();
    //     first_angle = true;
    // }

    targetTime = finalTime;
    
    double pi = 3.14159265359;

    double orin_deg = orienMsg->orientation_uav.data;

    yaw = orin_deg/180*pi;

    // if(orin_deg >= 180)
    // {
    //     yaw = (orin_deg - 180)/180*pi; 
    // }
    // else
    // {
    //     yaw = (orin_deg + 180)/180*pi;
    // }
}

boost::posix_time::time_duration NavFixSatConverter::gps_to_date(int gps_week, int gps_wins) 
{
    boost::posix_time::ptime gpsEpochStart(boost::gregorian::date(1980, 1, 6));
    boost::posix_time::ptime gpsTimePoint = gpsEpochStart + boost::posix_time::seconds(int(gps_week * 7 * 24 * 3600 + gps_wins*0.001)) ;
    boost::posix_time::time_duration duration = gpsTimePoint - boost::posix_time::from_time_t(0);
    
    return duration;
}

void NavFixSatConverter::ENU_TO_WGS(double &lat, double &lon, double &h, double lat0, double lon0, double h0, double xEast,double yNorth,double zUp )
{
    double a, b, f, e_sq, pi;
    pi = 3.14159265359;
    a = 6378137;
    b = 6356752.3142;
    f = (a - b) / a;
    e_sq = f * (2-f);
    pi = 3.14159265359;
    double lamb,phi,s,N,sin_lambda,cos_lambda,sin_phi,cos_phi; 
    lamb = pi/180*(lat0);
    phi = pi/180*(lon0);
    s =  sin(lamb);
    N = a /  sqrt(1 - e_sq * s * s);

    sin_lambda =  sin(lamb);
    cos_lambda =  cos(lamb);
    sin_phi =  sin(phi);
    cos_phi =  cos(phi);

    double x0,y0,z0,t,zd,xd,yd,x,y,z;
    x0 = (h0 + N) * cos_lambda * cos_phi;
    y0 = (h0 + N) * cos_lambda * sin_phi;
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

    t = cos_lambda * zUp - sin_lambda * yNorth;

    zd = sin_lambda * zUp + cos_lambda * yNorth;
    xd = cos_phi * t - sin_phi * xEast ;
    yd = sin_phi * t + cos_phi * xEast;

    x = xd + x0 ;
    y = yd + y0 ;
    z = zd + z0 ;


    double x2,y2,z2;
    x2 = x*x;
    y2 = y*y;
    z2 = z*z;

    double e,b2,e2,ep,r,r2,E2,F,G,c,s2,P,Q,ro,tmp,U,V,zo,height,temp;
    e =  sqrt (1-(b/a)*(b/a)); 
    b2 = b*b ;
    e2 = e *e; 
    ep = e*(a/b); 
    r =  sqrt(x2+y2); 
    r2 = r*r ;
    E2 = a*a- b*b; 
    F = 54*b2*z2 ;
    G = r2 + (1-e2)*z2 - e2*E2 ;
    c = (e2*e2*F*r2)/(G*G*G) ;
    s2 = pow(( 1 + c +  sqrt(c*c + 2*c) ),(1/3)) ;
    P = F / (3 * (s2+1/s2+1)*(s2+1/s2+1)* G*G) ;
    Q =  sqrt(1+2*e2*e2*P) ;
    ro = -(P*e2*r)/(1+Q) +  sqrt((a*a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2); 
    tmp = (r - e2*ro)*(r - e2*ro);
    U =  sqrt( tmp + z2 ) ;
    V =  sqrt( tmp + (1-e2)*z2 ) ;
    zo = (b2*z)/(a*V) ;

    height = U*( 1 - b2/(a*V) ); 
    
    lat =  atan( (z + ep*ep*zo)/r ) ;

    temp =  atan(y/x) ;

    double longitude;
    if (x >=0  )   
        longitude = temp ;
    else{
        if ((x < 0) && (y >= 0))
            longitude = pi + temp ;
        else 
            longitude = temp - pi ;
    }

    lat = lat/(pi/180) ;
    lon = longitude/(pi/180) ;
    h = height ;

}