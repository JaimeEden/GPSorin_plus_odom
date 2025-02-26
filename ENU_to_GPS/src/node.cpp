#include "ENU_to_GPS/ENU_to_GPS.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_converter");

    NavFixSatConverter converter;

    ros::spin();

    return 0;
}
