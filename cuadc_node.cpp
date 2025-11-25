#include <ros/ros.h>
#include <cuadc/cuadc.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cuadc");
    ros::NodeHandle nh("~");
    CUADC::CUADCConfig config;
    config.getParamters(nh);
    ros::NodeHandle nh_;
    CUADC::CUADC_MultiCopter copter(nh_, config);
    ros::spin();
    return 0;
}
