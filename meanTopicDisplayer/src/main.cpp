#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>

ros::Subscriber sub;
ros::Publisher pub;
std_msgs::Float32MultiArray msg_float;

void mean_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    static int i = 0;
    static float gyroZ = 0.0;
    static float accX = 0.0;
    static float accY = 0.0;
    if( i >= 100)
    {
        i = 0;
        msg_float.data[0] = accX;
        msg_float.data[1] = accY;
        msg_float.data[2] = gyroZ;
        accX = 0.0;
        accY = 0.0;
        gyroZ = 0.0;
        pub.publish(msg_float);
    }
    else
    {
        i+=1;
        accX += (msg.get()->linear_acceleration.x)/100.0;
        accY += (msg.get()->linear_acceleration.y)/100.0;
        gyroZ += (msg.get()->angular_velocity.z)/100.0;
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "mean_displayer");
    ros::NodeHandle n;
    

    msg_float.data.push_back(0.0);
    msg_float.data.push_back(0.0);
    msg_float.data.push_back(0.0);

    sub = n.subscribe("sensor_msgs/Imu",150,mean_callback);
    pub = n.advertise<std_msgs::Float32MultiArray>("mean_IMU",10);
    ros::spin();
    return 0;
}
