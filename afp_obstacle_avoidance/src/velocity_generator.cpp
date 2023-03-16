#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <afp_obstacle_avoidance/paramConfig.h>

typedef enum _msg_type
{
    force_dampener_x,
    force_dampener_y,
    addinv_x,
    addinv_y,
    alpha_x,
    alpha_y,
    rinf,
    rsup,
    avoidance_bool
}msg_type;

//sensor_msgs::LaserScan lidarscan;
geometry_msgs::Twist receivedFromUser;
geometry_msgs::Twist receivedFromUser_copy;
geometry_msgs::Twist finalVelocityCommand;
geometry_msgs::TwistStamped finalVelocityCommand_stamped;
ros::Publisher published_velocity;
ros::Publisher published_velocity_stamped;
ros::Subscriber scan_sub;
ros::Subscriber teleop_sub;
std::string toprint;
char toprintnumber[50];

ros::NodeHandle* n_glob;

float fdx = 10.0;
float fdy = 10.0;
float inv_x = 1.0;
float inv_y = 1.0;
int alph_x = 1;
int alph_y = 1;
float limit_r_down = 0.15;
float limit_r_up = 0.4;

float force_x = 0.0;
float force_y = 0.0;

bool avoidance = false;

void callback(afp_obstacle_avoidance::paramConfig &config, uint32_t level)
{
    switch (level)
    {
        case msg_type::force_dampener_x:
            fdx = config.force_dampener_x;
            break;
        case msg_type::force_dampener_y:
            fdy = config.force_dampener_y;
            break;
        case msg_type::addinv_x:
            inv_x = config.addinv_x;
            break;
        case msg_type::addinv_y:
            inv_y = config.addinv_y;
            break;
        case msg_type::alpha_x:
            alph_x = config.alpha_x;
            break;
        case msg_type::alpha_y:
            alph_y = config.alpha_y;
            break;
        case msg_type::rinf:
            limit_r_down  = config.rinf;
            break;
        case msg_type::rsup:
            limit_r_up = config.rsup;
            break;
        case msg_type::avoidance_bool:
            avoidance = config.enable_avoidance;
            break;
        default:
            break;
    }


}

void laserscan_cb(const sensor_msgs::LaserScan::ConstPtr & msgLidar)
{
    
    receivedFromUser_copy = receivedFromUser;
    std::vector<float> lidarArray = msgLidar.get()->ranges;
    float increment;
    increment = msgLidar.get()->angle_increment;
    int count = 0;
    force_x = 0.0;
    force_y = 0.0;
    for( int i = 0; i< lidarArray.size(); i++)
    {
        
        if(lidarArray[i] < limit_r_up && lidarArray[i] > limit_r_down) 
        {
            sprintf(toprintnumber,"%f ",i*increment);
            toprint+=toprintnumber;
            count++;
            force_x+=cos(i*increment)/(pow(lidarArray[i],alph_x)*fdx+inv_x);
            force_y+=sin(i*increment)/(pow(lidarArray[i],alph_y)*fdy+inv_y);
        }
        
        
    }

    if(count > 0)
    {
        force_x/=-count;
        force_y/=-count;
        toprint+="\n";
        toprint+="\n";
        ROS_INFO("%s",toprint.c_str());   
    }
    if(count == 1 || avoidance == false)
    {
        force_x = 0;
        force_y = 0;
    }

    bool sgnx = (force_x+receivedFromUser_copy.linear.x > 0);
    bool sgny = (force_y+receivedFromUser_copy.linear.y > 0);
    finalVelocityCommand.linear.x = sqrt(pow(force_x+receivedFromUser_copy.linear.x,2));
    finalVelocityCommand.linear.x *= (1.0)*sgnx -(1.0)*(!sgnx);
    finalVelocityCommand.linear.y = sqrt(pow(force_y+receivedFromUser_copy.linear.y,2));
    finalVelocityCommand.linear.y *= (1.0)*sgny -(1.0)*(!sgny);
    finalVelocityCommand.angular.z = receivedFromUser_copy.angular.z;
    published_velocity.publish(finalVelocityCommand);
    finalVelocityCommand_stamped.header.stamp = ros::Time::now();
    finalVelocityCommand_stamped.twist = finalVelocityCommand;
    published_velocity_stamped.publish(finalVelocityCommand_stamped);
}

void velocity_from_user_cb(const geometry_msgs::Twist::ConstPtr & msgVel)
{
    receivedFromUser = *(msgVel.get());
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "velocity_generator");
    ros::NodeHandle n;
    n_glob = &n;

    finalVelocityCommand.angular.z = 0.0;
    finalVelocityCommand.linear.x = 0.0;
    finalVelocityCommand.angular.y = 0.0;
    receivedFromUser.angular.z = 0.0;
    receivedFromUser.linear.x = 0.0;
    receivedFromUser.linear.y = 0.0;
    
    n.getParam("/velocity_generator/addinv_x",inv_x);
    n.getParam("/velocity_generator/addinv_y",inv_y);
    n.getParam("/velocity_generator/alpha_y",alph_x);
    n.getParam("/velocity_generator/force_dampener_x",fdx);
    n.getParam("/velocity_generator/force_dampener_y",fdy);
    n.getParam("/velocity_generator/alpha_x",alph_y);
    n.getParam("velocity_generator/rinf",limit_r_down);
    n.getParam("velocity_generator/rsup",limit_r_up);
    n.getParam("velocity_generator/enable_avoidance",avoidance);

    finalVelocityCommand_stamped.header.frame_id = "base_link";

    
    ROS_INFO("Entered Main");
    published_velocity = n.advertise<geometry_msgs::Twist>("/cmd_vel",100);
    published_velocity_stamped = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel_stamped",100);
    
    ROS_INFO("advertised /cmd_vel");
    teleop_sub = n.subscribe("/cmd_vel_teleop",100,velocity_from_user_cb);
    ROS_INFO("subbed to cmd_vel_teleop");
    scan_sub = n.subscribe("/scan",100,laserscan_cb);
    ROS_INFO("subbed to scan");

    dynamic_reconfigure::Server<afp_obstacle_avoidance::paramConfig> server;
    dynamic_reconfigure::Server<afp_obstacle_avoidance::paramConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return EXIT_SUCCESS;
}
