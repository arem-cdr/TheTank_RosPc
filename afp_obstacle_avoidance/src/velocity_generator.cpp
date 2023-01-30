#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

//sensor_msgs::LaserScan lidarscan;
geometry_msgs::Twist receivedFromUser;
geometry_msgs::Twist finalVelocityCommand;
ros::Publisher published_velocity;

float force_x;
float force_y;

void laserscan_cb(const sensor_msgs::LaserScan::ConstPtr & msgLidar)
{
    std::vector<float> lidarArray = msgLidar.get()->ranges;
    float increment;
    increment = msgLidar.get()->angle_increment;
    int count = 0;
    for( int i = 0; i< lidarArray.size(); i++)
    {

        if(lidarArray[i] <= 0.03 && lidarArray[i] > 0.015) 
        {
            count++;
            force_x+=1/(lidarArray[i]*cos(-i*increment)*30.0);
            force_y+=1/(lidarArray[i]*sin(-i*increment)*30.0);
        }
        
    }
    force_x/=count;
    force_y/=count;
    bool sgnx = (force_x+receivedFromUser.linear.x > 0);
    bool sgny = (force_y+receivedFromUser.linear.y > 0);
    finalVelocityCommand.linear.x = sqrt(pow(force_x+receivedFromUser.linear.x,2));
    finalVelocityCommand.linear.x *= (1.0)*sgnx -(1.0)*(!sgnx);
    finalVelocityCommand.linear.y = sqrt(pow(force_y+receivedFromUser.linear.y,2));
    finalVelocityCommand.linear.y *= (1.0)*sgny -(1.0)*(!sgny);
    published_velocity.publish(finalVelocityCommand);
}

void velocity_from_user_cb(const geometry_msgs::Twist::ConstPtr & msgVel)
{
    receivedFromUser = *(msgVel.get());
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "velocity_generator");
    ros::NodeHandle n;
    n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    n.subscribe("teleop/cmd_vel",100,velocity_from_user_cb);
    n.subscribe("sensor_msgs/LaserScan",100,laserscan_cb);
    ros::spin();
    return EXIT_SUCCESS;
}
