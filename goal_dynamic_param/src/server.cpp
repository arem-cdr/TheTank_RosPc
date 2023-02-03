#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <goal_dynamic_param/paramConfig.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>
#include <tf/tf.h>

ros::NodeHandle* nh_glob;
ros::Publisher* pub_glob;


ros::Subscriber global_odo_sub;
nav_msgs::Odometry odo_glob;
double theta_odo;
bool done = true;
bool odo_received = false;
bool first_odo_received = false;

float precision = 0.005;
float precision_angle= (M_PI*0.5)/180.0;

float last_error_x = 0.0;
float sum_error_x= 0.0;

float last_error_y= 0.0;
float sum_error_y= 0.0;

float last_error_theta= 0.0;
float sum_error_theta= 0.0;

float Kp_dist = 1.0;
float Kd_dist = 0.0;
float Ki_dist = 0.0;

float Kp_theta = 1.0;
float Kd_theta = 0.0;
float Ki_theta = 0.0;

ros::Time odo_time_last;
ros::Time odo_time;
ros::Duration ellapsed;

geometry_msgs::Twist tosend;

typedef enum _msg_type
{
  goal_x,
  goal_y,
  goal_theta,
  go,
  nearest_plus_pi_2,
  nearest_minus_pi_2,
  go_1m_fwd_now,
  go_1m_bwd_now,
  go_1m_side_left_now,
  go_1m_side_right_now,
  msg_precision,
  msg_precision_deg,
  msg_Kp_dist,
  msg_Ki_dist,
  msg_Kd_dist,
  msg_Kp_angle,
  msg_Ki_angle,
  msg_Kd_angle,

}msg_type;

typedef struct _goal
{
  float x;
  float y;
  float theta;
}goal;

goal mygoal {0.0,0.0,0.0};
bool go_global = false;

float keepAngle(float angle_rad)
{
  return atan2(sin(angle_rad),cos(angle_rad));
}

float nearestPI_2Mult(float angle_rad)
{
  angle_rad = keepAngle(angle_rad);
  if(fabs(angle_rad - M_PI_2) < M_PI_4 )
  {
    return M_PI_2;
  }
  else if(fabs(angle_rad -M_PI) < M_PI_4 )
  {
    return M_PI;
  }
  else if(fabs(angle_rad + M_PI_2) < M_PI_4 )
  {
    return -M_PI_2;
  }
  else
  {
    return -M_PI;
  }
}

void callback(goal_dynamic_param::paramConfig &config, uint32_t level) {
   

  
  switch (level)
  {
  case msg_type::goal_x:
    /* code */
    mygoal.x = config.goal_x_m;
    break;
  case msg_type::goal_y:
    /* code */
    mygoal.y = config.goal_y_m;
    break;
  case msg_type::goal_theta:
    mygoal.theta = keepAngle(M_PI*config.goal_theta_deg/180.0);
    break;
  case msg_type::go:
    go_global = true;
    /* code */
    break;
  case msg_type::nearest_plus_pi_2:
    /* code */
    mygoal.x = odo_glob.pose.pose.position.x;
    mygoal.y = odo_glob.pose.pose.position.y;
    mygoal.theta = keepAngle(nearestPI_2Mult(theta_odo)+M_PI_2);
    go_global = true;
    break;

  case msg_type::nearest_minus_pi_2:
    mygoal.x = odo_glob.pose.pose.position.x;
    mygoal.y = odo_glob.pose.pose.position.y;
    mygoal.theta = keepAngle(nearestPI_2Mult(theta_odo)+M_PI_2);
    go_global = true;
    break;
  case go_1m_fwd_now:
      mygoal.x = odo_glob.pose.pose.position.x + 1.0;
      mygoal.y = odo_glob.pose.pose.position.y;
      mygoal.theta = keepAngle(theta_odo);
      go_global = true;
    break;
  case go_1m_bwd_now:
      mygoal.x = odo_glob.pose.pose.position.x - 1.0;
      mygoal.y = odo_glob.pose.pose.position.y;
      mygoal.theta = keepAngle(theta_odo);
      go_global = true;
    break;
  case go_1m_side_left_now:
      mygoal.x = odo_glob.pose.pose.position.x;
      mygoal.y = odo_glob.pose.pose.position.y + 1.0;
      mygoal.theta = keepAngle(theta_odo);
      go_global = true;
    break;
  case go_1m_side_right_now:
      mygoal.x = odo_glob.pose.pose.position.x;
      mygoal.y = odo_glob.pose.pose.position.y - 1.0;
      mygoal.theta = keepAngle(theta_odo);
      go_global = true;
    break;
  case msg_type::msg_precision:
      precision = config.precision_m;
    break;
  case msg_type::msg_precision_deg:
      precision_angle = (M_PI*config.precision_deg)/180.0;
    break;
  case msg_type::msg_Kp_dist:
      Kp_dist = config.Kp_dist;
    break;
  case msg_type::msg_Ki_dist:
      Ki_dist = config.Ki_dist;
    break;
  case msg_type::msg_Kd_dist:
      Kd_dist = config.Kd_dist;
    break;
  case msg_type::msg_Kp_angle:
      Kp_theta = config.Kp_angle;
    break;
  case msg_type::msg_Ki_angle:
      Ki_theta = config.Ki_angle;
    break;
  case msg_type::msg_Kd_angle:
      Kd_theta = config.Kd_angle;
    break;

      
  default:
    break;
  }



}

// void callback()
void odo_callback(const nav_msgs::Odometry::ConstPtr &odo)
{

  odo_time_last = odo_glob.header.stamp;
  odo_glob = (*odo.get());
  odo_time = odo_glob.header.stamp;
  if(first_odo_received == false)
  {
    first_odo_received = true;
    odo_time_last = ros::Time::now();
    odo_time = ros::Time::now();
    
  }
  ellapsed = odo_time - odo_time_last;
  tf::Quaternion q(
      odo->pose.pose.orientation.x,
      odo->pose.pose.orientation.y,
      odo->pose.pose.orientation.z,
      odo->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  theta_odo = keepAngle(yaw);
  

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "goal_dynamic_param");
  ros::NodeHandle nh;  
  nh_glob = &nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",20);
  pub_glob = &pub;
  dynamic_reconfigure::Server<goal_dynamic_param::paramConfig> server;
  dynamic_reconfigure::Server<goal_dynamic_param::paramConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  global_odo_sub = nh.subscribe("/nav_msgs/odo",1000,odo_callback);
  while(ros::ok())
  {
    if(odo_received)
    {
      odo_received = false;
      if(go_global == true || done == false)
      {
        go_global = false;
        done = false;


        float error_x = mygoal.x - odo_glob.pose.pose.position.x;
        float error_y = mygoal.y - odo_glob.pose.pose.position.y;
        float error_theta = keepAngle(mygoal.theta-theta_odo);

        if( fabs(error_x) > 30*precision || fabs(error_y) > 30*precision )
        {
          if(error_x > error_y)
          {
            error_y = (error_y*30*precision)/(fabs(error_x));
            error_x = (error_x*30*precision)/(fabs(error_x));
          }
          else
          {
            error_x = (error_x*30*precision)/(fabs(error_y));
            error_y = (error_y*30*precision)/(fabs(error_y));
          }
        }

        float dt = ellapsed.toSec();

        



        sum_error_theta += dt*error_theta;
        sum_error_x += dt*error_x;
        sum_error_y += dt*error_y;

        float diff_error_x = error_x - last_error_x;
        float diff_error_y = error_y - last_error_y;
        float diff_error_theta = error_theta - last_error_theta;

        tosend.linear.x = Kp_dist * error_x - Kd_dist*diff_error_x + Ki_dist*sum_error_x;
        tosend.linear.y = Kp_dist * error_y - Kd_dist*diff_error_y + Ki_dist*sum_error_y;
        tosend.angular.z = Kp_theta* error_theta - Kd_theta*diff_error_theta + Ki_theta*sum_error_theta;

        last_error_x = error_x;
        last_error_y = error_y;
        last_error_theta = error_theta;
        

        if(fabs(error_x) < precision 
        && fabs(error_y) < precision
        && fabs(error_theta) < precision_angle)
        {
          done = true;
          go_global = false;
          last_error_x = 0.0;
          last_error_y = 0.0;
          last_error_theta = 0.0;

        }
      }
    }
    ros::spinOnce();

  }
  
  

  

  
  ros::spin();
  return 0;
}
