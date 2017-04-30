#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <depth_measure/depth.h>

#include <iostream>  
#include <string>
#include <iostream>
#include <cmath>

class VIS
{
  //ros::Publisher marker_pub_vel;
  ros::Publisher marker_pub_fix;


  void synCallback(const sensor_msgs::NavSatFixConstPtr& gpsFix_input, const geometry_msgs::TwistStampedConstPtr& gpsVel_input)
  {
      
      sensor_msgs::NavSatFix gpsFix_msg = *gpsFix_input;
      geometry_msgs::TwistStamped gpsVel_msg = *gpsVel_input;
      

      visualization_msgs::Marker points;

      float latitude = gpsFix_msg.latitude;
      float longitude = -gpsFix_msg.longitude;
      double velX = gpsVel_msg.twist.linear.x;
      double velY = gpsVel_msg.twist.linear.y;
      std::cout<<velX<<"y"<<velY<<std::endl;
      // assuming y is the direction we're heading in. (points ot the front of the car)
      // double velToAdd = (pow((pow(velX,2)+pow(velY,2)),0.5))*velY/abs(velY); // in m/s
      // double velToAdd = copysign(velY,(pow((pow(velX,2)+pow(velY,2)),0.5)));//*velY/abs(velY); // in m/s
      double velToAdd = ((velX < 0) - (velX > 0))*(pow((pow(velX,2)+pow(velY,2)),0.5));//*velY/abs(velY); // in m/s
      // Create the vertices for the points and lines


      points.header = gpsFix_msg.header;
      points.header.frame_id = "/my_frame" ;


      
      points.ns  = "gps";
      points.action  = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = 1.0;

      points.id = 0;

      points.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
   

      // POINTS markers use x and y scale for width/height respectively

      points.scale.z = 2;

      points.pose.position.x = 0.0;
      points.pose.position.y = -2.0;
      points.pose.position.z = 0.0;

      // Points are green
      
      points.color.a = 1.0;
      
      points.color.r = 1.0f;
      points.color.g = 0.55f;
      points.color.b = 0.0f;



      std::ostringstream lat;
      std::ostringstream lon;
      std::ostringstream vel;

      lat<<latitude;
      lon<<longitude;
      vel<<velToAdd;


      points.text = "Latitude:" +lat.str()+ " N, Longitude:" + lon.str() + " W, Velocity:" + vel.str() + "m/s";

      std::cout<<points.text<<std::endl;



      marker_pub_fix.publish(points);

  }

  public:
   void run()
   {
    ros::NodeHandle node_handle_;

    message_filters::Subscriber<sensor_msgs::NavSatFix> gpsFix_sub(node_handle_, "/fix", 1);
    message_filters::Subscriber<geometry_msgs::TwistStamped> gpsVel_sub(node_handle_, "/vel", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::TwistStamped> MySyncPolicy;
      // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)

   

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), gpsFix_sub, gpsVel_sub);
    sync.registerCallback(boost::bind(&VIS::synCallback, this, _1, _2));

    marker_pub_fix = node_handle_.advertise<visualization_msgs::Marker>("visualization_gps", 10);
    //marker_pub_vel = node_handle_.advertise<visualization_msgs::Marker>("visualization_syn_vel", 10);


    ros::spin();
   }

  ~VIS()
  {

  }

  VIS()
  {

  }

};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "vis_syn");
  
  VIS app;;
  app.run();

  return 0;
  
}
