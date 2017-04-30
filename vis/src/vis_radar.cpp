#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <depth_measure/depth.h>
#include <beginner_tutorials/RadarTCP.h>
#include <cmath>

ros::Publisher marker_pub;
void MarkerCallback(const beginner_tutorials::RadarTCP::ConstPtr& input)
{
    beginner_tutorials::RadarTCP msg = *input;
    

    visualization_msgs::MarkerArray radarArray;
    radarArray.markers.resize(64);


    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 64; ++i)
    {
      // position =  depth.pos;
      // geometry_msgs::Point p;

      radarArray.markers[i].header.frame_id = "/my_frame";
      radarArray.markers[i].header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one

      if (msg.scanType == 1)
      {
        radarArray.markers[i].ns = "radar_vis_medium";
      } else {
        radarArray.markers[i].ns = "radar_vis_long";        
      }
      radarArray.markers[i].id = i;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      radarArray.markers[i].type = visualization_msgs::Marker::ARROW;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      radarArray.markers[i].action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

      radarArray.markers[i].pose.position.z = 0;
      radarArray.markers[i].pose.orientation.x = 0.0;
      radarArray.markers[i].pose.orientation.y = 0.0;
      radarArray.markers[i].pose.orientation.z = 1.0;
      radarArray.markers[i].pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      radarArray.markers[i].scale.x = 1.0;
      radarArray.markers[i].scale.y = 1.0;
      radarArray.markers[i].scale.z = 1.0;

      // Set the color -- be sure to set alpha to something non-zero!
      radarArray.markers[i].color.r = 0.0f;
      radarArray.markers[i].color.g = 0.0f;
      radarArray.markers[i].color.b = 1.0f;



      if (msg.p_amp[i] < 0.1) //Low amplitude detection 
      {
        radarArray.markers[i].color.a = 0.1;   // 0.1
      } else if (msg.p_amp[i] > 10.0) { // high amplitude
        radarArray.markers[i].color.a = 1.0;
      } else {
        radarArray.markers[i].color.a = 0.3;   // 0.7
      }


      if (msg.p_rng[i] < 1.0) //Too close
      {
        radarArray.markers[i].color.a = 0.0;
      } 

      radarArray.markers[i].pose.position.x = msg.p_rng[i] * sin(msg.p_ang[i]*0.0174533);
      // cout << sin(msg.p_ang[i]) << endl;
      radarArray.markers[i].pose.position.y = msg.p_rng[i] * cos(msg.p_ang[i]*0.0174533) - 1;
      // cout << cos(msg.p_ang[i]) << endl << endl;
      //p.z = 0;
      // points.points.push_back(p);

      if (msg.scanType == 1)
      {
        radarArray.markers[i].color.r = 1.0f;//medium
        if (msg.p_rng[i] > 50.0 || msg.p_ang[i] > 45.0 || msg.p_ang[i] < -45.0)
        {
          radarArray.markers[i].color.a = 0.9;
          radarArray.markers[i].type = visualization_msgs::Marker::SPHERE;
          radarArray.markers[i].color.r = 0.5f;
          radarArray.markers[i].color.g = 0.5f;
          radarArray.markers[i].color.b = 0.5f;
        }
      } else {
        radarArray.markers[i].color.g = 1.0f;//long
        if (msg.p_rng[i] > 125.0 || msg.p_ang[i] > 10.0 || msg.p_ang[i] < -10.0)
        {
          radarArray.markers[i].color.a = 0.9;
          radarArray.markers[i].type = visualization_msgs::Marker::SPHERE;
          radarArray.markers[i].color.r = 0.5f;
          radarArray.markers[i].color.g = 0.5f;
          radarArray.markers[i].color.b = 0.5f;
        }
      }


      // The line list needs two points for each line
    }


    marker_pub.publish(radarArray);
}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "vis_radar");
  ros::NodeHandle nr;
  // ros::Subscriber sub_person = n.subscribe("/person_pos", 1000, MarkerCallback);
  // ros::Subscriber sub_car = n.subscribe("/car_pos", 1000, MarkerCallback);
  ros::Subscriber sub_radar = nr.subscribe("parsed_radar", 100, MarkerCallback);
  marker_pub = nr.advertise<visualization_msgs::MarkerArray>("visualization_radar", 10);
 
  ros::spin();

  return 0;
  
}
