#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <depth_measure/depth.h>
#include <cmath>

ros::Publisher marker_pub;
void MarkerCallback(const depth_measure::depthConstPtr& input)
{
    depth_measure::depth depth = *input;
    visualization_msgs::Marker points;

    points.header = depth.header;
    points.header.frame_id = "/my_frame" ;
	  std::vector<depth_measure::position> position = depth.pos;
    int num = depth.pos.size();
    
    points.ns  = "car";
    points.action  = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;
 

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 1;
    points.scale.y = 1;

    // Points are green
    
    points.color.a = 1.0;

    points.color.r = 1.0f;


    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < num; ++i)
    {
      position =  depth.pos;
      geometry_msgs::Point p;
      p.x = position.at(i).x;
      p.y = position.at(i).z;
      //p.z = 0;
      points.points.push_back(p);


      // The line list needs two points for each line
    }
      // std::cout<<"test car"<<std::endl;

    marker_pub.publish(points);

}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "vis_car");
  ros::NodeHandle n;
  ros::Subscriber sub_car = n.subscribe("/car_pos", 1000, MarkerCallback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_car", 10);
 
  ros::spin();

  return 0;
  
}
