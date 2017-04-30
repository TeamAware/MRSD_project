#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <depth_measure/depth.h>
#include <beginner_tutorials/RadarTCP.h>
#include <cmath>

class VIS
{
    ros::Publisher marker_pub_radar;
  ros::Publisher marker_pub_car;
  ros::Publisher marker_pub_person;


  void synCallback(const beginner_tutorials::RadarTCPConstPtr& radar_input, const depth_measure::depthConstPtr& car_input, const depth_measure::depthConstPtr& person_input, const geometry_msgs::TwistStampedConstPtr& gpsVel_input)
  {
      beginner_tutorials::RadarTCP msg = *radar_input;
      geometry_msgs::TwistStamped gpsVel_msg = *gpsVel_input;
      

      visualization_msgs::MarkerArray radarArray;
      radarArray.markers.resize(64);

      double velX = gpsVel_msg.twist.linear.x;
      double velY = gpsVel_msg.twist.linear.y;
      // assuming y is the direction we're heading in. (points ot the front of the car)
      double velToAdd = (pow((pow(velX,2)+pow(velY,2)),0.5))*velY/abs(velY); // in m/s
      // Create the vertices for the points and lines
      for (uint32_t i = 0; i < 64; ++i)
      {
        // position =  depth.pos;
        // geometry_msgs::Point p;

       
        radarArray.markers[i].header = msg.header;
        radarArray.markers[i].header.frame_id = "/my_frame";
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


        // Convert from relative rngrat to absolute velocity 
        double scaleVel = msg.p_rat[i] + velToAdd;


        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        radarArray.markers[i].scale.x = 1.0;
        radarArray.markers[i].scale.z = 1.0;
        
        radarArray.markers[i].scale.y = std::min(scaleVel / 5.0, 0.5);
        std::cout << "vel" << scaleVel << std::endl;
         // 1 m on the graph = 5m/s = 11.18 mph = 18 km/h
        //  minimum displayed is 2.5m/s = 9 km/h
        

        // Set the color -- be sure to set alpha to something non-zero!
        radarArray.markers[i].color.r = 0.0f;
        radarArray.markers[i].color.g = 0.0f;
        radarArray.markers[i].color.b = 1.0f;



        if (msg.p_amp[i] < 0.1) //Low amplitude detection 
        {
          radarArray.markers[i].color.a = 0.1;
        } else if (msg.p_amp[i] > 10.0) { // high amplitude
          radarArray.markers[i].color.a = 1.0;
        } else {
          radarArray.markers[i].color.a = 0.7;
        }


        if (msg.p_rng[i] < 1.0) //Too close
        {
          radarArray.markers[i].color.a = 0.0;
        } 

        radarArray.markers[i].pose.position.x = msg.p_rng[i] * sin(msg.p_ang[i]*0.0174533);
        // cout << sin(msg.p_ang[i]) << endl;
        radarArray.markers[i].pose.position.y = msg.p_rng[i] * cos(msg.p_ang[i]*0.0174533);
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

      marker_pub_radar.publish(radarArray);
      //// car marker
      depth_measure::depth depth = *car_input;
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

      marker_pub_car.publish(points);

      /// person marker

      depth_measure::depth depth_person = *person_input;
      visualization_msgs::Marker points_person;

      points_person.header = depth_person.header;
      points_person.header.frame_id = "/my_frame" ;
      std::vector<depth_measure::position> person_position = depth_person.pos;
      int num2 = depth_person.pos.size();
      
      points_person.ns  = "person";
      points_person.action  = visualization_msgs::Marker::ADD;
      points_person.pose.orientation.w = 1.0;

      points_person.id = 0;

      points_person.type = visualization_msgs::Marker::POINTS;
   

      // POINTS markers use x and y scale for width/height respectively
      points_person.scale.x = 1;
      points_person.scale.y = 1;

      // Points are green
      
      points_person.color.a = 1.0;
      points_person.color.g = 1.0f;



      // Create the vertices for the points and lines
      for (uint32_t i = 0; i < num2; ++i)
      {
        person_position =  depth_person.pos;
        geometry_msgs::Point p;
        p.x = position.at(i).x;
        p.y = position.at(i).z;
        //p.z = 0;
        points_person.points.push_back(p);

        // std::cout<<"test person"<<std::endl;
        // The line list needs two points for each line
      }


      marker_pub_person.publish(points_person);

  }

  public:
   void run()
   {
    ros::NodeHandle node_handle_;

    message_filters::Subscriber< beginner_tutorials::RadarTCP> radar_sub(node_handle_, "parsed_radar", 1);
    message_filters::Subscriber<depth_measure::depth> car_sub(node_handle_, "/car_pos", 1);
    message_filters::Subscriber<depth_measure::depth> person_sub(node_handle_, "/person_pos", 1);
    message_filters::Subscriber<geometry_msgs::TwistStamped> gpsVel_sub(node_handle_, "/vel", 1);

    typedef message_filters::sync_policies::ApproximateTime<beginner_tutorials::RadarTCP,depth_measure::depth,depth_measure::depth, geometry_msgs::TwistStamped> MySyncPolicy;
      // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)

   

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), radar_sub,car_sub, person_sub, gpsVel_sub);
    sync.registerCallback(boost::bind(&VIS::synCallback, this, _1, _2, _3, _4));

    marker_pub_radar = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_syn_radar", 10);
    marker_pub_car = node_handle_.advertise<visualization_msgs::Marker>("visualization_syn_car", 10);
    marker_pub_person = node_handle_.advertise<visualization_msgs::Marker>("visualization_syn_person", 10);

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
