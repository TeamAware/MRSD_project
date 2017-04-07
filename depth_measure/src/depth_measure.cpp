/* The depth_measure node subscribe to 3 topics 
1. depth map named("depth") published by elas node, format is sensor_msg/image
2. detectin results named("/obj_car/image_obj" & "/obj_person/image_obj")
format is 
Header header
string type
image_rect[] obj
where image_rect[] is
int32 x
int32 y
int32 height
int32 width
float32 score
*/
#include <iostream>
#include <vector>
#include <algorithm> 
#include <string>
#include <memory>
#include <iomanip>
#include <iosfwd>
#include <utility>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


#include <cv_tracker/image_obj.h>
// #include <rect_class_score.h>

#include <depth_measure/depth.h>
// #include <rect_class_score.h>

// #include <opencv2/contrib/contrib.hpp>
// #include <opencv2/highgui/highgui.hpp>

using namespace std;

using namespace sensor_msgs;
using namespace message_filters;

using namespace cv_tracker;
class DepthMeasureApp
{
  ros::Subscriber subscriber_depth_map_;
  ros::Subscriber subscriber_ssd_car_;
  ros::Subscriber subscriber_ssd_person_;
  ros::Publisher publisher_car_position_;
  ros::Publisher publisher_person_position_;
  ros::NodeHandle node_handle_;

  // TODO: convert the detection x y to depth map coordinate.  Need to find out the coord
  // assume the output should be like [xmin, ymin, height, width]
  std::vector<int> convert_det_res_to_image_coord(cv_tracker::image_obj& in_message)
  {

    std::vector<int> coord(4,0);
    // auto test = image_obj;
    // std::cout<<image_obj.obj;
    
    // std_msgs::Header h = in_message.header;
    // cout<<h<<endl;
    // string type = in_message.type;
    // cout<<type<<endl;

   std::vector<cv_tracker::image_rect> objects = in_message.obj;
   int size = objects.size();
   if (size>0)
   {
   		 coord[0] = objects.at(0).x;
   		 coord[1] = objects.at(0).y;
   		 coord[2] = objects.at(0).height;
   		 coord[3] = objects.at(0).width;
   }
    return coord;
  }

  void image_callback(const sensor_msgs::ImageConstPtr& DepthMap, const cv_tracker::image_objConstPtr& orig_image_obj)
  {
  	cout<<"callback"<<endl;
    // show time stamp 
  	// std_msgs::Header hd = DepthMap->header;
  	// cout<<"depth"<<endl;
   //  // cout<<h<<endl; //all at once
   //  cout<<hd.stamp<<endl; //specific parts of it
   //  cout<<hd.seq<<endl;

   //  std_msgs::Header h = orig_image_obj->header;
  	// cout<<"obj"<<endl;
   //  // cout<<h<<endl; //all at once
   //  cout<<h.stamp<<endl; //specific parts of it
   //  cout<<h.seq<<endl;
  	
    cv_tracker::image_obj image_obj = *orig_image_obj;

    sensor_msgs::Image depth_map = *DepthMap;
    
     // if (orig_image_obj.type == "car") {};
    //Receive Image, convert it to OpenCV Mat
    depth_measure::depth output_message;
    
    output_message.header = depth_map.header;
    
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(depth_map, "mono8");//toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat image = cv_image->image;
    
    
    std::vector<int> coor = DepthMeasureApp::convert_det_res_to_image_coord(image_obj);
    // cout<< "out"<<endl;
    // std::vector<int> coor(4,0);
    // coor[0] = image_obj.obj[0].x;
    // coor[1] = image_obj.obj[0].y;
    // coor[2] = image_obj.obj[0].height;
    // coor[3] = image_obj.obj[0].width;
    std::vector<float> dep;
    for (int x = coor[0]; x< coor[0]+coor[2]; x++)
    {
      for (int y = coor[1]; y< coor[1]+coor[3]; y++)
      {

        float di = image.at<float>(y,x);

        // TODO: don't know the format of the depth 
        if ((di < 100000) && (di > 100))
         {
          	dep.push_back(di);

        }
      }
      // cout<<"image"<<endl;

    }

    if (dep.size()>0)
      {
        std::sort(dep.begin(), dep.end());
        output_message.x = coor[0]+coor[1]/2;
        output_message.depth = dep[int(dep.size()/2)]/1000;
        cout<<output_message.depth<<"output"<<endl;
      }

    // output_message.x = 10;
    // output_message.depth = 20;
    
    
    if (image_obj.type == "car")
    {
      cout<<"car"<<endl;
      output_message.type = "car";
      publisher_car_position_.publish(output_message);
    }
    else if (image_obj.type == "person")
    {
      cout<<"person"<<endl;
      output_message.type = "person";
      publisher_person_position_.publish(output_message);
    }
    else
      std::cout<<"Do not find car or person";
  }
  /// try to extract the time stamp
  // void testcallback(const sensor_msgs::ImageConstPtr& DepthMap)
  // {
  // 	std_msgs::Header h = DepthMap->header;
  // 	cout<<"obj"<<endl;
  //   // cout<<h<<endl; //all at once
  //   cout<<h.stamp<<endl; //specific parts of it
  //   cout<<h.seq<<endl;
  // }
  // void testcallback2(const cv_tracker::image_objConstPtr& orig_image_obj)
  // {
  // 	std_msgs::Header h = orig_image_obj->header;
  // 	cout<<"obj"<<endl;
  //   // cout<<h<<endl; //all at once
  //   cout<<h.stamp<<endl; //specific parts of it
  //   cout<<h.seq<<endl;
  // }
public:
  void Run()
  {
    //ROS STUFF
    ros::NodeHandle private_node_handle("~");//to receive args


    //RECEIVE IMAGE TOPIC NAME
    std::string depth_map_topic_str;
    if (private_node_handle.getParam("stereo_vision_node", depth_map_topic_str))
    {
      ROS_INFO("Setting image node to %s", depth_map_topic_str.c_str());
    }
    else
    {
      ROS_INFO("No stereo vision node received, defaulting to elas_ros/depth, you can use stereo_vision_node:=YOUR_TOPIC");
      depth_map_topic_str = "elas_ros/depth";
    }

    //cout<<"*****"<<endl;
    //RECEIVE CONVNET FILENAMES
    std::string ssd_car_topic_str;
    std::string ssd_person_topic_str;
    if (private_node_handle.getParam("ssd_car_topic_str", ssd_car_topic_str))
    {
      ROS_INFO("ssd_car_topic_str: %s", ssd_car_topic_str.c_str());
    }
    else
    {
      ROS_INFO("No ssd_car_topic_str was received. Finishing execution.");
      return;
    }
    if (private_node_handle.getParam("ssd_person_topic_str", ssd_person_topic_str))
    {
      ROS_INFO("ssd_person_topic_str: %s", ssd_person_topic_str.c_str());
    }
    else
    {
      ROS_INFO("No ssd_person_topic_str was received. Finishing execution.");
      return;
    }

    // publish
    publisher_car_position_ = node_handle_.advertise<depth_measure::depth>("/car_pos", 1);
    
    publisher_person_position_ = node_handle_.advertise<depth_measure::depth>("/person_pos", 1);

    


    // subscribe  &  synchronize
    // synchronize
    // TODO: make sure the synchronize method 
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handle_, depth_map_topic_str, 1);
    message_filters::Subscriber<cv_tracker::image_obj> car_sub(node_handle_, ssd_car_topic_str, 1);
    message_filters::Subscriber<cv_tracker::image_obj> person_sub(node_handle_, ssd_person_topic_str, 1);

// test time stamp
    // subscriber_depth_map_ = node_handle_.subscribe(depth_map_topic_str, 1, &DepthMeasureApp::testcallback, this);
    // subscriber_ssd_person_ = node_handle_.subscribe(ssd_person_topic_str, 1, &DepthMeasureApp::testcallback2, this);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, cv_tracker::image_obj> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), depth_sub, car_sub);
    sync.registerCallback(boost::bind(&DepthMeasureApp::image_callback, this, _1, _2));

    Synchronizer<MySyncPolicy> sync2(MySyncPolicy(100), depth_sub, person_sub);
    sync2.registerCallback(boost::bind(&DepthMeasureApp::image_callback,this, _1, _2));


    
    /*std::string config_topic("/config");  config_topic += ros::this_node::getNamespace() + "/ssd";
    subscriber_ssd_config_ =node_handle_.subscribe(config_topic, 1, &RosSsdApp::config_cb, this);*/

    ros::spin();
    ROS_INFO("END DepthMeasure");

  }

  ~DepthMeasureApp()
  {
   
  }

  DepthMeasureApp()
  {
    
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_measure");

  DepthMeasureApp app;
 
  app.Run();

  return 0;
}