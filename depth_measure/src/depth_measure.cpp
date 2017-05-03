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

#include <stereo_msgs/DisparityImage.h>

#include <cv_tracker/image_obj.h>
// #include <rect_class_score.h>

#include <depth_measure/depth.h>
// #include <rect_class_score.h>

#include <opencv2/core/core.hpp>
// #include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

// using namespace sensor_msgs;
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

	cv::Mat camera_matrix_left_;
	cv::Mat camera_matrix_right_;
	cv::Mat distcoeff_left_;
	cv::Mat distcoeff_right_;
	cv::Mat rectification_;
	cv::Mat depth_image_;
  cv::Mat Q_;
	float world_x_, world_y_, world_z_;
	// int image_u_, image_v_;
  
  
  void disparityCallback(const stereo_msgs::DisparityImageConstPtr& disp_map, const cv_tracker::image_objConstPtr& orig_image_obj)
  {

    // Extract info from object detection
  	cv_tracker::image_obj image_obj = *orig_image_obj;
  	std::vector<cv_tracker::image_rect> objects = image_obj.obj;
  	int size = objects.size();
  	cout << "Number of object(s):" << size << endl;

    stereo_msgs::DisparityImage disparity = *disp_map;

    /**********************************************************************************************/

    // Convert disparity map to depth image
    // std::cout << "<<<<<<<<<<<<<<<<<<<inside disparity callback<<<<<<<<<<<<<<<<<<<<< " << std::endl;

    // cv::Mat_<float> disparityMat(disparity.image.height,
    //                              disparity.image.width,
    //                              reinterpret_cast<float*>(&(disparity.image.data[0])));

    
    cv_bridge::CvImagePtr disp_ptr = cv_bridge::toCvCopy(disparity.image);
    cv::Mat_<float> disparityMat = disp_ptr->image;

    cv::Mat_<uint8_t> eightBitDisparityMat = disparityMat * (255/(disparity.max_disparity));   // For visulization ONLY

    // imwrite("/home/teama/Documents/disparity.png", eightBitDisparityMat);
  
    sensor_msgs::Image eightBitDisparity;

    uint32_t imageSize = disparity.image.height * disparity.image.width;

    eightBitDisparity.data.resize(imageSize);
    memcpy(&eightBitDisparity.data[0], &(eightBitDisparityMat.at<uint8_t>(0,0)), imageSize);

    ///@ stereo rectify
    // cv::Mat baseline_trans = (cv::Mat_<double>(3,1) << -disparity.T, 0, 0);
    // cv::Mat R1,R2,P1,P2,Q;

    // cv::stereoRectify(camera_matrix_left_, distcoeff_left_, camera_matrix_right_, distcoeff_right_, cv::Size(disparity.image.width, disparity.image.height),
    // rectification_, baseline_trans, R1, R2, P1, P2, Q);
    
    disparityMat = disparityMat*16;
    cv::reprojectImageTo3D(disparityMat, depth_image_, Q_, true);

    /*********************************************************************************************/
    
    // Create new output message
    depth_measure::depth output_message;
    output_message.header = disparity.header;



    ///@ extract object location in 3D world X Y Z
    for (int i = 0; i< size; i++) 
    {
    	std::vector<int> coor(4,0);
    	coor[0] = objects.at(i).x;
    	coor[1] = objects.at(i).y;
    	coor[2] = objects.at(i).height;
    	coor[3] = objects.at(i).width;
    	std::vector<float> x_w;
        std::vector<float> z_w;
        
        cv::rectangle(eightBitDisparityMat,cv::Rect(coor[0],coor[1],coor[3],coor[2]),cv::Scalar(250,250,0,255),10);
   
        // cout << "Entering the loop" << endl;

    	for (int x = coor[0]+coor[3]*0.2; x < coor[0]+coor[3]*0.8; x++)
    	{
    		for (int y = coor[1]+coor[2]*0.2; y < coor[1]+coor[2]*0.8; y++)
    		{
          
	          if ( x < 2048 && y < 1536)
	          {
	            world_z_ = depth_image_.at<cv::Vec3f>(y,x).val[2] / 1000;
	            world_x_ = depth_image_.at<cv::Vec3f>(y,x).val[0] / 1000;

	            if ( abs(world_z_-10.0) > 0.01 && world_z_ > 8 && world_z_ < 50)
	            //if ( abs(world_z_ -10.0) > 0.01)
	            {
	              //cout << world_z_ << endl;
	              z_w.push_back(world_z_);
	              x_w.push_back(world_x_);

	            } 
	          }  
    		}
    	}

      // cout << "vector size" << z_w.size() << " , " <<  x_w.size() << endl; 

    	if (z_w.size()>0 && x_w.size()>0)
    	{
    		std::sort(x_w.begin(), x_w.end());
    		std::sort(z_w.begin(), z_w.end());
    		depth_measure::position _3dpos;
    		// Find median 
    		_3dpos.u = coor[0]+coor[3]/2;
    		_3dpos.v = coor[1]+coor[2]/2;

        int mid_x = int(x_w.size()/2);
        int mid_z = int(z_w.size()/2);
        
    		//_3dpos.x = (x_w[mid_x-2]+x_w[mid_x-1]+x_w[mid_x]+x_w[mid_x+1]+x_w[mid_x+2])/5; // in meter
    	  //_3dpos.z = (z_w[mid_z-2]+z_w[mid_z-1]+z_w[mid_z]+z_w[mid_z+1]+z_w[mid_z+2])/5; // in meter

        _3dpos.x = x_w[mid_x]; // in meter
        _3dpos.z = z_w[mid_z]; // in meter
    		
                output_message.pos.push_back(_3dpos);
    		std::cout << "world coordinate (in m): ["<<_3dpos.x<<", "<<_3dpos.z<<"]"<<std::endl;
        }
     }
      
     cv::namedWindow("disparity image",0);
     cv::imshow("disparity image", eightBitDisparityMat);
     cv::waitKey(5);

     if (image_obj.type == "car")
     {
     	cout<<"car"<<endl;
     	output_message.type = "car";
     	publisher_car_position_.publish(output_message);
     }
     else if (image_obj.type == "person")
     {
     	ros::Duration(0.5).sleep();
     	cout<<"person"<<endl;
     	output_message.type = "person";
     	publisher_person_position_.publish(output_message);
     }
     else
     	std::cout<<"Do not find car or person"; 

     cout<< "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
  }


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
      ROS_INFO("No stereo vision node received, defaulting to elas_ros/disparity, you can use stereo_vision_node:=YOUR_TOPIC");
      depth_map_topic_str = "elas_ros/disparity";
      
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


    ///@ disparityImage subscriber
    // ros::Subscriber disparity_sub = node_handle_.subscribe("/stereo/disparity",1, &DepthMeasureApp::disparityCallback, this);

    // subscribe  &  synchronize
    message_filters::Subscriber<stereo_msgs::DisparityImage> depth_sub(node_handle_, depth_map_topic_str, 1);
    message_filters::Subscriber<cv_tracker::image_obj> car_sub(node_handle_, ssd_car_topic_str, 1);
    message_filters::Subscriber<cv_tracker::image_obj> person_sub(node_handle_, ssd_person_topic_str, 1);

    typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::DisparityImage, cv_tracker::image_obj> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)

 

    message_filters::Synchronizer<MySyncPolicy> sync2(MySyncPolicy(100), depth_sub, person_sub);
    sync2.registerCallback(boost::bind(&DepthMeasureApp::disparityCallback,this, _1, _2));

    

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), depth_sub, car_sub);
    sync.registerCallback(boost::bind(&DepthMeasureApp::disparityCallback,this, _1, _2));
    /*std::string config_topic("/config");  config_topic += ros::this_node::getNamespace() + "/ssd";
    subscriber_ssd_config_ =node_handle_.subscribe(config_topic, 1, &RosSsdApp::config_cb, this);*/

    ros::spin();
    ROS_INFO("END DepthMeasure");

  }

  ~DepthMeasureApp()
  {
   
  }

  DepthMeasureApp()
    //image_u_(0),
    //image_v_(0)
  {
    camera_matrix_left_ = (cv::Mat_<double>(3,3) << 2303.43746088419, -4.21282619389635, 923.015455365695, 0, 2310.01226770638, 732.247167052805, 0, 0, 1);
    camera_matrix_right_ = (cv::Mat_<double>(3,3) << 2296.96668143633, -16.4574536015561, 916.148685771905, 0, 2312.50483358507, 730.274159146837, 0, 0, 1);
    distcoeff_left_ = (cv::Mat_<double>(1,5) << -0.194316814591641, 0.0747425432700480, -0.00805513929417376, -0.000803816810343558, 0.102199531164671);
    distcoeff_right_ = (cv::Mat_<double>(1,5) << -0.201498647366297, 0.693041742817990,	-0.00561146185365105, -0.00000314777924943280, -2.98599033080007);
  	rectification_ = (cv::Mat_<double>(3,3) << 0.999943879164678, -0.00460803118572566, 0.00953963152783269, 0.00472530829082278, 0.999913090309601, -0.0123078547708965, -0.00948208746279643, 0.0123522417437556, 0.999878748719689);
    Q_ = (cv::Mat_<double>(4,4) << 1, 0, 0, -1154.214171409607, 0, 1, 0, -632.4434103965759, 0, 0, 0, 2169.743884915773, 0, 0, 0.001808307352261878, -0);
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_measure");

  DepthMeasureApp app;
 
  app.Run();

  return 0;
}
