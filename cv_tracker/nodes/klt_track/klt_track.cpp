/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//ROS STUFF
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_tracker/image_obj.h>
#include <cv_tracker/image_obj_tracked.h>
#include <cv_tracker/image_obj_ranged.h>

//TRACKING STUFF
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "LkTracker.hpp"

#include <iostream>
#include <stdio.h>

#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

using namespace std;
class RosTrackerApp
{
	ros::Subscriber 	subscriber_image_raw_;
	ros::Subscriber 	subscriber_image_obj_;
	ros::Subscriber 	subscriber_klt_config_;
	ros::Publisher 		publisher_tracked_objects_;//ROS
	ros::NodeHandle 	node_handle_;

	std::string 		tracked_type_;

	bool 				ready_;

	bool				track_ready_;
	bool				detect_ready_;

	int					num_trackers_;

	std::vector<LkTracker*> obj_trackers_;
	std::vector<cv::LatentSvmDetector::ObjectDetection> obj_detections_;

	std::vector<float> ranges_;
	std::vector<float> min_heights_;
	std::vector<float> max_heights_;

	cv_tracker::image_obj_tracked ros_objects_msg_;//sync

	void Sort(const std::vector<float> in_scores, std::vector<unsigned int>& in_out_indices)
	{
		for (unsigned int i = 0; i < in_scores.size(); i++)
			for (unsigned int j = i + 1; j < in_scores.size(); j++)
			{
				if (in_scores[in_out_indices[j]] > in_scores[in_out_indices[i]])
				{
					//float x_tmp = x[i];
					int index_tmp = in_out_indices[i];
					//x[i] = x[j];
					in_out_indices[i] = in_out_indices[j];
					//x[j] = x_tmp;
					in_out_indices[j] = index_tmp;
				}
			}
	}

	void ApplyNonMaximumSuppresion(std::vector< LkTracker* >& in_out_source, float in_nms_threshold)
	{
		if (in_out_source.empty())
			return;

		unsigned int size = in_out_source.size();

		std::vector<float> area(size);
		std::vector<float> scores(size);
		std::vector<int> x1(size);
		std::vector<int> y1(size);
		std::vector<int> x2(size);
		std::vector<int> y2(size);
		std::vector<unsigned int> indices(size);
		std::vector<bool> is_suppresed(size);

		for(unsigned int i = 0; i< in_out_source.size(); i++)
		{
			cv::LatentSvmDetector::ObjectDetection tmp = in_out_source[i]->GetTrackedObject();
			area[i] = tmp.rect.width * tmp.rect.height;
			if (area[i]>0)
				is_suppresed[i] = false;
			else
			{
				is_suppresed[i] = true;
				in_out_source[i]->NullifyLifespan();
			}
			indices[i] = i;
			scores[i] = tmp.score;
			x1[i] = tmp.rect.x;
			y1[i] = tmp.rect.y;
			x2[i] = tmp.rect.width + tmp.rect.x;
			y2[i] = tmp.rect.height + tmp.rect.y;
		}

		Sort(area, indices);//returns indices ordered based on scores

		for(unsigned int i=0; i< size; i++)
		{

			for(unsigned int j= i+1; j< size; j++)
			{
				if(is_suppresed[indices[i]] || is_suppresed[indices[j]])
					continue;
				int x1_max = std::max(x1[indices[i]], x1[indices[j]]);
				int x2_min = std::min(x2[indices[i]], x2[indices[j]]);
				int y1_max = std::max(y1[indices[i]], y1[indices[j]]);
				int y2_min = std::min(y2[indices[i]], y2[indices[j]]);
				int overlap_width = x2_min - x1_max + 1;
				int overlap_height = y2_min - y1_max + 1;
				if(overlap_width > 0 && overlap_height>0)
				{
					float overlap_part = (overlap_width*overlap_height)/area[indices[j]];
					if(overlap_part > in_nms_threshold)
					{
						is_suppresed[indices[j]] = true;
						in_out_source[indices[j]]->NullifyLifespan();
						if (in_out_source[indices[j]]->GetFrameCount() > in_out_source[indices[i]]->GetFrameCount())
						{
							in_out_source[indices[i]]->object_id = in_out_source[indices[j]]->object_id;
						}

					}
				}
			}
		}
		return ;
	}

	void publish_if_possible()
	{
		
		ROS_INFO(" image %d\n",track_ready_);
		ROS_INFO(" detect %d\n",detect_ready_);
		if (track_ready_ && detect_ready_)
		{
			
			ROS_INFO("  prepare to publish\n");
			publisher_tracked_objects_.publish(ros_objects_msg_);
			track_ready_ = false;
			detect_ready_ = false;
			ROS_INFO("  finished publish\n");
		}
	}

public:
	void image_callback(const sensor_msgs::Image& image_source)
	{
		
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
		cv::Mat image_track = cv_image->image;

		cv::LatentSvmDetector::ObjectDetection empty_detection(cv::Rect(0,0,0,0),0,0);
		unsigned int i;

		std::vector<bool> tracker_matched(obj_trackers_.size(), false);
		std::vector<bool> object_matched(obj_detections_.size(), false);
		 
		//check object detections vs current trackers
		// ROS_INFO("%d",int(obj_detections_.size()));
		for (i =0; i< obj_detections_.size(); i++)
		{
			ROS_INFO(" I %u\n", i);
			ROS_INFO(" trackers size %lu\n",  obj_trackers_.size());
			for (unsigned int j = 0; j < obj_trackers_.size(); j++)
			{
				ROS_INFO(" j %u\n", j);
				if (tracker_matched[j] || object_matched[i])
					continue;

				cv::LatentSvmDetector::ObjectDetection tmp_detection = obj_detections_[i];
				int area = tmp_detection.rect.width * tmp_detection.rect.height;
				cv::Rect intersection = tmp_detection.rect & obj_trackers_[j]->GetTrackedObject().rect;
				if ( (intersection.width * intersection.height) > area*0.3 )
				{

					obj_trackers_[j]->Track(image_track, obj_detections_[i], true);
					tracker_matched[j] = true;
					object_matched[i] = true;
					//std::cout << "matched " << i << " with " << j << std::endl;
				}
			}
		}
		
		//run the trackers not matched
		for(i = 0; i < obj_trackers_.size(); i++)
		{
			ROS_INFO(" trackers not match %u\n", i);
			if(!tracker_matched[i])
			{
				ROS_INFO("START TO TRACK");
				obj_trackers_[i]->Track(image_track, empty_detection, false);
				ROS_INFO("finished TO TRACK");
			}
		}

		//create trackers for those objects not being tracked yet
		for(unsigned int i=0; i<obj_detections_.size(); i++)
		{
			ROS_INFO(" trackers new obj %u\n", i);
			if (!object_matched[i])//if object wasn't matched by overlapping area, create a new tracker
			{
				if (num_trackers_ >10)
					num_trackers_=0;
				ROS_INFO("CREATE NEW TRACKERS");
				LkTracker* new_tracker = new LkTracker(++num_trackers_);
				new_tracker->Track(image_track, obj_detections_[i], true);
				ROS_INFO("finished NEW TRACKERS");
				//std::cout << "added new tracker" << std::endl;
				obj_trackers_.push_back(new_tracker);
			}
		}
		
		ApplyNonMaximumSuppresion(obj_trackers_, 0.3);

		//remove those trackers with its lifespan <=0
		std::vector<LkTracker*>::iterator it;
		for(it = obj_trackers_.begin(); it != obj_trackers_.end();)
		{
			if ( (*it)->GetRemainingLifespan()<=0 )
			{
				it = obj_trackers_.erase(it);
				//std::cout << "deleted a tracker " << std::endl;
			}
			else
				it++;
		}

		//copy results to ros msg
		unsigned int num = obj_trackers_.size();

		std::vector<cv_tracker::image_rect> rect_array;//tracked rectangles
		std::vector<int> real_data(num,0);//boolean array to show if data in rect_ranged comes from tracking or detection
		std::vector<unsigned int> obj_id(num, 0);//id number for each rect_range
		std::vector<unsigned int> lifespan(num, 0);//remaining lifespan of each rectranged
		
		for(i=0; i < num; i++)
		{
			ROS_INFO("prepare the output message");
			cv_tracker::image_rect rect;
			LkTracker tracker_tmp = *obj_trackers_[i];
			rect.x = tracker_tmp.GetTrackedObject().rect.x;
			rect.y = tracker_tmp.GetTrackedObject().rect.y;
			rect.width = tracker_tmp.GetTrackedObject().rect.width;
			rect.height = tracker_tmp.GetTrackedObject().rect.height;
			rect.score = tracker_tmp.GetTrackedObject().score;
			// rect_ranged.max_height = tracker_tmp.max_height_;
			// rect_ranged.min_height = tracker_tmp.min_height_;
			// rect_ranged.range = tracker_tmp.range_;
			ROS_INFO("rect x %d\n", rect.x);
			ROS_INFO("rect y %d\n", rect.y);
			ROS_INFO("rect width %d\n", rect.width);
			rect_array.push_back(rect);

			lifespan[i] = tracker_tmp.GetRemainingLifespan();
			obj_id[i] = tracker_tmp.object_id;
			if(lifespan[i]==tracker_tmp.DEFAULT_LIFESPAN_)
				real_data[i] = 1;

			cv::rectangle(image_track, tracker_tmp.GetTrackedObject().rect, cv::Scalar(0,255,0), 2);
			ROS_INFO(" rect data");
		}

		//std::cout << "TRACKERS: " << obj_trackers_.size() << std::endl;

		obj_detections_.clear();
        ranges_.clear();
        if (num>0)
        {
        	cv_tracker::image_obj_tracked tmp_objects_msg;

			tmp_objects_msg.type = tracked_type_;
			tmp_objects_msg.total_num = num;
			copy(rect_array.begin(), rect_array.end(), back_inserter(tmp_objects_msg.rect)); // copy vector
			copy(real_data.begin(), real_data.end(), back_inserter(tmp_objects_msg.real_data)); // copy vector
			copy(obj_id.begin(), obj_id.end(), back_inserter(tmp_objects_msg.obj_id)); // copy vector
			copy(lifespan.begin(), lifespan.end(), back_inserter(tmp_objects_msg.lifespan)); // copy vector
			ROS_INFO("tem x %d\n", tmp_objects_msg.rect.at(0).x);
			ROS_INFO("tem y %d\n", tmp_objects_msg.rect.at(0).y);
			ROS_INFO("tem width %d\n", tmp_objects_msg.rect.at(0).width);
			tmp_objects_msg.header = image_source.header;

			ros_objects_msg_ = tmp_objects_msg;

			//publisher_tracked_objects_.publish(ros_objects_msg);

			//cv::imshow("KLT",image_track);
			//cv::waitKey(1);

			track_ready_ = true;
        }
		
		ROS_INFO("image callback");
		//ready_ = false;

		publish_if_possible();

	}

	// void detections_callback(cv_tracker::image_obj_ranged image_objects_msg)
	// {
	// 	//if(ready_)
	// 	//	return;
	// 	if (!detect_ready_)//must NOT overwrite, data is probably being used by tracking.
	// 	{
	// 		unsigned int num = image_objects_msg.obj.size();
	// 		std::vector<cv_tracker::image_rect_ranged> objects = image_objects_msg.obj;
	// 		tracked_type_ = image_objects_msg.type;
	// 		//points are X,Y,W,H and repeat for each instance
	// 		obj_detections_.clear();
 //            ranges_.clear();
            
	// 		for (unsigned int i=0; i<num;i++)
	// 		{
	// 			cv::Rect tmp;
	// 			tmp.x = objects.at(i).rect.x;
	// 			tmp.y = objects.at(i).rect.y;
	// 			tmp.width = objects.at(i).rect.width;
	// 			tmp.height = objects.at(i).rect.height;
	// 			obj_detections_.push_back(cv::LatentSvmDetector::ObjectDetection(tmp, 0));
	// 			ranges_.push_back(objects.at(i).range);
	// 			min_heights_.push_back(objects.at(i).min_height);
	// 			max_heights_.push_back(objects.at(i).max_height);
	// 		}
	// 		detect_ready_ = true;
	// 	}

	// 	publish_if_possible();
	// 	//ready_ = true;
	// }
	void detections_callback(cv_tracker::image_obj image_objects_msg)
	{
		// 
		// ROS_INFO("detection");
	//	if (ready_)
	//		return;
	
		ready_ = false;
		if (!detect_ready_)//must NOT overwrite, data is probably being used by tracking.
		{
			unsigned int num = image_objects_msg.obj.size();
			std::cout<<"num"<<num<<std::endl;
			std::vector<cv_tracker::image_rect> objects = image_objects_msg.obj;
			//object_type = image_objects_msg.type;
			//points are X,Y,W,H and repeat for each instance
			obj_detections_.clear();
			tracked_type_ = image_objects_msg.type;
			for (unsigned int i=0; i<num;i++)
			{
				cv::Rect tmp;
				tmp.x = objects.at(i).x;
				tmp.y = objects.at(i).y;
				tmp.width = objects.at(i).width;
				tmp.height = objects.at(i).height;
				obj_detections_.push_back(cv::LatentSvmDetector::ObjectDetection(tmp, 0));
				std::cout<<obj_detections_.size()<<endl;
			}
			detect_ready_ = true;
		}
			publish_if_possible();
		ready_ = true;
		ROS_INFO("detection callback");
	}

	void klt_config_cb()
	{

	}


	void Run()
	{
		std::string image_raw_topic_str;
		std::string image_obj_topic_str;

		ros::NodeHandle private_node_handle("~");//to receive args

		if (private_node_handle.getParam("image_raw_node", image_raw_topic_str))
			{
				ROS_INFO("Setting image node to %s", image_raw_topic_str.c_str());
			}
		else
		{
			ROS_INFO("No image node received, defaulting to /image_raw, you can use _image_raw_node:=YOUR_TOPIC");
			image_raw_topic_str = "/stereo/left/image_raw";
		}
		if (private_node_handle.getParam(ros::this_node::getNamespace() + "/img_obj_node", image_obj_topic_str))
			{
				ROS_INFO("Setting object node to %s", image_obj_topic_str.c_str());
			}
		else
		{
			ROS_INFO("No object node received, defaulting to image_obj_ranged, you can use _img_obj_node:=YOUR_TOPIC");
			image_obj_topic_str = "/obj_car/image_obj";
		}


		publisher_tracked_objects_ = node_handle_.advertise<cv_tracker::image_obj_tracked>("image_obj_tracked", 1);

		ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
		ROS_INFO("Subscribing to... %s", image_obj_topic_str.c_str());

		subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &RosTrackerApp::image_callback, this);
		subscriber_image_obj_ = node_handle_.subscribe(image_obj_topic_str, 1, &RosTrackerApp::detections_callback, this);
		
		// std::string config_topic("/config");
		// config_topic += ros::this_node::getNamespace() + "/klt";
		//node_handle.subscribe(config_topic, 1, &RosTrackerApp::klt_config_cb, this);

		ros::spin();
		ROS_INFO("END klt");
	}

	RosTrackerApp()
	{
		ready_ = true;
		num_trackers_ = 0;
		track_ready_  = false;
		detect_ready_ = false;
	}

};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "klt");

	RosTrackerApp app;

	app.Run();

	return 0;
}
