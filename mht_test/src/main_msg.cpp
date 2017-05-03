#include <iostream>
#include <list>
#include <map>

#include <Eigen/Dense>

#include <openmht/multi/MHT.h>
#include <openmht/plot/Plot.h>

#include <boost/random.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/math/distributions/uniform.hpp>
#include <boost/random/normal_distribution.hpp>

///////////////////////////////////////////////////////
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <beginner_tutorials/RadarTCP.h>
#include <cmath>

ros::Publisher filt_pub;

double dt = 1.0/20.0; //update period of 20Hz 
//  start an instance of openMHT
openmht::MHT mht;   
// set the sensor update period (seconds).
// mht.set_dt(dt);
// std::list<openmht::Measurement> m_list_medium;
// std::list<openmht::Measurement> m_list_long;
std::list<openmht::Measurement> m_list;
// std::list<openmht::Entity> ents;
// std::map<int, std::list<Eigen::Vector2d> > tracked;
// std::map<int, Eigen::Vector2d > tracked;
int medFlag = 0;
double amp_th = 5.0;
///////////////////////////////////////////////////////



using std::cout;
using std::endl;

// class Contact {
// public:
//      int id;
//      Eigen::Vector2d position;
//      Eigen::Vector2d velocity;
// };


// // Generate random numbers with normal dist
// boost::mt19937 gener_;
// boost::normal_distribution<> normal_dist_(0,1.0);
// boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > rng_normal(gener_,normal_dist_);

// void step_dynamics(std::list<Contact> &contacts, double dt)
// {
//      for (std::list<Contact>::iterator it = contacts.begin(); 
//           it != contacts.end(); it++) {
//           it->position = it->position + it->velocity*dt;
//      }
// }

// Contact add_noise(Contact &contact)
// {
//      Contact c = contact;          
//      c.position(0,0) = c.position(0,0) + rng_normal();
//      c.position(1,0) = c.position(1,0) + rng_normal();          
//      return c;
// }



void FilterCallback(const beginner_tutorials::RadarTCP::ConstPtr& input) {
    beginner_tutorials::RadarTCP msg = *input;
    

    if (msg.scanType == 1)  //medium range (clear and add)
    {
        medFlag = 1;
        cout << "in medium range" << endl;
        // Clear
        m_list.clear();

        // Add medium
        for (uint32_t i = 0; i < 64; ++i)
        {
            if (msg.p_rng[i] > 0.0 && msg.p_amp[i] > amp_th)
            {
                openmht::Measurement m;
                
                double x = msg.p_rng[i] * sin(msg.p_ang[i]*0.0174533);
                double y = msg.p_rng[i] * cos(msg.p_ang[i]*0.0174533);
                
                Eigen::Vector2d position(x,y);
                
                m.set_position(position);
                m_list.push_back(m);
            }
        }

    // } else {                //long range (add and process)
    } else if (medFlag == 1){                //long range (add and process)

        medFlag = 0;
        cout << "in long range" << endl;

        // Add long
        for (uint32_t i = 0; i < 64; ++i)
        {
            if (msg.p_rng[i] > 0.0 && msg.p_amp[i] > amp_th)
            {
                openmht::Measurement m;
                
                double x = msg.p_rng[i] * sin(msg.p_ang[i]*0.0174533);
                double y = msg.p_rng[i] * cos(msg.p_ang[i]*0.0174533);
                
                Eigen::Vector2d position(x,y);
                
                m.set_position(position);
                m_list.push_back(m);
            }
        }

        // Process
        mht.process_measurements(m_list);

        m_list.clear();
        // Get a list of the fused entities / contacts:
        // ents.clear();
        std::list<openmht::Entity> ents_radar = mht.entities();


    

        visualization_msgs::MarkerArray radarArray;
        // cout << "ent size" << ents.size() << endl;

        // std::list<openmht::Entity>::iterator it = ents.begin();

        // cout << "......" << endl;


        // Save results in tracked 
        std::map<int, Eigen::Vector2d > tracked;
        int trackedSize = 0;
        for (std::list<openmht::Entity>::iterator it = ents_radar.begin(); it != ents_radar.end(); it++) {            
            // tracked[it->id()].push_back(it->position());
            tracked[it->id()] = it->position();
            trackedSize++;
        }
        // cout << "tracked num = " << tracked.size() << endl;
        cout << "tracked size = " << trackedSize << endl;

        radarArray.markers.resize(trackedSize);
        //radarArray.markers.resize(tracked.size());

        // Create the vertices for the points and lines
        for (int tr_id = 0; tr_id < trackedSize; tr_id++) 
        {
        
            // cout << "......................................." << endl;

            // position =  depth.pos;
            // geometry_msgs::Point p;
            // cout << tr_id << ",";
            // cout << "2345" << endl;

            // radarArray.markers[tr_id].header.stamp = ros::Time::now();//msg.header.stamp;
            radarArray.markers[tr_id].header = msg.header;
            radarArray.markers[tr_id].header.frame_id = "/my_frame";


            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one

            // if (msg.scanType == 1)
            // {
            //   radarArray.markers[i].ns = "radar_vis_medium";
            // } else {
            //   radarArray.markers[i].ns = "radar_vis_long";        
            // }

            radarArray.markers[tr_id].ns = "radar_filt_vis";
            radarArray.markers[tr_id].id = tr_id;
            // cout << "456" << endl;

            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
            radarArray.markers[tr_id].type = visualization_msgs::Marker::ARROW;
            // cout << "2346" << endl;
            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            radarArray.markers[tr_id].action = visualization_msgs::Marker::ADD;

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

            radarArray.markers[tr_id].pose.position.z = 0;
            radarArray.markers[tr_id].pose.orientation.x = 0.0;
            radarArray.markers[tr_id].pose.orientation.y = 0.0;
            radarArray.markers[tr_id].pose.orientation.z = 1.0;
            radarArray.markers[tr_id].pose.orientation.w = 1.0;
            // cout << "asdfqw2345" << endl;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            radarArray.markers[tr_id].scale.x = 1.0;
            radarArray.markers[tr_id].scale.y = 1.0;
            radarArray.markers[tr_id].scale.z = 1.0;

            // Set the color -- be sure to set alpha to something non-zero!
            radarArray.markers[tr_id].color.r = 0.0f;
            radarArray.markers[tr_id].color.g = 0.0f;
            radarArray.markers[tr_id].color.b = 1.0f;
            radarArray.markers[tr_id].color.a = 1.0;



            // if (msg.p_amp[i] < 0.1) //Low amplitude detection 
            // {
            //     radarArray.markers[i].color.a = 0.1;   // 0.1
            // } else if (msg.p_amp[i] > 10.0) { // high amplitude
            //     radarArray.markers[i].color.a = 1.0;
            // } else {
            //     radarArray.markers[i].color.a = 0.3;   // 0.7
            // }


            // if (msg.p_rng[i] < 1.0) //Too close
            // {
            //     radarArray.markers[i].color.a = 0.0;
            // } 

            // cout << tr_id << endl;


            Eigen::Vector2d filtPos = tracked[tr_id];
            // Eigen::Vector2d filtPos = tracked[tr_id].back();
            // cout << "asdf" << endl;

            radarArray.markers[tr_id].pose.position.x = filtPos(0);
            // cout << "asdf124" << endl;

            // cout << sin(msg.p_ang[i]) << endl;
            radarArray.markers[tr_id].pose.position.y = filtPos(1);
            // cout << "asdf234" << endl;

            double chk_ang = (-1.0 * atan2(filtPos(1),filtPos(0)) * 180.0 /3.14159265) + 90.0;


            if (filtPos(1)<1.0 || abs(chk_ang) > 45.0 || filtPos(1)>120.0)
            {
                radarArray.markers[tr_id].color.a = 0.0;   
            }


            if (filtPos(1)>60.0 &&  abs(chk_ang) > 10.0)
            {
                radarArray.markers[tr_id].color.a = 0.0;
            }

            // cout << cos(msg.p_ang[i]) << endl << endl;
            //p.z = 0;
            // points.points.push_back(p);

            // if (msg.scanType == 1)
            // {
            //     radarArray.markers[i].color.r = 1.0f;//medium
            //     if (msg.p_rng[i] > 50.0 || msg.p_ang[i] > 45.0 || msg.p_ang[i] < -45.0)
            //     {
            //       radarArray.markers[i].color.a = 0.9;
            //       radarArray.markers[i].type = visualization_msgs::Marker::SPHERE;
            //       radarArray.markers[i].color.r = 0.5f;
            //       radarArray.markers[i].color.g = 0.5f;
            //       radarArray.markers[i].color.b = 0.5f;
            //     }
            // } else {
            //     radarArray.markers[i].color.g = 1.0f;//long
            //     if (msg.p_rng[i] > 125.0 || msg.p_ang[i] > 10.0 || msg.p_ang[i] < -10.0)
            //     {
            //       radarArray.markers[i].color.a = 0.9;
            //       radarArray.markers[i].type = visualization_msgs::Marker::SPHERE;
            //       radarArray.markers[i].color.r = 0.5f;
            //       radarArray.markers[i].color.g = 0.5f;
            //       radarArray.markers[i].color.b = 0.5f;
            //     }
            // }
        }
        // ents.clear();
        cout << "end loop" << endl << endl;


        filt_pub.publish(radarArray);

    }

          // The line list needs two points for each line
        


}




int main(int argc, char* argv[]) {

    ros::init(argc, argv, "main_msg");
    ros::NodeHandle nrf;
    // ros::Subscriber sub_person = n.subscribe("/person_pos", 1000, MarkerCallback);
    // ros::Subscriber sub_car = n.subscribe("/car_pos", 1000, MarkerCallback);
    ros::Subscriber sub_radar = nrf.subscribe("parsed_radar", 100, FilterCallback);
    filt_pub = nrf.advertise<visualization_msgs::MarkerArray>("radar_filt_vis", 100);


// Lines for legend
    // ros::Publisher lines_pub = n.advertise<visualization_msgs::Marker>("lines_marker", 10);
    // visualization_msgs::Marker points, line_strip, line_list;
    
    // points.header.frame_id = line_list.header.frame_id = "/my_frame";
    // points.header.stamp = line_list.header.stamp = ros::Time::now();
    // points.ns = line_list.ns = "lines_marker";
    // points.action =  line_list.action = visualization_msgs::Marker::ADD;
    // points.pose.orientation.w =  line_list.pose.orientation.w = 1.0;


    // points.id = 0;
    // // line_strip.id = 1;
    // line_list.id = 2;


    // points.type = visualization_msgs::Marker::POINTS;
    // line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    // line_list.type = visualization_msgs::Marker::LINE_LIST;

    // // POINTS markers use x and y scale for width/height respectively
    // points.scale.x = 0.2;
    // points.scale.y = 0.2;

    // // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    // // line_strip.scale.x = 0.1;
    // line_list.scale.x = 0.1;


    // // Points are green
    // points.color.g = 1.0f;
    // points.color.a = 1.0;

    // // Line strip is blue
    // line_strip.color.b = 1.0;
    // line_strip.color.a = 1.0;

    // // Line list is red
    // line_list.color.r = 1.0;
    // line_list.color.a = 1.0;

    // geometry_msgs::Point p;
    // p.x = (int32_t)i - 50;
    // p.y = y;
    // p.z = z;

    // points.points.push_back(p);
    // line_strip.points.push_back(p);

    // // The line list needs two points for each line
    // line_list.points.push_back(p);
    // p.z += 1.0;
    // line_list.points.push_back(p);

    // marker_pub.publish(points);
    // marker_pub.publish(line_strip);
    // marker_pub.publish(line_list);

    mht.set_dt(dt);


    ros::spin();

    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////
// Generate fake data 

        // //generating rand dists seeded on time 
  //    rng_normal.engine().seed(static_cast<unsigned int>(std::time(0)));
  //    rng_normal.distribution().reset();
     

  //    // For example data 
  //    int num_contacts = 3;   
  //    std::list<Contact> contacts;


  //    //Generating random readings 
  //    for (int i = 0; i < num_contacts; i++) {
  //         Contact c;     
  //         c.id = i;
  //         c.position << rng_normal()*10, rng_normal()*10;
  //         c.velocity << rng_normal()*1, rng_normal()*1;
  //         contacts.push_back(c);
  //    }

  //    // Printing rands with norm dist 
  //    for (int i = 0; i < 100; i++) {
  //         cout << rng_normal() << endl;
  //    }
     
///////////////////////////////////////////////////////////////////////////////////////////////
// Actual code starts 





     // TODO modify this to work continously using messages? 
     // double t0 = 0; // start time
     // double dt = 1; //update period 
     // double tend = 20; //time end 
     
     // openmht::Plot plot;
     // std::map<int, std::list<Eigen::Vector2d> > truth, measured, tracked;
          
     // //  start an instance of openMHT
     // openmht::MHT mht;   
     // // set the sensor update period (seconds).
     // mht.set_dt(dt);


     // for (double t = t0; t < tend; t += dt) {

     //      // Convert from your list of 2D detection points to a list of openMHT measurements 
     //      std::list<openmht::Measurement> m_list;


     //      // Add all new points to the list to process them (after for loop)
     //      for (std::list<Contact>::iterator it = contacts.begin(); it != contacts.end(); it++) {

     //           //latest measurement from sensor? 
     //           Contact c = add_noise(*it);
               
     //           //add measurement to list 
     //           openmht::Measurement m;
     //           m.set_position(c.position);
     //           m_list.push_back(m);

     //           // Save the measured position for plotting later 
     //           //(this uses the id, otherwise algo doesnt seem to)
     //           measured[c.id].push_back(c.position);
     //      }
          
     //      // Process the list of measurements with MHT
     //      // Runs algos and checks for culled tracks and if needed to be declared dead 
     //      mht.process_measurements(m_list);

     //      // Get a list of the fused entities / contacts:
     //      std::list<openmht::Entity> ents = mht.entities();

     //      // Save the filtered locations of each track
     //      for (std::list<openmht::Entity>::iterator it = ents.begin(); it != ents.end(); it++) {            
     //           tracked[it->id()].push_back(it->position());
     //      }                              
          
     //      // Save the truth tracks:
     //      for (std::list<Contact>::iterator it = contacts.begin(); it != contacts.end(); it++) {
     //           truth[it->id].push_back(it->position);
     //      }

     //      // Plot the results:
     //      std::string title = "Tracks";
     //      std::list< std::map<int, std::list<Eigen::Vector2d> > > lists;
     //      std::list<std::string> labels;
     //      std::list<std::string> styles;
     //      std::string options = "";
     //      std::list<std::string> objects;
          
     //      lists.push_back(truth);
     //      labels.push_back("Truth");
     //      styles.push_back("linespoints");
          
     //      lists.push_back(measured);
     //      labels.push_back("Measured");
     //      styles.push_back("linespoints");
          
     //      lists.push_back(tracked);          
     //      labels.push_back("Tracked");
     //      styles.push_back("linespoints");

     //      plot.plot(lists, title, labels, styles, options, objects, false);          

     //      // Step dynamics:
     //      step_dynamics(contacts,dt);
     // }
     // plot.wait();

