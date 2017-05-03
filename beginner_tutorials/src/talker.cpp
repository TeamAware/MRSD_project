/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>


///////////////////////////////////////////////////////////////////////
//RADAR stuff


// INCLUDES AND INIT
#include <iostream>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <string.h>
#include <boost/program_options.hpp>
#include <boost/asio.hpp>
// #include <boost/array.hpp>

// #include <RadarTCP.msg>
#include <beginner_tutorials/RadarTCP.h>

using namespace std;
using namespace boost;
using boost::asio::ip::tcp;
namespace po = boost::program_options;

const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;
const unsigned int ESR_XCP_PAYLOAD_SIZE = 8568;
const unsigned int ESR_DSP_VER_1_OFFSET = 93;
const unsigned int ESR_DSP_VER_1_SIZE = 1;
const unsigned int ESR_DSP_VER_2_OFFSET = 100;
const unsigned int ESR_DSP_VER_2_SIZE = 1;
const unsigned int ESR_DSP_VER_3_OFFSET = 107;
const unsigned int ESR_DSP_VER_3_SIZE = 1;
const unsigned int ESR_SCAN_TYPE_OFFSET = 30;
const unsigned int ESR_SCAN_TYPE_SIZE = 1;
const unsigned int ESR_SCAN_INDEX_OFFSET = 14;
const unsigned int ESR_SCAN_INDEX_SIZE = 2;
const unsigned int ESR_TGT_RPT_CNT_OFFSET = 4350;
const unsigned int ESR_TGT_RPT_CNT_SIZE = 2;
const unsigned int ESR_TGT1_RNG_OFFSET = 4626;
const unsigned int ESR_TGT1_RNG_SIZE = 2;

bool quit = false;

const unsigned int RNG_OFFSET = 4358;
const unsigned int RNG_SIZE = 2;
const unsigned int RAT_OFFSET = 4626;
const unsigned int RAT_SIZE = 2;
const unsigned int ANG_OFFSET = 4894;
const unsigned int ANG_SIZE = 2;
const unsigned int AMP_OFFSET = 5028;
const unsigned int AMP_SIZE = 2;


// HELPER FUNCTIONS
void shutdown_handler(int s) {
    printf("Caught signal %d. Shutting down ESR 2.5 TCP Parser...\n", s);
    quit = true;
}

unsigned long read_value(std::array<unsigned char, ESR_XCP_PAYLOAD_SIZE> &bufArray, unsigned int offset, unsigned int size) {
    unsigned long retVal = 0;

    for (unsigned int i = size; i > 0; i--) {
        retVal <<= 8;
        //Need to use -1 because array is 0-based
        //and offset is not.
        retVal |= bufArray[(offset - 1) + i];
    }

    return retVal;
}



// END RADAR STUFF 
///////////////////////////////////////////////////////////////////////


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  // ros::Publisher radar_parse_pub = n.advertise<std_msgs::String>("parsed_radar", 1000);
  ros::Publisher radar_parse_pub = n.advertise<beginner_tutorials::RadarTCP>("parsed_radar", 1000);


// %EndTag(PUBLISHER)%


//MIGHT NEED TO REMOVE FOR CONTINOUS DATA STREAM
// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%

/////////////////////////////////////////////////////////////////////
  //RADAR MAIN INIT
  // int count = 0; //SHOULDNT BE NEEDED 

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = shutdown_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  string ip;
  unsigned long port = 0;
///////////////////////////////////////////////////////////////////////
// RADAR PRE LOOP 

  try {
      po::options_description desc("Options");
      desc.add_options()
          ("help,h", "Display this help menu and exit.")
          ("ip,i", po::value<string>(), "Provide the IP address for the parser to use to connect to the ESR. Otherwise, the default of 169.254.145.71 is used.")
          ("port,p", po::value<unsigned long>(&port), "Provide the port for the parser to use to connect to the ESR. Otherwise, the default of 5555 is used.");

      po::variables_map vm;

      try {
          po::store(po::parse_command_line(argc, argv, desc), vm);

          if (vm.count("help")) {
              cout << endl;
              cout << "  AutonomouStuff ESR 2.5 TCP Packet Parser  " << endl;
              cout << "  -h               Display this help menu and exit." << endl;
              cout << "  -i <ip_address>  Provide the IP address for the parser to use to connect to the ESR." << endl;
              cout << "                   Otherwise, the default of 169.254.145.71 is used." << endl;
              cout << "  -p <port>        Provide the port for the parser to use to connect to the ESR. Otherwise, " << endl;
              cout << "                   the default of 5555 is used." << endl;
              cout << endl;
              return SUCCESS;
          }

          if (vm.count("ip")) {
              ip = vm["ip"].as<string>();
          }
          
          if (vm.count("port")) {
              port = vm["port"].as<int>();
          }

          po::notify(vm);
      } catch (po::error& e) {
          cerr << "ERROR: " << e.what() << endl << endl;
          cerr << desc << endl;
          return ERROR_IN_COMMAND_LINE;
      }
  } catch (std::exception& e) {
      cerr << "Unhandled exception reached while processing arguments: "
           << e.what() << ", application will now exit." << endl;
      return ERROR_UNHANDLED_EXCEPTION;
  }

  //Default IP address for 12V ESR 2.5
  string finalIP = "192.168.2.21";
  if (!ip.empty()) {
      finalIP = ip;
  }

  cout << "IP Address: " << finalIP << endl;

  //Default port for 12V ESR 2.5
  //Default for 24V is 4444
  unsigned long finalPort = 5555;
  if (port > 0) {
      finalPort = port;
  }
  
  stringstream sPort;
  sPort << finalPort;
  cout << "Port: " << sPort.str() << endl;

  std::array<unsigned char, ESR_XCP_PAYLOAD_SIZE> xcpMsgBuf;
  bool gotFirstPayload = false;
  bool gotLastPacket = false;
  unsigned int packetCounter = 0;

  asio::io_service io;
  tcp::resolver res(io);
  tcp::resolver::query query(tcp::v4(), finalIP.c_str(), sPort.str());
  tcp::resolver::iterator it = res.resolve(query);
  tcp::socket socket(io);

  //////////////////////////////////////////////////////////////////////////////
// connection and parse

try {
        socket.connect(*it);
        cout << "Connected!" << endl;

        while (ros::ok()) {
            if (quit) break;

            beginner_tutorials::RadarTCP msg;


            // cout << "in while loop" << endl;
            std::array<unsigned char, ESR_XCP_PAYLOAD_SIZE> msgBuf;
            size_t rcvSize;
            system::error_code error;

            rcvSize = socket.read_some(asio::buffer(msgBuf), error);
            // cout << rcvSize << endl;

            if (gotFirstPayload == true) {
                // cout << "gotFirstPayload!" << endl;

                for (unsigned int i = 0; i < rcvSize; i++) {
                    xcpMsgBuf[packetCounter + i] = msgBuf[i];
                }

                packetCounter += rcvSize;
            }
                
            if (rcvSize == 1268) {
                gotFirstPayload = true;
                //Reset everything because we got the remainder packet.
                gotLastPacket = true;
                //xcpMsgBuf.fill(0);
                packetCounter = 0;
            } else {
                gotLastPacket = false;
            }

            if (gotLastPacket == true) {
                cout << "parsing!" << endl;


                //Get bytes in little-endian order.
                unsigned long dsp1 = (uint8_t)read_value(xcpMsgBuf, ESR_DSP_VER_1_OFFSET, ESR_DSP_VER_1_SIZE);
                unsigned long dsp2 = (uint8_t)read_value(xcpMsgBuf, ESR_DSP_VER_2_OFFSET, ESR_DSP_VER_2_SIZE);
                unsigned long dsp3 = (uint8_t)read_value(xcpMsgBuf, ESR_DSP_VER_3_OFFSET, ESR_DSP_VER_3_SIZE);
                cout << "DSP Version: " << dsp1 << "." << dsp2 << "." << dsp3 << endl;
                unsigned long scanType = read_value(xcpMsgBuf, ESR_SCAN_TYPE_OFFSET, ESR_SCAN_TYPE_SIZE);
                cout << "Scan type: " << scanType << endl;
                unsigned long scanIndex = (uint16_t)read_value(xcpMsgBuf, ESR_SCAN_INDEX_OFFSET, ESR_SCAN_INDEX_SIZE);
                cout << "Scan index: " << scanIndex << endl;
                unsigned long timeStamp = (uint32_t)read_value(xcpMsgBuf, 83, 4);
                cout << "Time Stamp: " << timeStamp << endl;
                // unsigned long gLookIndex = (uint16_t)read_value(xcpMsgBuf, 51, 2);
                // cout << "gLookIndex: " << gLookIndex << endl;
                // unsigned long mmrIndex = (uint16_t)read_value(xcpMsgBuf, 59, 2);
                // cout << "mmrIndex: " << mmrIndex << endl;
                unsigned long targetCount = (uint16_t)read_value(xcpMsgBuf, ESR_TGT_RPT_CNT_OFFSET, ESR_TGT_RPT_CNT_SIZE);
                cout << "Target count: " << targetCount << endl << endl;
                // uint16_t target1Rng = (uint16_t)read_value(xcpMsgBuf, ESR_TGT1_RNG_OFFSET, ESR_TGT1_RNG_SIZE);
                // cout << "Target 1 range: " << target1Rng << endl << endl;
                // cout << "Target 1 range: " << (float)target1Rng << endl << endl;

                boost::array<float,64> rng_arr;
                boost::array<float,64> rat_arr;
                boost::array<float,64> ang_arr;
                boost::array<float,64> amp_arr;
                // int16_t rat_arr[64];
                // int16_t ang_arr[64];
                // int16_t amp_arr[64];
                
                float ang_offset = 7.0; // in degrees

                for (int t_id = 0; t_id < 64; ++t_id)
                {
                  /* code */
                  rng_arr[t_id] = ((int16_t)read_value(xcpMsgBuf, RNG_OFFSET + (t_id*RNG_SIZE), RNG_SIZE))*1.0/128.0;
                  //[0,204.7] SCALE: 0.1 OFFSET: 200
                  rat_arr[t_id] = ((int16_t)read_value(xcpMsgBuf, RAT_OFFSET + (t_id*RAT_SIZE), RAT_SIZE))*1.0/128.0;
                  //[-81.92,81.91] SCALE: 0.01 OFFSET: 81.91
                  ang_arr[t_id] = (((int16_t)read_value(xcpMsgBuf, ANG_OFFSET + (t_id*ANG_SIZE), ANG_SIZE))*1.0/128.0)-ang_offset;
                  //[-51.2,51.1] SCALE: 0.1 OFFSET: 0
                  amp_arr[t_id] = ((int16_t)read_value(xcpMsgBuf, AMP_OFFSET + (t_id*AMP_SIZE), AMP_SIZE))*1.0/128.0;


                }

                  msg.header.stamp = ros::Time::now();
                  msg.scanType = scanType;
                  msg.timeStamp = timeStamp;
                  msg.scanIndex = scanIndex;
                  msg.targetCount = targetCount; 
                  msg.p_rng = rng_arr;
                  msg.p_rat = rat_arr; 
                  msg.p_ang = ang_arr;
                  msg.p_amp = amp_arr; 


                  radar_parse_pub.publish(msg);

            }

           

            // cout << "msg assigned!" << endl;


        // %Tag(PUBLISH)%
        // %EndTag(PUBLISH)%

        // %Tag(SPINONCE)%
            ros::spinOnce();
        // %EndTag(SPINONCE)%

        // %Tag(RATE_SLEEP)%
            // loop_rate.sleep();
        // %EndTag(RATE_SLEEP)%
            // ++count;            
        }
    } catch (std::exception& e) {
        cerr << e.what() << endl;
        socket.close();
        return ERROR_UNHANDLED_EXCEPTION;
    }

    socket.close();

    return SUCCESS; 
}
///////////////////////////////////////////////////////////////////////////////////
//   while (ros::ok())
//   {
// // %EndTag(ROS_OK)%
//     /**
//      * This is a message object. You stuff it with data, and then publish it.
//      */
// // %Tag(FILL_MESSAGE)%
//     // std_msgs::String msg;

//     // std::stringstream ss;
//     // ss << "hello world " << count;
//     // msg.data = ss.str();

//     beginner_tutorials::RadarTCP msg;
//     msg.scanType = 1;
//     msg.scanIndex = count;
//     msg.targetCount = 64; 
//     msg.p_range = 1000;
//     msg.p_rangeRate = 666; 
//     msg.p_theta = 30;
//     msg.p_amplitude = 10;

// // %EndTag(FILL_MESSAGE)%

// // %Tag(ROSCONSOLE)%
//     // ROS_INFO("%s", msg.data.c_str());
// // %EndTag(ROSCONSOLE)%

//     /**
//      * The publish() function is how you send messages. The parameter
//      * is the message object. The type of this object must agree with the type
//      * given as a template parameter to the advertise<>() call, as was done
//      * in the constructor above.
//      */
// // %Tag(PUBLISH)%
//     radar_parse_pub.publish(msg);
// // %EndTag(PUBLISH)%

// // %Tag(SPINONCE)%
//     ros::spinOnce();
// // %EndTag(SPINONCE)%

// // %Tag(RATE_SLEEP)%
//     loop_rate.sleep();
// // %EndTag(RATE_SLEEP)%
//     ++count;
//   }


//   return 0;
// }
// %EndTag(FULLTEXT)%
