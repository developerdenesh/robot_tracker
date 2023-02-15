
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: D
 *********************************************************************/

#ifndef ROBOT_TRACKER_H_
#define ROBOT_TRACKER_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <std_srvs/Trigger.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

// For conversion from quaternion to euler
#include "tf/transform_datatypes.h"

// Added dynamic reconfigure
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include <robot_tracker/json.hpp>
using json = nlohmann::ordered_json;

// This is used to read files
#include <fstream>

namespace robot_tracker
{
  class RobotTracker
  {
  public:
    /**
     * @brief Construct a new Cleaning Tracker object
     *
     */
    RobotTracker();

    /**
     * @brief Destroy the Cleaning Tracker object
     *
     */
    ~RobotTracker();

    /**
     * @brief This method initialises all the publishers, subsribers, service and actions servers and clients
     *
     */
    void initialisePubSubSrvActions();

    /**
     * @brief Initialise the dynamic dynamic reconfigure and grab some important ROS parameters
     *
     */
    void initialiseParameters();

    /**
     * @brief Reset the numbers by setting their string representatives to take on the value of empty strings
     *
     * @param first
     * @param second
     */
    void resetNumbers(std::string &first, std::string &second);

    /**
     * @brief Take in the stringified JSON and convert it into a set of std::vector
     *
     * @param data
     * @param return_this_set_of_coordinates
     */
    void parseStringIntoVector(std::string &data, std::vector<std::pair<double, double>> &return_this_set_of_coordinates);

    /**
     * @brief This service will be called to provide the zone coordinates information from cleaning_chef
     *
     * @return true
     * @return false
     */
    bool readZoneInformation();

    /**
     * @brief Callback to obtain information about the robot pose
     *
     * @param data
     */
    void robotPoseCallback(const geometry_msgs::Pose::ConstPtr &data);

    /**
     * @brief Check if robot pose is valid (does not contain nan or inf values)
     *
     * @param data
     * @return true
     * @return false
     */
    bool isRobotPoseValid(const geometry_msgs::Pose &data);

    /**
     * @brief use the robot's position to check if it is within the zones
     *
     * @return true
     * @return false
     */
    std::string checkifRobotIsInPolygon(const geometry_msgs::Pose &temp_pose_container);

    /**
     * @brief Publish which zone the robot is currently in, if it is not in a zone, publish an empty string like so: ""
     *
     */
    void pubRobotZoneInfo(const std::string &result_of_check);

    /**
     * @brief Publish the text of the cleaning zone for easier visualisation
     *
     */
    void pubCleaningZoneNameToRviz(std::string data);

    bool dummy_variable_{false};
    int publish_throttle_in_seconds_{1};

    ros::Subscriber robot_pose_sub_;
    ros::Publisher robot_zone_info_pub_;
    ros::Publisher polygon_text_visualisation_pub_;

    std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr;

    // Take note that the default access-specifier for structs are public
    struct zone_coordinates
    {
      std::string name;
      std::vector<std::pair<double, double>> coordinates;
    };

    std::vector<zone_coordinates> zone_coordinates_;

  private:
    ros::NodeHandle nh_;
    ros::Time last_time_;

    std::string robot_pose_topic_{"/robot_pose"};
    std::string json_file_name_{"robot_zones.json"};

    geometry_msgs::Pose robot_pose_;
    nav_msgs::Path nav_message_;

    int dummy_int_;
    double dummy_double_;
    enum charactor_symbols
    {
      open_square_bracket = '[',
      close_square_bracket = ']',
      comma = ',',
      ending_square_bracket = 1,
    };
  };
}

#endif
