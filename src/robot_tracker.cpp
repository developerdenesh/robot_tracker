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

#include <robot_tracker/robot_tracker.h>

namespace robot_tracker
{
    RobotTracker::RobotTracker() : dummy_variable_(true)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s initialised with: %d", rclcpp::Node::get_name().c_str(), dummy_variable_);
        last_time_ = rclcpp::Clock().now();
        initialisePubSubSrvActions();
        readZoneInformation();
    }

    RobotTracker::~RobotTracker() {}

    // ======================================================================================================
    // ======================= Initialisation Functions ======================================================
    // ======================================================================================================
    void RobotTracker::initialisePubSubSrvActions()
    {
        // Create a subscriber to obtain information about the robot's pose
        robot_pose_sub_ = nh_.subscribe(robot_pose_topic_, 1, &RobotTracker::robotPoseCallback, this);
        robot_zone_info_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_zone_info", 10, true);
        polygon_text_visualisation_pub_ = nh_.advertise<visualization_msgs::Marker>(ros::this_node::getName() + "/polygon_text", 1, true);

        // Initialise global robot pose to 0
        robot_pose_.position.x = 0.0;
        robot_pose_.position.y = 0.0;
        robot_pose_.position.z = 0.0;
        robot_pose_.orientation.x = 0.0;
        robot_pose_.orientation.y = 0.0;
        robot_pose_.orientation.z = 0.0;
        robot_pose_.orientation.w = 1.0;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The pose subscribed to is: %s", robot_pose_sub_.getTopic().c_str());
    }


    // ======================================================================================================
    // ======================= Zone Tracking Functions ======================================================
    // ======================================================================================================
    bool RobotTracker::readZoneInformation()
    {
        std::string path = ament_index_cpp::get_package_share_directory("robot_tracker");
        std::ifstream ifs(path + "/" + json_file_name_);
        json coordinate_information_in_json_format = json::parse(ifs);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received the following json: %s\n", coordinate_information_in_json_format.dump().c_str());

        zone_coordinates_.clear();

        // Now iterate through the json
        for (json::iterator it = coordinate_information_in_json_format.begin(); it != coordinate_information_in_json_format.end(); ++it)
        {
            // Define a new instance
            zone_coordinates zone_coordinate_instance;

            // Convert the coordinates to a standard string
            std::string stringyfied_coordinates = std::string(it.value().dump());

            // Debug statements
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Zone name: %s", std::string(it.key()).c_str());
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates: %s\n", stringyfied_coordinates.c_str());

            // Obtain the coordinates in the form of c++ pairs
            std::vector<std::pair<double, double>> coordinates;
            parseStringIntoVector(stringyfied_coordinates, coordinates);

            // Populate the vector with the coordinate information
            zone_coordinate_instance.name = std::string(it.key());
            zone_coordinate_instance.coordinates = coordinates;
            zone_coordinates_.push_back(zone_coordinate_instance);
        }

        return true;
    }

    void robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr data)
    {
        geometry_msgs::msg::Pose temp_pose_container;
        temp_pose_container.position.x = data->position.x;
        temp_pose_container.position.y = (*data).position.y;
        temp_pose_container.position.z = (*data).position.z;

        temp_pose_container.orientation.x = (*data).orientation.x;
        temp_pose_container.orientation.y = (*data).orientation.y;
        temp_pose_container.orientation.z = (*data).orientation.z;
        temp_pose_container.orientation.w = (*data).orientation.w;

        if (isRobotPoseValid(temp_pose_container))
        {
            std::string result_of_check = checkifRobotIsInPolygon(temp_pose_container);
            pubRobotZoneInfo(result_of_check);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Robot pose is invalid");
        }
    }

    bool RobotTracker::isRobotPoseValid(const geometry_msgs::msg::Pose &data)
    {
        if (std::isinf(data.position.x) || std::isnan(data.position.y) || std::isinf(data.position.y) || std::isnan(data.position.y))
        {
            return false;
        }
        return true;
    }

    std::string RobotTracker::checkifRobotIsInPolygon(const geometry_msgs::msg::Pose &robot_pose)
    {
        std::vector<std::string> zones_robot_is_in;

        for (int i = 0; i < zone_coordinates_.size(); i++)
        {
            bool is_in = false;
            std::vector<std::pair<double, double>> fake_polygon_coordinates = zone_coordinates_[i].coordinates;
            int size = fake_polygon_coordinates.size();

            double p_x, p_y, x1, x2, y1, y2;

            p_x = robot_pose.position.x;
            p_y = robot_pose.position.y;

            for (int j = 0; j < size - 1; j++)
            {
                x1 = fake_polygon_coordinates[j].first;
                y1 = fake_polygon_coordinates[j].second;
                x2 = fake_polygon_coordinates[j + 1].first;
                y2 = fake_polygon_coordinates[j + 1].second;

                // Check #1: if the y is between the y of the 2 points
                // Check #2: then check if the x is less than the x along the line with the same y coordinate
                if ((p_y < y1 != p_y < y2) && (p_x < (x2 - x1) * (p_y - y1) / (y2 - y1) + x1))
                {
                    is_in = !is_in;
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the value of is in is: %d", is_in);

            // Make sure the full round of the convex polygon is checked by checking index 0 again
            if (size > 1)
            {
                x1 = fake_polygon_coordinates[size - 1].first;
                y1 = fake_polygon_coordinates[size - 1].second;
                x2 = fake_polygon_coordinates[0].first;
                y2 = fake_polygon_coordinates[0].second;

                // Check #1: if the y is between the y of the 2 points
                // Check #2: then check if the x is less than the x along the line with the same y coordinate
                if ((p_y < y1 != p_y < y2) && (p_x < (x2 - x1) * (p_y - y1) / (y2 - y1) + x1))
                {
                    is_in = !is_in;
                }
            }

            if (is_in)
            {
                zones_robot_is_in.push_back(std::string(zone_coordinates_[i].name));
            }
        }

        // If robot is in zones, return the first zone the robot is found to be in, else return an empty string
        return (zones_robot_is_in.empty()) ? ("") : (zones_robot_is_in[0]);
    }

    void RobotTracker::pubRobotZoneInfo(const std::string &result_of_check)
    {
        // Publish the zone message at a throttled speed
        std_msgs::msg::String result;
        result.data = result_of_check;
        robot_zone_info_pub_->publish(result);
        pubCleaningZoneNameToRviz(result_of_check);
    }

    void RobotTracker::pubCleaningZoneNameToRviz(std::string data)
    {
        visualization_msgs::msg::Marker vis_message;
        geometry_msgs::msg::Point point_msg_no_thirty_two;

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = 0.0;
        pose_msg.position.y = 0.0;
        pose_msg.position.z = 0.0;

        pose_msg.orientation.x = 0.0;
        pose_msg.orientation.y = 0.0;
        pose_msg.orientation.z = 0.0;
        pose_msg.orientation.w = 1.0;

        vis_message.pose = pose_msg;
        if (data.empty())
        {
            vis_message.text = " ";
        }
        else
        {
            vis_message.text = data;
        }
        vis_message.type = 9;
        vis_message.action = 0;

        std_msgs::msg::ColorRGBA color_variable;
        color_variable.a = 1;
        color_variable.b = 1;
        color_variable.g = 1;
        color_variable.r = 1;

        geometry_msgs::msg::Vector3 scale_variable;
        scale_variable.x = 0.5;
        scale_variable.y = 0.5;
        scale_variable.z = 0.5;

        vis_message.scale = scale_variable;
        vis_message.color = color_variable;

        vis_message.header.frame_id = "base_link";
        vis_message.header.stamp = rclcpp::Clock().now();
        polygon_text_visualisation_pub_->publish(vis_message);
    }

    // ======================================================================================================
    // ======================= Helper Functions ======================================================
    // ======================================================================================================
    void RobotTracker::resetNumbers(std::string &first, std::string &second)
    {
        // Reset the numbers by setting the string to an empty string
        first = "";
        second = "";
    }

    void RobotTracker::parseStringIntoVector(std::string &data, std::vector<std::pair<double, double>> &set_of_coordinates_to_be_returned)
    {
        int open_bracket_count = 0;
        std::string first_number = "";
        std::string second_number = "";
        bool first_word = true;

        // An example of the string that could be parsed into a vector
        // "[[4.244000212252141, 1.794285890034268], [4.189143068577563, -1.5245713022777], [9.15371457112687, -1.5794284459522792], [9.044000283777715, 1.4651430279868]]"
        for (char const &c : data)
        {
            if (c == open_square_bracket)
            {
                open_bracket_count++;
            }
            else if (c == close_square_bracket)
            {
                first_word = true;
                open_bracket_count--;

                if (open_bracket_count == ending_square_bracket)
                {
                    // Convert strings to double using standard library
                    double first_number_in_double_format = std::stod(first_number);
                    double second_number_in_double_format = std::stod(second_number);
                    set_of_coordinates_to_be_returned.push_back(std::make_pair(first_number_in_double_format, second_number_in_double_format));
                    resetNumbers(first_number, second_number);
                }
            }
            else if (open_bracket_count == 2)
            {
                if (c == comma)
                {
                    first_word = false;
                }
                else
                {
                    (first_word) ? (first_number += c) : (second_number += c);
                }
            }
        }
    }

}
