#include <robot_tracker/robot_tracker.h>
#include <gtest/gtest.h>

class TestSuite : public ::testing::Test
{
private:
    ros::ServiceServer mServer;
    std::string mPlate;
    bool mFailService, mFailUnique;
    bool callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        mPlate = "SGK4255P";
        res.success = !mFailUnique;
        return !mFailService;
    }

public:
    TestSuite() : mFailService(false), mFailUnique(false)
    {
        mServer = ros::NodeHandle().advertiseService("carRegistry", &TestSuite::callback, this);
    }
    ~TestSuite() {}

    /**
     * @brief  Code here will execute just before the test starts
     *
     */
    void SetUp() override
    {
    }

    /**
     * @brief Code here will execute after the test ends
     *
     */
    void TearDown() override
    {
    }

    const std::string &numberPlate() const { return mPlate; }

    /**
     * @brief Setter for mFailUnique
     *
     */
    void failUnique()
    {
        mFailUnique = true;
    }

    /**
     * @brief Setter for mFailService
     *
     */
    void failService()
    {
        mFailService = true;
    }
};

// Test #1: Check if the robot_pose topic is accurate
TEST(TestSuite, testCase1)
{
    robot_tracker::RobotTracker robot_tracker_;
    ASSERT_EQ(robot_tracker_.robot_pose_sub_.getTopic(), std::string("/robot_pose")) << "Test failed because the topic subscribed to is: " << robot_tracker_.robot_pose_sub_.getTopic() << " instead of /robot_pose";
}

// Test #2: Check if robot position result is invalid
TEST(TestSuite, testCase2)
{
    robot_tracker::RobotTracker robot_tracker_;

    geometry_msgs::Pose invalid_pose;
    invalid_pose.position.x = std::numeric_limits<double>::infinity();
    invalid_pose.position.y = std::numeric_limits<double>::quiet_NaN();

    ASSERT_FALSE(robot_tracker_.isRobotPoseValid(invalid_pose)) << "Test failed to identify malformed robot pose";
}

// Check if a point just within the polygon is considered to be within the polygon
TEST(TestSuite, testCase3)
{
    robot_tracker::RobotTracker robot_tracker_;

    geometry_msgs::Pose example_pose;
    example_pose.position.x = 0.0;
    example_pose.position.y = 0.0;

    // Define an instance of the struct
    robot_tracker::RobotTracker::zone_coordinates zone_instance_;
    zone_instance_.name = "testing_zone";
    zone_instance_.coordinates.push_back(std::make_pair(-1.0, -1.0));
    zone_instance_.coordinates.push_back(std::make_pair(-1.0, 1.0));
    zone_instance_.coordinates.push_back(std::make_pair(1.0, 1.0));
    zone_instance_.coordinates.push_back(std::make_pair(1.0, -1.0));

    robot_tracker_.zone_coordinates_.clear();

    // Push the struct into the container
    robot_tracker_.zone_coordinates_.push_back(zone_instance_);

    // Padding of the zone should take place
    std::string result = robot_tracker_.checkifRobotIsInPolygon(example_pose);
    ASSERT_EQ(result, "testing_zone") << "Test failed to identify point on line of polygon";
}

// Check if a point on the line is considered to be within the polygon
TEST(TestSuite, testCase4)
{
    robot_tracker::RobotTracker robot_tracker_;

    geometry_msgs::Pose example_pose;
    example_pose.position.x = 1.0;
    example_pose.position.y = 1.0;

    // Define an instance of the struct
    robot_tracker::RobotTracker::zone_coordinates zone_instance_;
    zone_instance_.name = "testing_zone";
    zone_instance_.coordinates.push_back(std::make_pair(-1.0, -1.0));
    zone_instance_.coordinates.push_back(std::make_pair(1.0, -1.0));
    zone_instance_.coordinates.push_back(std::make_pair(1.0, 1.0));
    zone_instance_.coordinates.push_back(std::make_pair(-1.0, 1.0));

    // Push the struct into the container
    robot_tracker_.zone_coordinates_.clear();
    robot_tracker_.zone_coordinates_.push_back(zone_instance_);

    // Padding of the zone should take place
    std::string result = robot_tracker_.checkifRobotIsInPolygon(example_pose);
    EXPECT_EQ(result, "") << "Test managed to identify a point on the line, the algorithm should not work this way, but no harm no foul";
}

// Check if a point within a convulated polygon is counted (it would not be)
TEST(TestSuite, testCase5)
{
    robot_tracker::RobotTracker robot_tracker_;

    geometry_msgs::Pose invalid_pose;
    invalid_pose.position.x = std::numeric_limits<double>::infinity();
    invalid_pose.position.y = std::numeric_limits<double>::quiet_NaN();

    ASSERT_FALSE(robot_tracker_.isRobotPoseValid(invalid_pose)) << "Test failed to identify malformed robot pose";
}

// Check if the order of drawing the polygon matters
TEST(TestSuite, testCase6)
{
    geometry_msgs::Pose example_pose;
    example_pose.position.x = 0.0;
    example_pose.position.y = 0.0;
    robot_tracker::RobotTracker robot_tracker_;
    std::string result;

    // ======================================================================================
    // Bottom left first
    // ======================================================================================
    {
        robot_tracker_.zone_coordinates_.clear();
        // Define an instance of the struct
        robot_tracker::RobotTracker::zone_coordinates zone_instance_;
        zone_instance_.name = "testing_zone";
        zone_instance_.coordinates.push_back(std::make_pair(-1.0, -1.0));
        zone_instance_.coordinates.push_back(std::make_pair(-1.0, 1.0));
        zone_instance_.coordinates.push_back(std::make_pair(1.0, 1.0));
        zone_instance_.coordinates.push_back(std::make_pair(1.0, -1.0));

        // Push the struct into the container
        robot_tracker_.zone_coordinates_.clear();
        robot_tracker_.zone_coordinates_.push_back(zone_instance_);

        // Padding of the zone should take place
        std::string result = robot_tracker_.checkifRobotIsInPolygon(example_pose);
        EXPECT_EQ(result, "testing_zone") << "Test managed to identify a point in the polygon despite being drawn precariously";
    }

    // ======================================================================================
    // Top left first
    // ======================================================================================
    {
        robot_tracker_.zone_coordinates_.clear();
        // Define an instance of the struct
        robot_tracker::RobotTracker::zone_coordinates zone_instance_;
        zone_instance_.name = "testing_zone";
        zone_instance_.coordinates.push_back(std::make_pair(-1.0, 1.0));
        zone_instance_.coordinates.push_back(std::make_pair(1.0, 1.0));
        zone_instance_.coordinates.push_back(std::make_pair(1.0, -1.0));
        zone_instance_.coordinates.push_back(std::make_pair(-1.0, -1.0));

        // Push the struct into the container
        robot_tracker_.zone_coordinates_.clear();
        robot_tracker_.zone_coordinates_.push_back(zone_instance_);

        // Padding of the zone should take place
        result = robot_tracker_.checkifRobotIsInPolygon(example_pose);
        EXPECT_EQ(result, "testing_zone") << "Test managed to identify a point in the polygon despite being drawn precariously";
    }

    // ======================================================================================
    // Top right first
    // ======================================================================================
    {
        robot_tracker_.zone_coordinates_.clear();
        // Define an instance of the struct
        robot_tracker::RobotTracker::zone_coordinates zone_instance_;
        zone_instance_.name = "testing_zone";
        zone_instance_.coordinates.push_back(std::make_pair(1.0, 1.0));
        zone_instance_.coordinates.push_back(std::make_pair(1.0, -1.0));
        zone_instance_.coordinates.push_back(std::make_pair(-1.0, -1.0));
        zone_instance_.coordinates.push_back(std::make_pair(-1.0, 1.0));

        // Push the struct into the container
        robot_tracker_.zone_coordinates_.clear();
        robot_tracker_.zone_coordinates_.push_back(zone_instance_);

        // Padding of the zone should take place
        result = robot_tracker_.checkifRobotIsInPolygon(example_pose);
        EXPECT_EQ(result, "testing_zone") << "Test managed to identify a point in the polygon despite being drawn precariously";
    }

    // ======================================================================================
    // Bottom right first
    // ======================================================================================
    {
        robot_tracker_.zone_coordinates_.clear();
        // Define an instance of the struct
        robot_tracker::RobotTracker::zone_coordinates zone_instance_;
        zone_instance_.name = "testing_zone";
        zone_instance_.coordinates.push_back(std::make_pair(1.0, -1.0));
        zone_instance_.coordinates.push_back(std::make_pair(-1.0, -1.0));
        zone_instance_.coordinates.push_back(std::make_pair(-1.0, 1.0));
        zone_instance_.coordinates.push_back(std::make_pair(1.0, 1.0));

        // Push the struct into the container
        robot_tracker_.zone_coordinates_.push_back(zone_instance_);

        // Padding of the zone should take place
        result = robot_tracker_.checkifRobotIsInPolygon(example_pose);
        EXPECT_EQ(result, "testing_zone") << "Test managed to identify a point in the polygon despite being drawn precariously";
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    // Initialise the ROS Stuff
    ros::init(argc, argv, "robot_tracker_tester");
    ros::NodeHandle nh;

    int result = RUN_ALL_TESTS();

    ros::shutdown();
    return result;
}
