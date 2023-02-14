#include <robot_tracker/robot_tracker.h>

// Bring in gtest
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
