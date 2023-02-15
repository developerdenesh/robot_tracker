#include <robot_tracker/robot_tracker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tracker");
    robot_tracker::RobotTracker robot_tracker_;
    ros::spin();
    return (0);
}
