#include<tum_ics_ur_robot_lli/Robot/RobotArmConstrained.h>
#include<tum_ics_ur10_controller_tutorial/SimpleEffortControl.h>
#include<QApplication>

int main(int argc, char **argv) {
    QApplication a(argc, argv);

    ros::init(argc, argv, "testRobotArmClass", ros::init_options::AnonymousName);
    
    ros::NodeHandle nh;

    QString configFilePath = argv[1];
    ROS_INFO_STREAM("Config File: " << configFilePath.toStdString().c_str());

    // starts robotArm communication and the thread
    tum_ics_ur_robot_lli::Robot::RobotArmConstrained robot(configFilePath);;
    if (!robot.init()) {
        return -1;
    }

    // create controller
    ROS_INFO_STREAM("Create Controller");
    tum_ics_ur_robot_lli::RobotControllers::SimpleEffortControl controller(1.0, "SimpleController");

    //The control must be connected to the robot after the init()-->The dynamic model needs to
    ROS_INFO_STREAM("Add Controller");
    if (!robot.add(&controller)) {
        return -1;
    }
    controller.setQHome(robot.qHome());
    controller.setQPark(robot.qPark());

    ros::Subscriber targetSub = nh.subscribe("target", 10, &tum_ics_ur_robot_lli::RobotControllers::SimpleEffortControl::targetPositionUpdateCallback, &controller);
    ros::Subscriber obstaclesSub = nh.subscribe("obstacles", 10, &tum_ics_ur_robot_lli::RobotControllers::SimpleEffortControl::obstaclesPositionUpdateCallback, &controller);

    // RUN !
    ROS_INFO_STREAM("Start Robot");
    robot.start();
    ROS_INFO_STREAM("Start main thread");
    ros::spin();
    ROS_INFO_STREAM("main: Stoping RobotArm()");
    robot.stop();
    ROS_INFO_STREAM("main: Stopped!!!");

    return 0;
}