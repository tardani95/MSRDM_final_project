#include <tum_ics_ur10_controller_tutorial/ControlTaskStateMachine.h>

namespace tum_ics_ur_robot_lli {
namespace RobotControllers {

    ControlTaskStateMachine::ControlTaskStateMachine() : 
        task(ControlTask::MOVE_OUT_SINGULARITY),
        control_mode(ControlMode::JS),
        operational_control(false),
        reg_pose(Vector6d::Zero()),
        t_start(0.0),
        t_end(0.0) {
        ROS_INFO_STREAM("Control Task State Machine created");
    }

    ControlTaskStateMachine::~ControlTaskStateMachine(){
        ROS_INFO_STREAM("Control Task State Machine deleted");
    }

    bool ControlTaskStateMachine::isRunning(double current_time){
        return manouverTime(current_time) < t_end;
    }

    double ControlTaskStateMachine::manouverTime(double current_time){
        return current_time - t_start;
    }

    double ControlTaskStateMachine::getTaskTime(){
        return t_end;
    }

    ControlTask ControlTaskStateMachine::getCurrentTask(){
        return task;
    }

    void ControlTaskStateMachine::changeTask(ControlTask next_task, 
                                             double task_time,
                                             double current_time){
        t_start = current_time;
        t_end = task_time;
        task = next_task;

        switch(task){
            case ControlTask::BREAK:{
                ROS_INFO_STREAM("Emergency breaking!");
                control_mode = ControlMode::JS;
                operational_control = false;
            }
            break;

            case ControlTask::MOVE_OUT_SINGULARITY:{
                ROS_INFO_STREAM("Move out singularity with joint space control");
                control_mode = ControlMode::JS;
                operational_control = false;
            }
            break;

            case ControlTask::MOVE_DOWN_AND_ROTATE_UPWARDS:{
                ROS_INFO_STREAM("Move down and rotate end-effector upwards with cartesian space control");
                control_mode = ControlMode::CS;
                operational_control = true;
            }
            break;

            case ControlTask::MOVE_IN_CIRCLE_POINT_UPWARDS:{
                ROS_INFO_STREAM("Moving in a circle trajectory and pointing upwards with cartesian space control");
                control_mode = ControlMode::CS;
                operational_control = true;
            }
            break;

            case ControlTask::OBSTACLE_AVOIDANCE:{
                ROS_INFO_STREAM("Obstacle avoidance!");
                control_mode = ControlMode::IMPEDANCE;
                operational_control = true;

            }
            break;

            default:
                ROS_ERROR_STREAM("Unknown ControlTask state!");
        }
    }

    ControlMode ControlTaskStateMachine::getControlMode(){
        return control_mode;
    }

    void ControlTaskStateMachine::setControlMode(ControlMode new_control_mode){
        control_mode = new_control_mode;
    }

}  // namespace RobotControllers
}  // namespace tum_ics_ur_robot_lli