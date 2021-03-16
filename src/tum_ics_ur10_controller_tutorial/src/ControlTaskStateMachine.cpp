#include <tum_ics_ur10_controller_tutorial/ControlTaskStateMachine.h>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        ControlTaskStateMachine::ControlTaskStateMachine() :
                task(ControlTask::MOVE_OUT_SINGULARITY),
                control_mode(ControlMode::JS),
                operational_control(false),
                reg_pose(Vector6d::Zero()),
                t_start(0.0),
                t_end(0.0),
                m_Kp(Matrix6d::Identity()),
                m_Kd(Matrix6d::Identity()),
                m_Ki(Matrix6d::Zero()),
                m_JS_Kp(Matrix6d::Identity()),
                m_JS_Kd(Matrix6d::Identity()),
                m_JS_Ki(Matrix6d::Zero()),
                m_CS_Kp(Matrix6d::Identity()),
                m_CS_Kd(Matrix6d::Identity()),
                m_CS_Ki(Matrix6d::Zero()),
                m_IM_Kp(Matrix6d::Identity()),
                m_IM_Kd(Matrix6d::Identity()),
                m_IM_Ki(Matrix6d::Zero()),
                m_MX_Kp(Matrix6d::Identity()),
                m_MX_Kd(Matrix6d::Identity()),
                m_MX_Ki(Matrix6d::Zero()) {
            ROS_INFO_STREAM("Control Task State Machine created");
        }

        ControlTaskStateMachine::~ControlTaskStateMachine() {
            ROS_INFO_STREAM("Control Task State Machine deleted");
        }

        void ControlTaskStateMachine::initControllerGains(ControlMode cm,
                                                          const Matrix6d &Kd,
                                                          const Matrix6d &Kp,
                                                          const Matrix6d &Ki) {
            switch (cm) {
                case ControlMode::JS: {
                    m_JS_Kp = Kp;
                    m_JS_Kd = Kd;
                    m_JS_Ki = Ki;
                }
                    break;

                case ControlMode::CS: {
                    m_CS_Kp = Kp;
                    m_CS_Kd = Kd;
                    m_CS_Ki = Ki;
                }
                    break;

                case ControlMode::MIXED: {
                    m_MX_Kp = Kp;
                    m_MX_Kd = Kd;
                    m_MX_Ki = Ki;
                }
                    break;

                case ControlMode::IMPEDANCE: {
                    m_IM_Kp = Kp;
                    m_IM_Kd = Kd;
                    m_IM_Ki = Ki;
                }
                    break;

                default:
                    ROS_ERROR_STREAM("[initControllerGains]: no such control mode exists!");
            }
        }

        Matrix6d ControlTaskStateMachine::getKp() {
            return m_Kp;
        }

        Matrix6d ControlTaskStateMachine::getKp(ControlMode cm){
            switch (cm)
            {
            case ControlMode::JS:
                return m_JS_Kp;
            case ControlMode::CS:
                return m_CS_Kp;
            case ControlMode::MIXED:
                return m_MX_Kp;
            case ControlMode::IMPEDANCE:
                return m_IM_Kp;
            default:
                return m_Kp;
            }
        }

        Matrix6d ControlTaskStateMachine::getKpIM() {
            return getKp(ControlMode::IMPEDANCE);
        }

        Matrix6d ControlTaskStateMachine::getKpCS() {
            return getKp(ControlMode::CS);
        }


        Matrix6d ControlTaskStateMachine::getKd() {
            return m_Kd;
        }

        Matrix6d ControlTaskStateMachine::getKd(ControlMode cm){
            switch (cm)
            {
            case ControlMode::JS:
                return m_JS_Kd;
            case ControlMode::CS:
                return m_CS_Kd;
            case ControlMode::MIXED:
                return m_MX_Kd;
            case ControlMode::IMPEDANCE:
                return m_IM_Kd;
            default:
                return m_Kd;
            }
        }

        Matrix6d ControlTaskStateMachine::getKdIM() {
            return getKd(ControlMode::IMPEDANCE);
        }

        Matrix6d ControlTaskStateMachine::getKdCS() {
            return getKd(ControlMode::CS);
        }

        Matrix6d ControlTaskStateMachine::getKi() {
            return m_Ki;
        }

        bool ControlTaskStateMachine::isRunning(double current_time) {
            return manouverTime(current_time) < t_end;
        }

        double ControlTaskStateMachine::manouverTime(double current_time) {
            return current_time - t_start;
        }

        double ControlTaskStateMachine::getTaskTime() {
            return t_end;
        }

        ControlTask ControlTaskStateMachine::getCurrentTask() {
            return task;
        }

        void ControlTaskStateMachine::changeTask(ControlTask next_task,
                                                 double task_time,
                                                 double current_time) {
            t_start = current_time;
            t_end = task_time;
            task = next_task;

            switch (task) {
                case ControlTask::BREAK: {
                    ROS_INFO_STREAM("Emergency breaking!");
                    control_mode = ControlMode::JS;
                    operational_control = false;
                }
                    break;

                case ControlTask::MOVE_OUT_SINGULARITY: {
                    ROS_INFO_STREAM("Move out singularity with joint space control");
                    control_mode = ControlMode::JS;
                    operational_control = false;
                }
                    break;

                case ControlTask::MOVE_DOWN_AND_ROTATE_UPWARDS: {
                    ROS_INFO_STREAM("Move down and rotate end-effector upwards with cartesian space control");
                    control_mode = ControlMode::CS;
                    operational_control = true;
                }
                    break;

                case ControlTask::MOVE_IN_CIRCLE_POINT_UPWARDS: {
                    ROS_INFO_STREAM("Moving in a circle trajectory and pointing upwards with cartesian space control");
                    control_mode = ControlMode::CS;
                    operational_control = true;
                }
                    break;

                case ControlTask::OBSTACLE_AVOIDANCE: {
                    ROS_INFO_STREAM("Obstacle avoidance!");
                    // TODO OR MIXED?
                    control_mode = ControlMode::IMPEDANCE;
                    operational_control = true;

                }
                    break;

                default:
                    ROS_ERROR_STREAM("Unknown ControlTask state!");
            }

            // set the control gains for the current mode
            switchControlGains();

        }

        void ControlTaskStateMachine::switchControlGains() {
            switch (getControlMode()) {

                case ControlMode::JS: {
                    m_Kp = m_JS_Kp;
                    m_Kd = m_JS_Kd;
                    m_Ki = m_JS_Ki;
                }
                    break;

                case ControlMode::CS: {
                    m_Kp = m_CS_Kp;
                    m_Kd = m_CS_Kd;
                    m_Ki = m_CS_Ki;
                }
                    break;

                case ControlMode::MIXED: {
                    m_Kp = m_MX_Kp;
                    m_Kd = m_MX_Kd;
                    m_Ki = m_MX_Ki;
                }
                    break;

                case ControlMode::IMPEDANCE: {
                    m_Kp = m_IM_Kp;
                    m_Kd = m_IM_Kd;
                    m_Ki = m_IM_Ki;
                }
                    break;

                default:
                    ROS_ERROR_STREAM("[switchControlGains]: no such control mode exists!");
            }
        }

        ControlMode ControlTaskStateMachine::getControlMode() {
            return control_mode;
        }

        void ControlTaskStateMachine::setControlMode(ControlMode new_control_mode) {
            control_mode = new_control_mode;
        }

    }  // namespace RobotControllers
}  // namespace tum_ics_ur_robot_lli