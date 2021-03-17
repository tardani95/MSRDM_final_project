#ifndef UR_ROBOT_LLI_CONTROLTASKSTATEMACHINE_H
#define UR_ROBOT_LLI_CONTROLTASKSTATEMACHINE_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>

// add namespaces to avoid namespace specifier for Vector6d and etc.
namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        enum ControlMode {
            JS = 0,
            CS,
            IMPEDANCE,
        };

        enum ControlTask {
            BREAK = 0,
            MOVE_OUT_SINGULARITY,
            MOVE_TO_CIRCULAR_TRAJECTORY_START,
            CIRCULAR_EF_TRAJECTORY_TRACKING,
        };


        class ControlTaskStateMachine {

        private:
            ControlTask task;
            ControlMode control_mode;

            bool operational_control;
            bool is_gazing;
            bool obstacle_avoidance_on;

            Vector6d reg_pose;

            double t_start;
            double t_end;

            Matrix6d m_Kp;
            Matrix6d m_Kd;
            Matrix6d m_Ki;

            Matrix6d m_JS_Kp;
            Matrix6d m_JS_Kd;
            Matrix6d m_JS_Ki;

            Matrix6d m_CS_Kp;
            Matrix6d m_CS_Kd;
            Matrix6d m_CS_Ki;

            Matrix6d m_IM_Kp;
            Matrix6d m_IM_Kd;
            Matrix6d m_IM_Ki;

            /* -------------- methods ----------- */
        public:
            ControlTaskStateMachine();

            ~ControlTaskStateMachine();

            void initControllerGains(ControlMode cm, 
                                    const Matrix6d &Kd, 
                                    const Matrix6d &Kp, 
                                    const Matrix6d &Ki);

            Matrix6d getKp();
            Matrix6d getKp(ControlMode cm);
            Matrix6d getKpCS();
            Matrix6d getKpIM();

            Matrix6d getKd();
            Matrix6d getKd(ControlMode cm);
            Matrix6d getKdCS();
            Matrix6d getKdIM();

            Matrix6d getKi();
            Matrix6d getKi(ControlMode cm);


            bool isRunning(double current_time);

            double manouverTime(double current_time);

            double getTaskTime();

            ControlTask getCurrentTask();

            void changeTask(ControlTask next_task, double task_time, double current_time);

            void switchControlGains();

            ControlMode getControlMode();

            void setControlMode(ControlMode new_control_mode);

            void startGazing();

            bool isGazing();

            bool isObstacleAvoidanceOn();

            void setObstacleAvoidance(bool state);

        };
    } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
#endif // UR_ROBOT_LLI_CONTROLTASKSTATEMACHINE_H