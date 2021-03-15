#ifndef UR_ROBOT_LLI_CONTROLTASKSTATEMACHINE_H
#define UR_ROBOT_LLI_CONTROLTASKSTATEMACHINE_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>

// add namespaces to avoid namespace specifier for Vector6d and etc.
namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        enum ControlMode {
            JS = 0,
            CS,
            MIXED,
            IMPEDANCE,
        };

        enum ControlTask {
            BREAK = 0,
            MOVE_OUT_SINGULARITY,
            MOVE_DOWN_AND_ROTATE_UPWARDS,
            MOVE_IN_CIRCLE_POINT_UPWARDS,
            OBSTACLE_AVOIDANCE,
        };


        class ControlTaskStateMachine {

        private:
            ControlTask task;
            ControlMode control_mode;

            bool operational_control;

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

            Matrix6d m_MX_Kp;
            Matrix6d m_MX_Kd;
            Matrix6d m_MX_Ki;

            /* -------------- methods ----------- */
        public:
            ControlTaskStateMachine();

            ~ControlTaskStateMachine();

            void initControllerGains(ControlMode cm, Matrix6d Kp, Matrix6d Kd, Matrix6d Ki);

            Matrix6d getKp();

            Matrix6d getKd();

            Matrix6d getKi();

            bool isRunning(double current_time);

            double manouverTime(double current_time);

            double getTaskTime();

            ControlTask getCurrentTask();

            void changeTask(ControlTask next_task, double task_time, double current_time);

            void switchControlGains();

            ControlMode getControlMode();

            void setControlMode(ControlMode new_control_mode);


        };
    } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
#endif // UR_ROBOT_LLI_CONTROLTASKSTATEMACHINE_H