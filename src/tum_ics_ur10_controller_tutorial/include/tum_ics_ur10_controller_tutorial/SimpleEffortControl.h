#ifndef UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
#define UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <ur10_robot_model/model_ur10.h>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        enum ControlMode{
            JS = 0,
            CS,
            MIXED,
            IMPEDANCE,
            UNKNOWN = -1,
        };

        enum ControlTask{
            MOVE_OUT_SINGULARITY = 0,
            MOVE_DOWN_AND_POINT_UPWARDS,
            MOVE_IN_CIRCLE,
        };

        class SimpleEffortControl : public ControlEffort {
        private:
            bool m_startFlag;
            bool m_startFlag2;

            Vector6d m_qStart;
            Vector6d m_xStart;
            Vector6d m_xGoal;
            JointState m_qInit;
            JointState m_qHome;
            JointState m_qPark;

            ros::NodeHandle n;
            ros::Publisher pubCtrlData;

            ur::UR10Model m_ur10_model;
            MatrixXd m_theta;

            Matrix6d m_Kp;
            Matrix6d m_Kd;
            Matrix6d m_Ki;
            Vector6d m_goal;
            double m_totalTime;

            Vector6d m_DeltaQ;
            Vector6d m_DeltaQp;

            Vector6d m_sumDeltaQ;
            Vector6d m_sumDeltaQp;

            ControlMode m_control_mode;

            double m_controlPeriod; //[s]

        public:
            SimpleEffortControl(double weight = 1.0, const QString &name = "SimpleEffortCtrl");

            ~SimpleEffortControl();

            void setQInit(const JointState &qinit);

            void setQHome(const JointState &qhome);

            void setQPark(const JointState &qpark);

        private:
            bool init();

            bool start();

            Vector6d tau(const RobotTime &time, 
                                          const JointState &current_js,
                                          const Vector6d &vQXrp,
                                          const Vector6d &vQXrpp,
                                          const Vector6d &vQXp);

            Vector6d tf2pose(ow::HomogeneousTransformation T);

            Vector6d update(const RobotTime &time, const JointState &current);

            bool stop();
        };

    } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
