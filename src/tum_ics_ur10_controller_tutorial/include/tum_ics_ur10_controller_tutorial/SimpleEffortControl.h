#ifndef UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
#define UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <ur10_robot_model/model_ur10.h>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        class SimpleEffortControl : public ControlEffort {
        private:
            bool m_startFlag;

            Vector6d m_qStart;
            JointState m_qInit;
            JointState m_qHome;
            JointState m_qPark;

            ros::NodeHandle n;
            ros::Publisher pubCtrlData;

            ur::UR10Model m_ur10_model;

            Matrix6d m_Kp;
            Matrix6d m_Kd;
            Matrix6d m_Ki;
            Vector6d m_goal;
            double m_totalTime;

            Vector6d m_DeltaQ;
            Vector6d m_DeltaQp;
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

            Vector6d update(const RobotTime &time, const JointState &current);

            bool stop();
        };

    } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
