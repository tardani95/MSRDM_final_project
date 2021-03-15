#ifndef UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
#define UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <tum_ics_ur10_controller_tutorial/ControlTaskStateMachine.h>
#include <ur10_robot_model/model_ur10.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_msgs/Objects.h>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

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
            ros::Publisher pubTrajMarker;

            visualization_msgs::MarkerArray m_marker_array;

            Vector3d m_target_pos;
            static const size_t m_max_num_obstacles = 4;
            size_t m_num_obstacles;
            VVector3d m_vObstacles_pos_0;
            Vector3d m_ef_traj_xd;

            ur::UR10Model m_ur10_model;
            MatrixXd m_theta;

            Matrix6d m_Kp;
            Matrix6d m_Kd;
            Matrix6d m_Ki;

            Matrix6d m_JS_Kp;
            Matrix6d m_JS_Kd;
            Matrix6d m_JS_Ki;

            Matrix6d m_CS_Kp;
            Matrix6d m_CS_Kd;
            Matrix6d m_CS_Ki;
            
            Vector6d m_goal;
            double m_totalTime;

            Vector6d m_DeltaQ;
            Vector6d m_DeltaQp;

            Vector6d m_anti_windup;

            Vector6d m_sumDeltaQ;
            Vector6d m_sumDeltaQp;

            ControlTaskStateMachine m_ct_sm;

            const Vector6d c_max_control_effort;
            double m_controlPeriod; //[s]

        public:
            SimpleEffortControl(double weight = 1.0, const QString &name = "SimpleEffortCtrl");

            ~SimpleEffortControl();

            void setQInit(const JointState &qinit);

            void setQHome(const JointState &qhome);

            void setQPark(const JointState &qpark);

            void obstaclesPositionUpdateCallback(const object_msgs::Objects msg);
            void targetPositionUpdateCallback(const object_msgs::Objects msg);

        private:
            bool initControllerGains(std::string ns, Matrix6d& p_Kd, Matrix6d& p_Kp, Matrix6d& p_Ki);

            bool init();

            bool start();

            
            Vector6d tau(const RobotTime &time, 
                                          const JointState &current_js,
                                          const Vector6d &vQXrp,
                                          const Vector6d &vQXrpp,
                                          const Vector6d &vQXp);

            Vector6d antiWindUp(Vector6d tau);

            Vector6d tf2pose(ow::HomogeneousTransformation T);

            Vector6d update(const RobotTime &time, const JointState &current);

            bool stop();
        };

    } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
