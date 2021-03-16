#ifndef UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
#define UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <tum_ics_ur10_controller_tutorial/ControlTaskStateMachine.h>
#include <ur10_robot_model/model_ur10.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_msgs/Objects.h>
#include <nav_msgs/Path.h>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        class SimpleEffortControl : public ControlEffort {
        private:
            bool m_startFlag;

            double m_totalTime;
            double m_NonSingTime;
            Vector6d m_qNonSing;

            Vector3d m_circ_traj_center;
            Vector3d m_circ_traj_radius;
            Vector3d m_circ_traj_phase_shift;
            Vector3d m_circ_traj_frequency;

            Vector6d m_qStart;
            Vector6d m_qCurrent;
            Vector6d m_qGoal;

            Vector6d m_xStart;
            Vector6d m_xCurrent;
            Vector6d m_xGoal;

            JointState m_qInit;
            JointState m_qHome;
            JointState m_qPark;

            ros::NodeHandle n;
            ros::Publisher pubCtrlData;
            ros::Publisher pubTrajMarker;
            ros::Publisher pubCartPath;
            ros::Publisher pubEFPath;

            visualization_msgs::MarkerArray m_marker_array;
            nav_msgs::Path path_desired_msg;
            nav_msgs::Path path_ef_msg;

            double m_last_time;
            size_t m_path_publish_ctr;

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
            bool loadControllerGains(std::string ns, Matrix6d &p_Kd, Matrix6d &p_Kp, Matrix6d &p_Ki);

            bool loadConfigParameters();

            bool init();

            bool start();


            Vector6d tau(const RobotTime &time,
                         const JointState &current_js,
                         const Vector6d &vQXrp,
                         const Vector6d &vQXrpp,
                         const Vector6d &vQXp);

            Vector6d antiWindUp(Vector6d tau);

            Vector6d tf2pose(ow::HomogeneousTransformation T);

            VVector3d sinusoid_traj_gen(const Vector3d amp, const Vector3d w, const Vector3d phase_shift, const Vector3d zero_offset, double time);

            void publishPathes(const JointState &current_js, const Vector6d &Qdes, const Vector6d &Xdes, double current_time, double update_hz, int max_path_size);

            Vector6d update(const RobotTime &time, const JointState &current);

            bool stop();
        };

    } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_SIMPLEEFFORTCONTROL_H
