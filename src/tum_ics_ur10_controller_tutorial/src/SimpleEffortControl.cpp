#include <tum_ics_ur10_controller_tutorial/SimpleEffortControl.h>

#include <tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        SimpleEffortControl::SimpleEffortControl(double weight, const QString &name)
                : ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
                  m_startFlag(false),
                  m_Kp(Matrix6d::Zero()),
                  m_Kd(Matrix6d::Zero()),
                  m_Ki(Matrix6d::Zero()),
                  m_goal(Vector6d::Zero()),
                  m_totalTime(100.0),
                  m_DeltaQ(Vector6d::Zero()),
                  m_DeltaQp(Vector6d::Zero()),
                  m_ur10_model(ur::UR10Model("ur10_model")) {
            pubCtrlData = n.advertise<tum_ics_ur_robot_msgs::ControlData>(
                    "SimpleEffortCtrlData", 100);

            m_controlPeriod = 0.002;

            ROS_INFO_STREAM("SimpleEffortCtrl Control Period: " << m_controlPeriod);
        }

        SimpleEffortControl::~SimpleEffortControl() {}

        void SimpleEffortControl::setQInit(const JointState &qinit) {
            m_qInit = qinit;
        }

        void SimpleEffortControl::setQHome(const JointState &qhome) {
            m_qHome = qhome;
        }

        void SimpleEffortControl::setQPark(const JointState &qpark) {
            m_qPark = qpark;
        }

        bool SimpleEffortControl::init() {
            ROS_WARN_STREAM("SimpleEffortControl::init");

            std::vector<double> vec;

            // check namespace
            std::string ns = "~simple_effort_ctrl";
            if (!ros::param::has(ns)) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): Control gains not defined in:" << ns);
                m_error = true;
                return false;
            }

            // D GAINS
            ros::param::get(ns + "/gains_d", vec);
            if (vec.size() < STD_DOF) {
                ROS_ERROR_STREAM("gains_d: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < STD_DOF; i++) {
                m_Kd(i, i) = vec[i];
            }
            ROS_WARN_STREAM("Kd: \n" << m_Kd);

            // P GAINS
            ros::param::get(ns + "/gains_p", vec);
            if (vec.size() < STD_DOF) {
                ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < STD_DOF; i++) {
                m_Kp(i, i) = vec[i] / m_Kd(i, i);
            }
            ROS_WARN_STREAM("Kp: \n" << m_Kp);

            // I GAINS
            ros::param::get(ns + "/gains_i", vec);
            if (vec.size() < STD_DOF) {
                ROS_ERROR_STREAM("gains_i: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < STD_DOF; i++) {
                m_Ki(i, i) = vec[i];
            }
            ROS_WARN_STREAM("Ki: \n" << m_Ki);
            

            // GOAL
            ros::param::get(ns + "/goal1", vec);
            if (vec.size() < STD_DOF) {
                ROS_ERROR_STREAM("goal: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < STD_DOF; i++) {
                m_goal(i) = vec[i];
            }

            // total time
            ros::param::get(ns + "/time", m_totalTime);
            if (!(m_totalTime > 0)) {
                ROS_ERROR_STREAM("m_totalTime: is negative:" << m_totalTime);
                m_totalTime = 100.0;
            }

            ROS_WARN_STREAM("Goal [DEG]: \n" << m_goal.transpose());
            ROS_WARN_STREAM("Total Time [s]: " << m_totalTime);
            m_goal = DEG2RAD(m_goal);
            ROS_WARN_STREAM("Goal [RAD]: \n" << m_goal.transpose());


            // initalize the adaptive robot model parameters
            if(!m_ur10_model.initRequest(n))
            {
                ROS_ERROR_STREAM("Error initalizing model");
                m_error = true;
                return false;
            }

            // call functions
            ow::VectorDof q, qP, qrP, qrPP;
            q << 2.82, -2.03, -1.43, -0.67, -1.0, 0.16;
            qP.setZero();
            qrP.setZero();
            qrPP.setZero();

            ur::UR10Model::Regressor Y = m_ur10_model.regressor(q, qP, qrP, qrPP);

            ur::UR10Model::Parameters th = m_ur10_model.parameterInitalGuess(); 

            Eigen::Affine3d T_ef_0 = m_ur10_model.T_ef_0(q);

            Eigen::Matrix<double,6,6> J_ef_0 = m_ur10_model.J_ef_0(q);

            // print something
            ROS_WARN_STREAM("T_ef_0=\n" << T_ef_0.matrix());
            ROS_WARN_STREAM("J_ef_0=\n" << J_ef_0);

            return true;
        }

        bool SimpleEffortControl::start() {
            ROS_WARN_STREAM("SimpleEffortControl::start");
            return true;
        }

        Vector6d SimpleEffortControl::update(const RobotTime &time,
                                             const JointState &current) {
            if (!m_startFlag) {
                m_qStart = current.q;
                ROS_WARN_STREAM("START [DEG]: \n" << m_qStart.transpose());
                m_startFlag = true;
            }

            // control torque
            Vector6d tau;
            tau.setZero();

            // poly spline
            VVector6d vQd;
            vQd = getJointPVT5(m_qStart, m_goal, time.tD(), m_totalTime);

            // erros
            m_DeltaQ = current.q - vQd[0];
            m_DeltaQp = current.qp - vQd[1];

            // reference
            JointState js_r;
            js_r = current;
            js_r.qp = vQd[1] - m_Kp * m_DeltaQ;
            js_r.qpp = vQd[2] - m_Kp * m_DeltaQp;

            // torque
            Vector6d Sq = current.qp - js_r.qp;
            tau = -m_Kd * Sq;
            // tau << 1.0, 50.0, 30.0, 0.5, 0.5, 0.5;

            // publish the ControlData (only for debugging)
            tum_ics_ur_robot_msgs::ControlData msg;
            msg.header.stamp = ros::Time::now();
            msg.time = time.tD();
            for (int i = 0; i < STD_DOF; i++) {
                msg.q[i] = current.q(i);
                msg.qp[i] = current.qp(i);
                msg.qpp[i] = current.qpp(i);

                msg.qd[i] = vQd[0](i);
                msg.qpd[i] = vQd[1](i);

                msg.Dq[i] = m_DeltaQ(i);
                msg.Dqp[i] = m_DeltaQp(i);

                msg.torques[i] = current.tau(i);
            }
            pubCtrlData.publish(msg);

            // ROS_WARN_STREAM("tau=" << tau.transpose());
            return tau;
        }

        bool SimpleEffortControl::stop() {
            return true;
        }

    }  // namespace RobotControllers
}  // namespace tum_ics_ur_robot_lli
