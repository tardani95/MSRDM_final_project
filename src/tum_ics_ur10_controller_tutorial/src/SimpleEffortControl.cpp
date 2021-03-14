#include <tum_ics_ur10_controller_tutorial/SimpleEffortControl.h>

#include <tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        SimpleEffortControl::SimpleEffortControl(double weight, const QString &name)
                : ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
                  m_startFlag(false),
                  m_startFlag2(false),
                  m_Kp(Matrix6d::Zero()),
                  m_Kd(Matrix6d::Zero()),
                  m_Ki(Matrix6d::Zero()),
                  m_goal(Vector6d::Zero()),
                  m_totalTime(100.0),
                  m_DeltaQ(Vector6d::Zero()),
                  m_DeltaQp(Vector6d::Zero()),
                  m_sumDeltaQ(Vector6d::Zero()),
                  m_sumDeltaQp(Vector6d::Zero()),
                  m_control_mode(ControlMode::JS),
                  m_ur10_model(ur::UR10Model("ur10_model")) {
            
            pubCtrlData = n.advertise<tum_ics_ur_robot_msgs::ControlData>(
                    "SimpleEffortCtrlData", 100);

            m_theta = m_ur10_model.parameterInitalGuess();

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

            return true;
        }

        bool SimpleEffortControl::start() {
            ROS_WARN_STREAM("SimpleEffortControl::start");
            return true;
        }

        Vector6d SimpleEffortControl::tf2pose(ow::HomogeneousTransformation T){
            Vector4d zero4d;
            zero4d.setZero();
            zero4d[3] = 1.0;
            Vector3d vX = (T * zero4d).head(3);
            Matrix3d rot_mat = T.matrix().topLeftCorner(3, 3);
            Vector3d euler_angles = rot_mat.eulerAngles(2, 1, 0);
            Vector6d pose;
            pose.topRows(3) = vX;
            pose.bottomRows(3) = euler_angles;

            return pose;
        }

        Vector6d SimpleEffortControl::tau(const RobotTime &time, 
                                          const JointState &current_js,
                                          const Vector6d &t_QXrp,
                                          const Vector6d &t_QXrpp,
                                          const Vector6d &t_QXp){
            // control torque
            Vector6d tau;
            tau.setZero();


            // robot model
            Vector6d QXrp, QXrpp, QXp, Q, Qp;
            
            QXrp = t_QXrp;
            QXrpp = t_QXrpp;

            QXp = t_QXp;

            Q = current_js.q;
            Qp = current_js.qp;

            ur::UR10Model::Regressor Yr;
            
            Vector6d Sq, Sqx;

            // controller 
            Sqx = QXp - QXrp;

            Vector6d Qrp, Qrpp;
            Eigen::Matrix<double,6,6> J_ef_0;
            PartialPivLU<Tum::Matrix6d> J_ef_0_lu;

            switch (m_control_mode)
            {
            case ControlMode::JS: /* joint space */
                Sq = Sqx;
                Qrp = QXrp;
                Qrpp = QXrpp;
                Yr = m_ur10_model.regressor(Q, Qp, Qrp, Qrpp);
                break;
            
            case ControlMode::CS: /* cartesian space */
                J_ef_0 = m_ur10_model.J_ef_0(current_js.q);
                // Eigen::Matrix<double,3,6> J_ef_0_v = J_ef_0.topRows(3);
                ROS_WARN_STREAM("Jef_0=\n" << J_ef_0);
                J_ef_0_lu = J_ef_0.lu();
                
                Sq = J_ef_0_lu.solve(Sqx);
                Qrp = J_ef_0_lu.solve(QXrp);
                Qrpp = J_ef_0_lu.solve(QXrpp);
                ROS_WARN_STREAM("Sq=\n" << Sq);
                ROS_WARN_STREAM("Qrp=\n" << Qrp);
                ROS_WARN_STREAM("Qrpp=\n" << Qrpp);

                Yr = m_ur10_model.regressor(Q, Qp, Qrp, Qrpp);
                break;

            default:
                ROS_WARN_STREAM("Using DEFAULT control mode: Joint Space");
                Sq = Sqx;
                Qrp = QXrp;
                Qrpp = QXrpp;
                Yr = m_ur10_model.regressor(Q, Qp, Qrp, Qrpp);
            }

            // ROS_WARN_STREAM("Sq =\n" << Sq);
            // calculate torque
            tau = -m_Kd*Sq + Yr*m_theta;

            // parameter update
            MatrixXd gamma = MatrixXd::Identity(81,81);
            gamma *= 0.0002;
            // parameter vector update
            m_theta -= gamma * Yr.transpose() * Sq;

            return tau;
        }

        Vector6d SimpleEffortControl::update(const RobotTime &time,
                                             const JointState &current) {

            if (!m_startFlag) {
                m_qStart = current.q;
                ROS_WARN_STREAM("START [DEG]: \n" << m_qStart.transpose());
                m_startFlag = true;
                m_control_mode = ControlMode::JS;
            }else if(!m_startFlag2 && time.tD() > m_totalTime) {
                m_startFlag2 = true;
                m_sumDeltaQ.setZero();
                m_sumDeltaQp.setZero();
                m_xStart = tf2pose(m_ur10_model.T_ef_0(current.q));
                m_xGoal = m_xStart;
                m_xGoal[2] = 0.15;
                m_xGoal.tail(3) << 2.1715, -1.5, 0.9643;
                m_control_mode = ControlMode::CS;

                ROS_WARN_STREAM("Desired polyline start= \n" << m_xStart);
                ROS_WARN_STREAM("Desired polyline end= \n" << m_xGoal);
            }

            // control torque
            Vector6d tau;
            tau.setZero();

            // poly spline
            VVectorDOFd vQXd;

            // time
            double ellapsed_time = time.tD();

            
            if (m_startFlag && ellapsed_time < m_totalTime){
                
                vQXd = getJointPVT5(m_qStart, m_goal, time.tD(), m_totalTime);
                
            }else if (m_startFlag2){
                Vector6d q_goal;
                q_goal << 0.0, -1, 2.16, -2.7, -4.8, 0; 
                ROS_WARN_STREAM("Goal pose from function =\n " << tf2pose(m_ur10_model.T_ef_0(q_goal)));
                // m_xGoal << 0.683465, 0.17, 0.2, 2.1715, -1.5, 0.9643;
                vQXd = getJointPVT5(m_xStart, m_xGoal, time.tD()-m_totalTime, m_totalTime);
                ROS_WARN_STREAM("Desired polyline\nStart: \n" << m_xStart << "\nGoal: \n" << m_xGoal);
                ROS_WARN_STREAM("Desired polyline\nQXd: \n" << vQXd[0] << "\nQXdp: \n" << vQXd[1]);
            }

            VectorDOFd QXd, QXdp, QXdpp; // desired qd,qdp,qdpp or xd,xdp,xdpp
            QXd = vQXd[0];
            QXdp = vQXd[1];
            QXdpp = vQXd[2];
            
            VectorDOFd QX, QXp; // current q or x

            switch(m_control_mode){
                case ControlMode::JS:
                    QX = current.q;
                    QXp = current.qp;

                    break;
                case ControlMode::CS:
                    QX = tf2pose(m_ur10_model.T_ef_0(current.q));
                    QXp = m_ur10_model.J_ef_0(current.q) * current.qp;

                    break;
                default:
                    ROS_ERROR_STREAM("bad control mode");
                    break;
            }

            // erros
            m_DeltaQ = QX - QXd;
            m_DeltaQp = QXp - QXdp;

            m_sumDeltaQ += m_DeltaQ * m_controlPeriod;
            m_sumDeltaQp += m_DeltaQp * m_controlPeriod;

            VectorDOFd QXrp, QXrpp; // reference qr or xr
            QXrp = QXdp - m_Kp * m_DeltaQ - m_Ki * m_sumDeltaQ;
            QXrpp = QXdpp - m_Kp * m_DeltaQp - m_Ki * m_sumDeltaQp;

            // // reference
            // JointState js_r;
            // js_r = current;
            // js_r.qp = vQXd[1] - m_Kp * m_DeltaQ - m_Ki * m_sumDeltaQ;
            // js_r.qpp = vQXd[2] - m_Kp * m_DeltaQp - m_Ki * m_sumDeltaQp;

            // torque calculation
            tau = SimpleEffortControl::tau(time, current, QXrp, QXrpp, QXp);
            ROS_WARN_STREAM("tau=" << tau.transpose());

            Vector6d max_control_effort;
            max_control_effort << 330, 330, 150, 54, 54, 54;

            for(size_t idx = 0; idx<6; idx++){
                double tau_idx = tau[idx];
                double max_tau_idx = max_control_effort[idx]; 
                if(tau_idx > max_tau_idx){
                    tau[idx] = max_tau_idx;
                    // switch on anti-windup
                    ROS_WARN_STREAM("anti-windup on");
                }
            }

            ROS_WARN_STREAM("tau=" << tau.transpose());

            // publish the ControlData (only for debugging)
            tum_ics_ur_robot_msgs::ControlData msg;
            msg.header.stamp = ros::Time::now();
            msg.time = time.tD();
            for (int i = 0; i < STD_DOF; i++) {
                msg.q[i] = current.q(i);
                msg.qp[i] = current.qp(i);
                msg.qpp[i] = current.qpp(i);

                msg.qd[i] = vQXd[0](i);
                msg.qpd[i] = vQXd[1](i);

                msg.Dq[i] = m_DeltaQ(i);
                msg.Dqp[i] = m_DeltaQp(i);

                msg.torques[i] = current.tau(i);
            }
            pubCtrlData.publish(msg);


            return tau;
        }

        bool SimpleEffortControl::stop() {
            return true;
        }

    }  // namespace RobotControllers
}  // namespace tum_ics_ur_robot_lli
