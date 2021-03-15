#include <tum_ics_ur10_controller_tutorial/SimpleEffortControl.h>
#include <tum_ics_ur_robot_msgs/ControlData.h>


namespace tum_ics_ur_robot_lli {
    namespace RobotControllers {

        SimpleEffortControl::SimpleEffortControl(double weight, const QString &name)
                : ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
                  m_startFlag(false),
                  m_last_time(0.0),
                  m_path_publish_ctr(0),
                  c_max_control_effort((Vector6d() << 330, 330, 150, 54, 54, 54).finished()),
                  m_Kp(Matrix6d::Zero()),
                  m_Kd(Matrix6d::Zero()),
                  m_Ki(Matrix6d::Zero()),
                  m_JS_Kp(Matrix6d::Zero()),
                  m_JS_Kd(Matrix6d::Zero()),
                  m_JS_Ki(Matrix6d::Zero()),
                  m_CS_Kp(Matrix6d::Zero()),
                  m_CS_Kd(Matrix6d::Zero()),
                  m_CS_Ki(Matrix6d::Zero()),
                  m_totalTime(100.0),
                  m_DeltaQ(Vector6d::Zero()),
                  m_DeltaQp(Vector6d::Zero()),
                  m_anti_windup(Vector6d::Ones()),
                  m_sumDeltaQ(Vector6d::Zero()),
                  m_sumDeltaQp(Vector6d::Zero()),
                  m_ct_sm(ControlTaskStateMachine()),
                  m_ur10_model(ur::UR10Model("ur10_model")) {

            pubCtrlData = n.advertise<tum_ics_ur_robot_msgs::ControlData>(
                    "SimpleEffortCtrlData", 100);

            pubTrajMarker = n.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 100);
            pubCartPath = n.advertise<nav_msgs::Path>("path_cs_des_traj", 10);
            pubEFPath = n.advertise<nav_msgs::Path>("path_ef_traj", 10);

            path_desired_msg.header.frame_id = "dh_arm_joint_0";
            path_ef_msg.header.frame_id = "dh_arm_joint_0";

            m_vObstacles_pos_0.reserve(m_max_num_obstacles);

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

        void SimpleEffortControl::obstaclesPositionUpdateCallback(const object_msgs::Objects msg) {
            // ROS_WARN_STREAM("Objects: \n" << msg);

            // Objects obstacles = msg.objects;
            m_num_obstacles = msg.objects.size();
            geometry_msgs::Point obstacle_position;
            Vector4d tmp_obs_pos = Vector4d::Zero();
            tmp_obs_pos[3] = 1.0;

            for (size_t idx = 0; idx < m_num_obstacles; idx++) {

                obstacle_position = msg.objects[idx].position.position;
                tmp_obs_pos[0] = obstacle_position.x;
                tmp_obs_pos[1] = obstacle_position.y;
                tmp_obs_pos[2] = obstacle_position.z;

                m_vObstacles_pos_0[idx] = (m_ur10_model.T_B_0() * tmp_obs_pos).head(3);
                // ROS_WARN_STREAM("Object "<< idx << " homogen position frame0 [m]: " << m_vObstacles_pos_0[idx].transpose());
            }
        }

        void SimpleEffortControl::targetPositionUpdateCallback(const object_msgs::Objects msg) {
            // ROS_WARN_STREAM("Objects: \n" << msg);
            geometry_msgs::Point target_x = msg.objects[0].position.position;
            m_target_pos[0] = target_x.x;
            m_target_pos[1] = target_x.y;
            m_target_pos[2] = target_x.z;
            // ROS_WARN_STREAM("Target position [m]: " << m_target_pos.transpose());
        }

        bool SimpleEffortControl::loadControllerGains(std::string t_ns, Matrix6d &p_Kd, Matrix6d &p_Kp, Matrix6d &p_Ki) {

            std::vector<double> vec;

            // check namespace
            std::string ns = "~" + t_ns;
            if (!ros::param::has(ns)) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): " << t_ns << " gains not defined in:" << ns);
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
                p_Kd(i, i) = vec[i];
            }
            ROS_WARN_STREAM(t_ns << " Kd: \n" << p_Kd);

            // P GAINS
            ros::param::get(ns + "/gains_p", vec);
            if (vec.size() < STD_DOF) {
                ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < STD_DOF; i++) {
                p_Kp(i, i) = vec[i];
            }
            ROS_WARN_STREAM(t_ns << " Kp: \n" << p_Kp);

            // I GAINS
            ros::param::get(ns + "/gains_i", vec);
            if (vec.size() < STD_DOF) {
                ROS_ERROR_STREAM("gains_i: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (size_t i = 0; i < STD_DOF; i++) {
                p_Ki(i, i) = vec[i];
            }
            ROS_WARN_STREAM(t_ns << " Ki: \n" << p_Ki);

            return true;
        }

        bool SimpleEffortControl::loadConfigParameters() {
            std::vector<double> vec;

            // check namespace
            std::string ns = "~other_parameter_settings";
            if (!ros::param::has(ns)) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): no namespace named " << ns);
                m_error = true;
                return false;
            }

            // total time
            ros::param::get(ns + "/time_total", m_totalTime);
            if (m_totalTime <= 0) {
                ROS_ERROR_STREAM("m_totalTime: is negative:" << m_totalTime);
                m_totalTime = 200.0;
            }

            // non singular joint state
            ros::param::get(ns + "/non_singular_state", vec);
            if (vec.size() < STD_DOF) {
                ROS_ERROR_STREAM("non_singular_state: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < STD_DOF; i++) {
                m_qNonSing(i) = vec[i];
            }

            // time for moving out singularity
            ros::param::get(ns + "/time_move_out_sing_state", m_NonSingTime);
            if (m_NonSingTime <= 0) {
                ROS_ERROR_STREAM("time_move_out_sing_state: is negative:" << m_NonSingTime);
                m_NonSingTime = 15.0;
            }

            std::string ns_circle_traj = "/circle_traj";
            // circular trajectory parameters
            // center
            ros::param::get(ns + ns_circle_traj + "/center", vec);
            if (vec.size() < 3) {
                ROS_ERROR_STREAM("circle_traj/center: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < 3; i++) {
                m_circ_traj_center(i) = vec[i];
            }

            // radius
            ros::param::get(ns + ns_circle_traj + "/radius", vec);
            if (vec.size() < 3) {
                ROS_ERROR_STREAM("circle_traj/radius: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < 3; i++) {
                m_circ_traj_radius(i) = vec[i];
            }

            // phase
            ros::param::get(ns + ns_circle_traj + "/phase", vec);
            if (vec.size() < 3) {
                ROS_ERROR_STREAM("circle_traj/phase: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < 3; i++) {
                m_circ_traj_phase_shift(i) = vec[i];
            }

            // frequency
            ros::param::get(ns + ns_circle_traj + "/frequency", vec);
            if (vec.size() < 3) {
                ROS_ERROR_STREAM("circle_traj/frequency: wrong number of dimensions:" << vec.size());
                m_error = true;
                return false;
            }
            for (int i = 0; i < 3; i++) {
                m_circ_traj_frequency(i) = vec[i];
            }

            return true;
        }

        bool SimpleEffortControl::init() {
            ROS_WARN_STREAM("SimpleEffortControl::init");

            if (!loadControllerGains("joint_space_ctrl", m_JS_Kd, m_JS_Kp, m_JS_Ki)) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): joint_space_ctrl could not be initialized");
                m_error = true;
                return false;
            } else {
                m_ct_sm.initControllerGains(ControlMode::JS, m_JS_Kp, m_JS_Kd, m_JS_Ki);
            }

            if (!loadControllerGains("cartesian_space_ctrl", m_CS_Kd, m_CS_Kp, m_CS_Ki)) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): cartesian_space_ctrl could not be initialized");
                m_error = true;
                return false;
            } else {
                m_ct_sm.initControllerGains(ControlMode::CS, m_CS_Kp, m_CS_Kd, m_CS_Ki);
            }

            if (!loadConfigParameters()) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): config parameters could not be initialized");
                m_error = true;
                return false;
            }

            ROS_WARN_STREAM("Total Time [s]: " << m_totalTime);
            ROS_WARN_STREAM("Time for moving out singularity [s]: " << m_NonSingTime);

            ROS_WARN_STREAM("Non-Singular Joint State [DEG]: " << m_qNonSing.transpose());
            m_qNonSing = DEG2RAD(m_qNonSing);
            ROS_WARN_STREAM("Non-Singular Joint State [RAD]: " << m_qNonSing.transpose());

            m_circ_traj_phase_shift = DEG2RAD(m_circ_traj_phase_shift);
            ROS_WARN_STREAM("Circle Trajectory parameters \
                 \nCenter in frame{0} [m]: " << m_circ_traj_center.transpose() <<
                                             "\nRadius [m]: " << m_circ_traj_radius.transpose() <<
                                             "\nPhase Shift [rad]: "<< m_circ_traj_phase_shift.transpose() <<
                                             "\nFrequency [rad/s]: " << m_circ_traj_frequency.transpose());

            // initalize the adaptive robot model parameters
            if (!m_ur10_model.initRequest(n)) {
                ROS_ERROR_STREAM("Error initalizing ur10 model");
                m_error = true;
                return false;
            }

            ros::Duration(1.0).sleep();

            return true;
        }

        bool SimpleEffortControl::start() {
            ROS_WARN_STREAM("SimpleEffortControl::start");
            return true;
        }

        Vector6d SimpleEffortControl::tf2pose(ow::HomogeneousTransformation T) {
            Vector4d zero4d;
            zero4d.setZero();
            zero4d[3] = 1.0;
            // Vector3d vX = T.position()
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
                                          const Vector6d &t_QXp) {
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
            Eigen::Matrix<double, 6, 6> J_ef_0;
            PartialPivLU<Tum::Matrix6d> J_ef_0_lu;

            switch (m_ct_sm.getControlMode()) {
                case ControlMode::JS: /* joint space */
                    Sq = Sqx;
                    Qrp = QXrp;
                    Qrpp = QXrpp;
                    Yr = m_ur10_model.regressor(Q, Qp, Qrp, Qrpp);
                    break;

                case ControlMode::CS: /* cartesian space */
                    J_ef_0 = m_ur10_model.J_ef_0(current_js.q);
                    // Eigen::Matrix<double,3,6> J_ef_0_v = J_ef_0.topRows(3);
                    // ROS_WARN_STREAM("Jef_0=\n" << J_ef_0);
                    J_ef_0_lu = J_ef_0.lu();

                    Sq = J_ef_0_lu.solve(Sqx);
                    Qrp = J_ef_0_lu.solve(QXrp);
                    Qrpp = J_ef_0_lu.solve(QXrpp);
                    // ROS_WARN_STREAM("Sq=\n" << Sq.transpose());
                    // ROS_WARN_STREAM("Qrp=\n" << Qrp.transpose());
                    // ROS_WARN_STREAM("Qrpp=\n" << Qrpp.transpose());

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
            tau = -m_ct_sm.getKd() * Sq + Yr * m_theta;

            // parameter update
            MatrixXd gamma = MatrixXd::Identity(81, 81);
            gamma *= 0.0002;
            // parameter vector update
            m_theta -= gamma * Yr.transpose() * Sq;

            return tau;
        }

        Vector6d SimpleEffortControl::antiWindUp(Vector6d tau) {
            std::stringstream ss;
            bool b_anti_windup = false;

            for (size_t idx = 0; idx < 6; idx++) {
                double tau_idx = tau[idx];
                double max_tau_idx = c_max_control_effort[idx];
                if (tau_idx > max_tau_idx) {
                    tau[idx] = max_tau_idx;
                    // switch on anti-windup
                    b_anti_windup = true;
                    ss << idx << ", ";
                    m_anti_windup[idx] = 0.0;
                } else {
                    m_anti_windup[idx] = 1.0;
                }
            }

            if (b_anti_windup) {
                ROS_WARN_STREAM("anti-windup on axes: " << ss.str());
            }

            return tau;
        }

        VVector3d SimpleEffortControl::sinusoid_traj_gen(const Vector3d amp, const Vector3d w, const Vector3d phase_shift, const Vector3d zero_offset, double time){
            Vector3d Xd, Xdp, Xdpp;

            Vector3d phi = w * time + phase_shift;
            Xd = zero_offset.array() +       amp.array() * phi.array().sin().array();
            Xdp =                w.array() * amp.array() * phi.array().cos().array();
            Xdpp = -1 * w.array().square() * amp.array() * phi.array().sin().array();
            
            VVector3d Xd_Xdp_Xdpp;
            Xd_Xdp_Xdpp.reserve(3);
            Xd_Xdp_Xdpp.push_back(Xd);
            Xd_Xdp_Xdpp.push_back(Xdp);
            Xd_Xdp_Xdpp.push_back(Xdpp);

            return Xd_Xdp_Xdpp;
        }

        Vector6d SimpleEffortControl::update(const RobotTime &time,
                                             const JointState &current) {

            // time
            double ellapsed_time = time.tD();

            /* ============= variable initialization ============== */

            // desired qd,qdp,qdpp or xd,xdp,xdpp
            VectorDOFd Qd, Qdp, Qdpp;
            VectorDOFd Xd, Xdp, Xdpp;

            VectorDOFd QXd, QXdp, QXdpp;

            // control torque
            Vector6d tau;
            tau.setZero();

            // poly spline QXd, QXdp, QXdpp
            VVectorDOFd vQXd;
            VVectorDOFd vQd_Qdp_Qdpp;
            VVectorDOFd vXd_Xdp_Xdpp;


            /* ============= first time run ============== */
            if (!m_startFlag) {
                // initialize state_space
                m_ct_sm.changeTask(ControlTask::MOVE_OUT_SINGULARITY, m_NonSingTime, ellapsed_time);

                m_startFlag = true;

                m_qStart = current.q;
                ROS_WARN_STREAM("START [DEG]: \n" << m_qStart.transpose());
            }

            // State Machine
            bool state_changed = false;

            switch (m_ct_sm.getCurrentTask()) {
                case ControlTask::BREAK: {
                    // Qd.setZero(); // should be set to the last value
                    Qdp.setZero();
                    Qdpp.setZero();

                    // QXd.setZero(); // should be set to the last value
                    QXdp.setZero();
                    QXdpp.setZero();

                    if (!m_ct_sm.isRunning(ellapsed_time)) {
                        // TODO adjust time
                        double task_time = 5.0;
                        m_ct_sm.changeTask(ControlTask::MOVE_OUT_SINGULARITY, task_time, ellapsed_time);
                        state_changed = true;
                        m_qStart = current.q;

                        ROS_WARN_STREAM(
                                "State Machine: Move out singularity!\n Task time = " << task_time << "\n Goal [rad] = "
                                                                                      << m_qNonSing.transpose());
                    }
                }
                    break;

                case ControlTask::MOVE_OUT_SINGULARITY: {

                    // TODO adjust goal name
                    vQXd = getJointPVT5(m_qStart, m_qNonSing, m_ct_sm.manouverTime(ellapsed_time),
                                        m_ct_sm.getTaskTime());

                    // compatible version
                    QXd = vQXd[0];
                    QXdp = vQXd[1];
                    QXdpp = vQXd[2];

                    // new version
                    vQd_Qdp_Qdpp = vQXd;
                    Qd = vQd_Qdp_Qdpp[0];
                    Qdp = vQd_Qdp_Qdpp[1];
                    Qdpp = vQd_Qdp_Qdpp[2];


                    if (!m_ct_sm.isRunning(ellapsed_time)) {
                        // TODO adjust time
                        double task_time = 6.0;
                        m_ct_sm.changeTask(ControlTask::MOVE_DOWN_AND_ROTATE_UPWARDS, task_time, ellapsed_time);
                        state_changed = true;

                        m_xStart = tf2pose(m_ur10_model.T_ef_0(current.q));
                        m_xGoal = m_xStart;

                        // move down
                        m_xGoal[2] = 0.2;

                        // rotate upwards (euler angles ZYX)
                        // m_xGoal.tail(3) << 2.1715, -1.5, 0.9643;

                        ROS_WARN_STREAM("State Machine: Move down and rotate EF upwards!\n Task time = " << task_time
                                                                                                         << "\n Goal [3x m, 3x rad] = "
                                                                                                         << m_xGoal.transpose());
                    }
                }
                    break;

                case ControlTask::MOVE_DOWN_AND_ROTATE_UPWARDS: {

                    // TODO adjust goal name
                    vQXd = getJointPVT5(m_xStart, m_xGoal, m_ct_sm.manouverTime(ellapsed_time), m_ct_sm.getTaskTime());

                    // compatible version
                    QXd = vQXd[0];
                    QXdp = vQXd[1];
                    QXdpp = vQXd[2];

                    // new version
                    vXd_Xdp_Xdpp = vQXd;
                    Xd = vXd_Xdp_Xdpp[0];
                    Xdp = vXd_Xdp_Xdpp[1];
                    Xdpp = vXd_Xdp_Xdpp[2];

                    if (!m_ct_sm.isRunning(ellapsed_time)) {
                        // TODO adjust time
                        double task_time = m_totalTime;
                        m_ct_sm.changeTask(ControlTask::MOVE_IN_CIRCLE_POINT_UPWARDS, task_time, ellapsed_time);
                        state_changed = true;

                        m_xStart = tf2pose(m_ur10_model.T_ef_0(current.q));

                        ROS_WARN_STREAM("State Machine: Move in circle pointing upwards!\n Task time = " << task_time);
                    }
                }
                    break;

                case ControlTask::MOVE_IN_CIRCLE_POINT_UPWARDS: {

                    VVector3d Xpd_Xpdp_Xpdpp = sinusoid_traj_gen(m_circ_traj_radius,m_circ_traj_frequency, m_circ_traj_phase_shift, m_circ_traj_center, m_ct_sm.manouverTime(ellapsed_time));

                    QXd.setZero();
                    QXdp.setZero();
                    QXdpp.setZero();
                    QXd.head(3) = Xpd_Xpdp_Xpdpp[0];
                    QXdp.head(3) = Xpd_Xpdp_Xpdpp[1];
                    QXdpp.head(3) = Xpd_Xpdp_Xpdpp[2];

                    Xd.setZero();
                    Xdp.setZero();
                    Xdpp.setZero();
                    Xd.head(3) = Xpd_Xpdp_Xpdpp[0];
                    Xdp.head(3) = Xpd_Xpdp_Xpdpp[1];
                    Xdpp.head(3) = Xpd_Xpdp_Xpdpp[2];

                    if (!m_ct_sm.isRunning(ellapsed_time)) {
                        // TODO adjust time
                        double task_time = 2.0;
                        m_ct_sm.changeTask(ControlTask::BREAK, task_time, ellapsed_time);
                        state_changed = true;

                        ROS_WARN_STREAM("State Machine: Break and start over!\n Task time = " << task_time);
                    }
                }
                    break;

                case ControlTask::OBSTACLE_AVOIDANCE: {
                    QXd.setZero();
                    QXdp.setZero();
                    QXdpp.setZero();

                    Xd.setZero();
                    Xdp.setZero();
                    Xdpp.setZero();

                    if (!m_ct_sm.isRunning(ellapsed_time)) {
                        // TODO adjust time
                        double task_time = 2.0;
                        m_ct_sm.changeTask(ControlTask::BREAK, task_time, ellapsed_time);
                        state_changed = true;

                        ROS_WARN_STREAM("State Machine: Break and start over!\n Task time = " << task_time);
                    }
                }
                    break;


                default:
                    ROS_ERROR_STREAM("Unknown ControlTask!");
            }
            

            if ( ellapsed_time - m_last_time > 0.05 ) {
                // path publishing
                geometry_msgs::PoseStamped des_pose;
                geometry_msgs::PoseStamped ef_pose;
                ow::HomogeneousTransformation Tef_0_current;
                ow::HomogeneousTransformation Tef_0_desired;


                ROS_INFO_STREAM("path publishing");
            
                des_pose.header.frame_id = "dh_arm_joint_0";
                des_pose.header.seq = m_path_publish_ctr;
                des_pose.header.stamp = ros::Time::now();

                ef_pose.header.frame_id = "dh_arm_joint_0";
                ef_pose.header.seq = m_path_publish_ctr;
                ef_pose.header.stamp = ros::Time::now();
                Tef_0_current = m_ur10_model.T_ef_0(current.q);
                ef_pose.pose.position.x = Tef_0_current.pos().x();
                ef_pose.pose.position.y = Tef_0_current.pos().y();
                ef_pose.pose.position.z = Tef_0_current.pos().z();

                switch (m_ct_sm.getControlMode()) {
                    case ControlMode::JS:
                        Tef_0_desired = m_ur10_model.T_ef_0(Qd);
                        des_pose.pose.position.x = Tef_0_desired.pos().x();
                        des_pose.pose.position.y = Tef_0_desired.pos().y();
                        des_pose.pose.position.z = Tef_0_desired.pos().z();

                        break;
                    case ControlMode::CS:
                        des_pose.pose.position.x = Xd[0];
                        des_pose.pose.position.y = Xd[1];
                        des_pose.pose.position.z = Xd[2];

                        break;
                    default:
                        ROS_ERROR_STREAM("bad control mode");
                        break;
                }

                path_desired_msg.poses.push_back(des_pose);
                path_desired_msg.header.stamp = ros::Time::now();
                pubCartPath.publish(path_desired_msg);

                path_ef_msg.poses.push_back(ef_pose);
                path_ef_msg.header.stamp = ros::Time::now();
                pubEFPath.publish(path_ef_msg);

                m_path_publish_ctr++;

                if (path_desired_msg.poses.size() > 150) {
                    // remove the first 10 elements
                    path_desired_msg.poses.erase(path_desired_msg.poses.begin(), path_desired_msg.poses.begin() + 5);
                    path_ef_msg.poses.erase(path_ef_msg.poses.begin(), path_ef_msg.poses.begin() + 5);
                }
                m_last_time = ellapsed_time;
            }


            if (state_changed) {
                Qd = current.q;
                Qdp.setZero();
                Qdpp.setZero();
                Xd = tf2pose(m_ur10_model.T_ef_0(current.q));
                Xdp.setZero();
                Xdpp.setZero();

                m_sumDeltaQ.setZero();
                m_sumDeltaQp.setZero();
                m_anti_windup.setOnes();

                switch (m_ct_sm.getControlMode()) {
                    case ControlMode::JS:
                        QXd = Qd;
                        QXdp = Qdp;
                        QXdpp = Qdpp;
                        break;
                    case ControlMode::CS:
                        QXd = Xd;
                        QXdp = Xdp;
                        QXdpp = Xdpp;
                        break;
                    default:
                        ROS_ERROR_STREAM("bad control mode");
                        break;
                }
            }

            // if(!m_startFlag2 && time.tD() > m_totalTime - 0.2) {
            //     m_startFlag2 = true;
            //     m_sumDeltaQ.setZero();
            //     m_sumDeltaQp.setZero();
            //     m_xStart = tf2pose(m_ur10_model.T_ef_0(current.q));
            //     m_xGoal = m_xStart;
            //     // m_xGoal[2] = 0.15;
            //     m_xGoal.tail(3) << 2.1715, -1.5, 0.9643;
            //     m_ct_sm.setControlMode(ControlMode::CS);

            //     m_Kp = m_CS_Kp;
            //     m_Kd = m_CS_Kd;
            //     m_Ki = m_CS_Ki;

            //     ROS_INFO_STREAM("Desired polyline start= \n" << m_xStart.transpose());
            //     ROS_INFO_STREAM("Desired polyline end= \n" << m_xGoal.transpose());
            // }

            VectorDOFd QX, QXp; // current q or x

            switch (m_ct_sm.getControlMode()) {
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

            Vector6d sumDeltaQ_inc = m_anti_windup.array() * (m_DeltaQ * m_controlPeriod).array();
            Vector6d sumDeltaQp_inc = m_anti_windup.array() * (m_DeltaQp * m_controlPeriod).array();

            m_sumDeltaQ += sumDeltaQ_inc;
            m_sumDeltaQp += sumDeltaQp_inc;

            VectorDOFd QXrp, QXrpp; // reference qr or xr
            QXrp = QXdp - m_ct_sm.getKp() * m_DeltaQ - m_ct_sm.getKi() * m_sumDeltaQ;
            QXrpp = QXdpp - m_ct_sm.getKp() * m_DeltaQp - m_ct_sm.getKi() * m_sumDeltaQp;

            // // reference
            // JointState js_r;
            // js_r = current;
            // js_r.qp = vQXd[1] - m_Kp * m_DeltaQ - m_Ki * m_sumDeltaQ;
            // js_r.qpp = vQXd[2] - m_Kp * m_DeltaQp - m_Ki * m_sumDeltaQp;

            // torque calculation
            tau = SimpleEffortControl::tau(time, current, QXrp, QXrpp, QXp);
            // ROS_WARN_STREAM("raw tau = " << tau.transpose());

            // anti-windup
            tau = SimpleEffortControl::antiWindUp(tau);
            // ROS_WARN_STREAM("max tau = " << tau.transpose());


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
            // pubCtrlData.publish(msg);


            return tau;
        }

        bool SimpleEffortControl::stop() {
            return true;
        }

    }  // namespace RobotControllers
}  // namespace tum_ics_ur_robot_lli
