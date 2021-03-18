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
                  m_euler_old(Vector3d::Zero()),
                  m_DeltaQ(Vector6d::Zero()),
                  m_DeltaQp(Vector6d::Zero()),
                  m_anti_windup(Vector6d::Ones()),
                  m_sumDeltaQ(Vector6d::Zero()),
                  m_sumDeltaQp(Vector6d::Zero()),
                  m_gazingSumDeltaQ(Vector3d::Zero()),
                  m_gazingSumDeltaQp(Vector3d::Zero()),
                  m_ct_sm(ControlTaskStateMachine()),
                  m_ur10_model(ur::UR10Model("ur10_model")) {

            pubCtrlData = n.advertise<tum_ics_ur_robot_msgs::ControlData>(
                    "SimpleEffortCtrlData", 100);

            pubTrajMarker = n.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 100);
            pubCartPath = n.advertise<nav_msgs::Path>("path_cs_des_traj", 10);
            pubEFPath = n.advertise<nav_msgs::Path>("path_ef_traj", 10);
            pubTargetPose = n.advertise<geometry_msgs::PoseStamped>("target_pose", 100);
            pubCurentGazingPose = n.advertise<geometry_msgs::PoseStamped>("gazing_pose", 100);

            path_desired_msg.header.frame_id = "dh_arm_joint_0";
            path_ef_msg.header.frame_id = "dh_arm_joint_0";
            target_pose_msg.header.frame_id = "dh_arm_joint_3";
            gazing_pose_msg.header.frame_id = "dh_arm_joint_3";

            m_fac = Vector6d::Ones();
            m_vObstacles_pos_0.reserve(m_max_num_obstacles);
            m_obs2joint_vDis.resize(m_max_num_obstacles);
            m_obs2joint_dis.resize(m_max_num_obstacles);
            m_obs2joint_act.resize(m_max_num_obstacles);

            for (int i_obs = 0; i_obs< m_max_num_obstacles; i_obs++){
                m_obs2joint_vDis[i_obs].resize(STD_DOF);
                m_obs2joint_dis[i_obs].setZero();
                m_obs2joint_act[i_obs].setZero();

                for (int i_joint = 0; i_joint < STD_DOF; i_joint++){
                    m_obs2joint_vDis[i_obs][i_joint].setZero();
                }
            }

            m_theta = m_ur10_model.parameterInitalGuess();
            m_gamma = 0.0002 * MatrixXd::Identity(81, 81);

            m_controlPeriod = 0.002;

            ROS_INFO_STREAM("SimpleEffortCtrl Control Period: " << m_controlPeriod);
        }

        SimpleEffortControl::~SimpleEffortControl() {}

        void joint2eulerZYXatT3(const JointState &current_js, Vector3d &euler, Vector3d &eulerP){
            Vector3d q456 = current_js.q.tail(3);
            Vector3d q456p = current_js.qp.tail(3);

            // double yaw, pitch, roll;
            // yaw = q456[0] + M_PI_2;
            // pitch = q456[2];
            // roll = - (q456[1] - M_PI_2);

            euler[0] = q456[0] + M_PI_2; //yaw - z
            euler[1] = q456[2];         //pitch - y
            euler[2] = -(q456[1] - M_PI_2); //roll - x

            eulerP[0] = q456p[0];
            eulerP[1] = q456p[2];
            eulerP[2] = -q456p[1];
        }

        void euler_p_ppZYXatT3_2_joint(const Vector3d &eulerP, const Vector3d &eulerPP, Vector3d &Qrp, Vector3d &Qrpp){
            // Qrp[0] = euler[0] - M_PI_2;
            Qrp[0] = eulerP[0];
            // Qrp[1] = -euler[2] + M_PI_2;
            Qrp[1] = -eulerP[2];
            // Qrp[2] = euler[1];
            Qrp[2] = eulerP[1];

            Qrpp[0] = eulerPP[0];
            Qrpp[1] = -eulerPP[2];
            Qrpp[2] = eulerPP[1];
        }


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
            m_target_pos_t[0] = target_x.x;
            m_target_pos_t[1] = target_x.y;
            m_target_pos_t[2] = target_x.z;
            m_target_pos_t = m_ur10_model.T_B_0() * m_target_pos_t;
            // ROS_WARN_STREAM("Target position [m]: " << m_target_pos_t.transpose());
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

            // sleep_after_loading_parameters
            ros::param::get(ns + "/sleep_after_loading_parameters", m_sleep_param_load);
            if (m_totalTime <= 0) {
                ROS_ERROR_STREAM("sleep_after_loading_parameters: is negative:" << m_sleep_param_load);
                m_sleep_param_load = 1.0;
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

            // link_modifier
            ros::param::get(ns + "/link_modifier", m_link_modifier);
            if (!(m_link_modifier <= 1.0 && m_link_modifier > 0.5)) {
                m_link_modifier = 0.7;
                ROS_ERROR_STREAM("link_modifier: is not in the interval [ 0.5 ; 1.0 ], setting to default: " << m_link_modifier);
            }

            // radial_influence
            ros::param::get(ns + "/radial_influence", m_radial_influence);
            if (m_radial_influence <= 0.1) {
                m_radial_influence = 0.1;
                ROS_ERROR_STREAM("radial_influence: is not in the interval [ 0.1 , inf], setting to default: " << m_radial_influence);
            }

            // repulsive_force_gain
            ros::param::get(ns + "/repulsive_force_gain", vec);
            if (vec.size() < STD_DOF) {
                m_fac = Vector6d::Ones();
                ROS_ERROR_STREAM("repulsive_force_gain size bad, setting to default: " << m_fac.transpose());
            }
            for (int i = 0; i < STD_DOF; i++) {
                m_fac(i) = vec[i];
            }

            // m_update_hz
            ros::param::get(ns + "/update_hz", m_update_hz);
            if (m_update_hz <= 0.0) {
                m_update_hz = 10;
                ROS_ERROR_STREAM("update_hz: is smaller than 0, setting to default: " << m_update_hz);
            }

            // max_path_size
            ros::param::get(ns + "/max_path_size", m_max_path_size);
            if (m_max_path_size <= 0.0) {
                m_max_path_size = 150;
                ROS_ERROR_STREAM("max_path_size: is smaller than 0, setting to default: " << m_max_path_size);
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

            ROS_WARN_STREAM("Link modifier [-]: " << m_link_modifier);
            ROS_WARN_STREAM("Radial influence [-]: " << m_radial_influence);
            ROS_WARN_STREAM("Repulsive force gain for joints [-]: " << m_fac.transpose());
            ROS_WARN_STREAM("Publisher update [Hz]: " << m_update_hz);
            ROS_WARN_STREAM("Max path length [-]: " << m_max_path_size);

            return true;
        }

        bool SimpleEffortControl::init() {
            ROS_WARN_STREAM("SimpleEffortControl::init");

            Matrix6d t_Kd, t_Kp, t_Ki;
            t_Kd.setZero();
            t_Kp.setZero();
            t_Ki.setZero();
            

            if (!loadControllerGains("joint_space_ctrl", t_Kd, t_Kp, t_Ki)) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): joint_space_ctrl could not be initialized");
                m_error = true;
                return false;
            } else {
                m_ct_sm.initControllerGains(ControlMode::JS, t_Kd, t_Kp, t_Ki);
            }

            if (!loadControllerGains("cartesian_space_ctrl", t_Kd, t_Kp, t_Ki)) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): cartesian_space_ctrl could not be initialized");
                m_error = true;
                return false;
            } else {
                m_ct_sm.initControllerGains(ControlMode::CS, t_Kd, t_Kp, t_Ki);
            }

            if (!loadControllerGains("impedance_ctrl", t_Kd, t_Kp, t_Ki)) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): impedance_ctrl could not be initialized");
                m_error = true;
                return false;
            } else {
                m_ct_sm.initControllerGains(ControlMode::IMPEDANCE, t_Kd, t_Kp, t_Ki);
            }

            if (!loadConfigParameters()) {
                ROS_ERROR_STREAM(
                        "SimpleEffortControl init(): config parameters could not be initialized");
                m_error = true;
                return false;
            }


            // initalize the adaptive robot model parameters
            if (!m_ur10_model.initRequest(n)) {
                ROS_ERROR_STREAM("Error initalizing ur10 model");
                m_error = true;
                return false;
            }

            // sleep to check out the settings 
            ros::Duration(m_sleep_param_load).sleep();

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
            Matrix3d rot_mat = T.matrix().topLeftCorner(3, 3);
            Vector3d euler_angles = rot_mat.eulerAngles(2, 1, 0);
            Vector6d pose;
            pose.topRows(3) = T.pos();
            // pose.bottomRows(3) = T.orientation().eulerAngles(2,1,0);
            pose.bottomRows(3) = Vector3d::Zero();

            return pose;
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
                } else if (tau_idx < -max_tau_idx){
                    tau[idx] = -max_tau_idx;
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

        Vector3d normal_eulerZYX(Vector3d euler){
            if ( euler[0] > M_PI){
                euler[0] -= M_PI;
            }

            if ( euler[1] > M_PI_2){
                euler[1] -= M_PI_2;
            } else if (euler[1] < -M_PI_2){
                euler[1] += M_PI_2;
            }

            if ( euler[2] > M_PI_2){
                euler[2] -= M_PI_2;
            } else if (euler[2] < -M_PI_2){
                euler[2] += M_PI_2;
            }

            return euler;
        }

        void SimpleEffortControl::publishMsgs(const JointState &current_js, const Vector6d &Qdes, const Vector6d &Xdes, double current_time, double update_hz, int max_path_size){
            
            if ( (current_time - m_last_time) > (1 / update_hz) ) {
                
                // ow::HomogeneousTransformation Target_0 = getTargetHT_0(m_ef_x, m_target_pos_t);
                ow::HomogeneousTransformation T_3_0 = m_ur10_model.T_j_0(current_js.q,2);
                ow::HomogeneousTransformation T_6_0 = m_ur10_model.T_ef_0(current_js.q);
                ow::HomogeneousTransformation Target_3 = getTargetHT_3(current_js, m_ef_x, m_target_pos_t);

                gazing_pose_msg.header.stamp = ros::Time::now();
                gazing_pose_msg.header.seq = m_path_publish_ctr;
                Vector3d euler, dummy;
                joint2eulerZYXatT3(current_js, euler, dummy);
                ow::HomogeneousTransformation eulerHT;
                eulerHT.affine() << Rz(euler[0]) * Ry(euler[1]) * Rx(euler[2]), Vector3d::Zero();
                gazing_pose_msg.pose = eulerHT.toPoseMsg();
                gazing_pose_msg.pose.position = ow::HomogeneousTransformation(T_3_0.inverse() * T_6_0).pos();
                pubCurentGazingPose.publish(gazing_pose_msg);


                target_pose_msg.header.stamp = ros::Time::now();
                target_pose_msg.header.seq = m_path_publish_ctr;
                // target_pose_msg.pose = Target_0.toPoseMsg();
                target_pose_msg.pose = Target_3.toPoseMsg();

                // ow::HomogeneousTransformation Rz;
                // Rz.affine() << Rzd(180.0), Vector3d::Zero();
                // Rz = Rz.inverse();
                // ow::HomogeneousTransformation tmp1, tmp2;
                // tmp2 = Rz*getTargetHT_3(current_js, m_ef_x, m_target_pos_t);
                Vector3d Qd_t = Target_3.orien().eulerAngles(2,1,0);
                tf::Matrix3x3 m(Target_3.orien().toQuaternionTF());
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                
                // Qd_t = RAD2DEG(normal_eulerZYX(Qd_t));
                Qd_t[0] = yaw;
                Qd_t[1] = pitch;
                Qd_t[2] = roll;

                // Qd_t = RAD2DEG(Qd_t);

                // target_pose_msg.pose.position.x = Qd_t[0];
                // target_pose_msg.pose.position.y = Qd_t[1];
                // target_pose_msg.pose.position.z = Qd_t[2];

                // Matrix3d rot_zxy = Rz(current_js.q[3]+M_PI_2) * Ry(current_js.q[4]) * Rz(current_js.q[5]);
                // ow::HomogeneousTransformation EF_3;
                // EF_3.affine() << rot_zxy, Target_3.pos();                
                // target_pose_msg.pose = Target_3.toPoseMsg();

                // ROS_INFO_STREAM("Target_0 ZYX: " << Target_0.orien().eulerAngles(2,1,0).transpose());
                // ROS_INFO_STREAM("EF_3 ZXY: " << (Target_3.orien().eulerAngles(2,1,2).transpose()));
                pubTargetPose.publish(target_pose_msg);


                // publish the ControlData (only for debugging)
                // tum_ics_ur_robot_msgs::ControlData msg;
                // msg.header.stamp = ros::Time::now();
                // msg.time = current_time;
                // for (int i = 0; i < STD_DOF; i++) {
                //     msg.q[i] = current_js.q(i);
                //     msg.qp[i] = current_js.qp(i);
                //     msg.qpp[i] = current_js.qpp(i);

                //     // msg.qd[i] = vQXd[0](i);
                //     msg.qd[i] = Qdes(i);
                //     // msg.qpd[i] = vQXd[1](i);
                //     msg.qpd[i] = Xdes(i);

                //     msg.Dq[i] = m_DeltaQ(i);
                //     msg.Dqp[i] = m_DeltaQp(i);

                //     msg.torques[i] = current_js.tau(i);
                // }
                // pubCtrlData.publish(msg);

                // path publishing
                geometry_msgs::PoseStamped des_pose;
                geometry_msgs::PoseStamped ef_pose;
                ow::HomogeneousTransformation Tef_0_current;
                ow::HomogeneousTransformation Tef_0_desired;


                // ROS_INFO_STREAM("path publishing");
            
                des_pose.header.frame_id = "dh_arm_joint_0";
                des_pose.header.seq = m_path_publish_ctr;
                des_pose.header.stamp = ros::Time::now();

                ef_pose.header.frame_id = "dh_arm_joint_0";
                ef_pose.header.seq = m_path_publish_ctr;
                ef_pose.header.stamp = ros::Time::now();
                Tef_0_current = m_ur10_model.T_ef_0(current_js.q);
                ef_pose.pose.position.x = Tef_0_current.pos().x();
                ef_pose.pose.position.y = Tef_0_current.pos().y();
                ef_pose.pose.position.z = Tef_0_current.pos().z();

                switch (m_ct_sm.getControlMode()) {
                    case ControlMode::JS:
                        Tef_0_desired = m_ur10_model.T_ef_0(Qdes);
                        des_pose.pose.position.x = Tef_0_desired.pos().x();
                        des_pose.pose.position.y = Tef_0_desired.pos().y();
                        des_pose.pose.position.z = Tef_0_desired.pos().z();

                        break;
                    case ControlMode::CS:
                        des_pose.pose.position.x = Xdes[0];
                        des_pose.pose.position.y = Xdes[1];
                        des_pose.pose.position.z = Xdes[2];

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

                if (path_desired_msg.poses.size() > max_path_size) {
                    // remove the first 10 elements
                    path_desired_msg.poses.erase(path_desired_msg.poses.begin(), path_desired_msg.poses.begin() + 5);
                    path_ef_msg.poses.erase(path_ef_msg.poses.begin(), path_ef_msg.poses.begin() + 5);
                }
                m_last_time = current_time;
            }
        }

        bool SimpleEffortControl::isObstacleClose(const JointState &current_js, const double rad_inf){

            bool is_obs_close = false;

            VVector3d robot_joint_Xpos_0;
            robot_joint_Xpos_0.reserve(STD_DOF);

            Vector6d joint2obs_dis;
            joint2obs_dis.setZero();

            for (int i_obs = 0; i_obs< m_num_obstacles; i_obs++){
                // m_obs2joint_vDis[i_obs][0] = m_vObstacles_pos_0[i_obs];

                for (int j_joint = 0; j_joint < STD_DOF; j_joint++){
                    // joint positions in frame 0
                    robot_joint_Xpos_0[j_joint] = m_ur10_model.T_j_0(current_js.q, j_joint).pos();

                    // obstacle to joint distance vector (with negative sign - at the end it will be a repulsive force)
                    m_obs2joint_vDis[i_obs][j_joint] = m_vObstacles_pos_0[i_obs] - robot_joint_Xpos_0[j_joint];
                    joint2obs_dis[j_joint] = m_obs2joint_vDis[i_obs][j_joint].norm();
                }

                // ROS_WARN_STREAM("Obstacle "<< i_obs << " distance to joints [m]: " << joint2obs_dis.transpose());

                for (int j_joint = 1; j_joint < STD_DOF; j_joint++){
                    Vector3d vjm1_j = robot_joint_Xpos_0[j_joint] - robot_joint_Xpos_0[j_joint-1];
                    double link_length = vjm1_j.norm();

                    link_length *= m_link_modifier;

                    // ROS_INFO_STREAM("Link "<< j_joint<< " modified length[m] = "<< link_length);
                    
                    if ( joint2obs_dis[j_joint-1] < link_length && joint2obs_dis[j_joint-1] < link_length){
                        // ROS_INFO_STREAM("Link "<< j_joint<< " obstacle is close");

                        Vector3d vjm1_obs = m_obs2joint_vDis[i_obs][j_joint-1];
                        Vector3d vj_obs = m_obs2joint_vDis[i_obs][j_joint];

                        Vector3d v_linkj_obs = vj_obs.array() * (Vector3d::Ones() - vjm1_j/link_length).array();
                        double link_obs_dis = v_linkj_obs.norm();

                        if ( link_obs_dis < joint2obs_dis[j_joint-1]){
                            m_obs2joint_vDis[i_obs][j_joint-1] = v_linkj_obs;
                        }

                        if ( link_obs_dis < joint2obs_dis[j_joint]){
                            m_obs2joint_vDis[i_obs][j_joint] = v_linkj_obs;
                        }
                    }
                }   

                             
                m_obs2joint_act[i_obs].setZero();
                for (int j_joint = 0; j_joint < STD_DOF; j_joint++){

                    double obs2joint_dis = m_obs2joint_vDis[i_obs][j_joint].norm();
                    
                    m_obs2joint_dis[i_obs][j_joint] = obs2joint_dis;

                    if (obs2joint_dis < rad_inf){
                        is_obs_close = true;
                        m_obs2joint_act[i_obs][j_joint] = 1;
                    }
                }   
                // ROS_WARN_STREAM("Obstacle "<< i_obs << " act joints [bool]: " << m_obs2joint_act[i_obs].transpose());
            }

            return is_obs_close;
        }

        ow::HomogeneousTransformation SimpleEffortControl::getTargetHT_0(const Vector3d &fromPosition_0, const Vector3d &inDirectionOfPosition_0){
            Vector3d vDir1 = inDirectionOfPosition_0 - fromPosition_0;
            // vDir1 /= vDir1.norm();

            Vector3d hV1 = vDir1;
            hV1[0] +=vDir1.norm();
            // hV1 /= hV1.norm();

            // Vector3d vDir2 = hV1.cross(vDir1);
            Vector3d vDir2 = vDir1.cross(hV1);
            Vector3d vDir3 = vDir1.cross(vDir2);

            ow::HomogeneousTransformation HT;
            vDir1 /= vDir1.norm();
            vDir2 /= vDir2.norm();
            vDir3 /= vDir3.norm();

            // ROS_WARN_STREAM("Dir1: " << vDir1.transpose());
            // ROS_WARN_STREAM("Dir2: " << vDir2.transpose());
            // ROS_WARN_STREAM("Dir3: " << vDir3.transpose());
            
            HT.affine() << vDir3, vDir1, vDir2, fromPosition_0;
            // hV1 = inDirectionOfPosition_0;
            // hV1[2] = 0;
            // HT.affine() << Matrix3d::Identity(), inDirectionOfPosition_0;
            // ROS_WARN_STREAM("HT: " << HT.matrix());
            return HT;
        }

        ow::HomogeneousTransformation SimpleEffortControl::getTargetHT_3(const JointState &current_js, const Vector3d &fromPosition_0, const Vector3d &inDirectionOfPosition_0){
            ow::HomogeneousTransformation T_3_0 = m_ur10_model.T_j_0(current_js.q,2);
            ow::HomogeneousTransformation Target_3 = T_3_0.inverse() * getTargetHT_0(fromPosition_0, inDirectionOfPosition_0);
            return Target_3;
        }

        Vector6d SimpleEffortControl::tauUR10Compensation(const Vector6d &Sq,
                                                    const Vector6d &Q, 
                                                    const Vector6d &Qp,
                                                    const Vector6d &Qrp, 
                                                    const Vector6d &Qrpp){

            ur::UR10Model::Regressor Yr = m_ur10_model.regressor(Q, Qp, Qrp, Qrpp);

            // calculate torque
            Vector6d tau_ur10_model_comp = Yr * m_theta;

            // parameter update
            m_theta -= m_gamma * Yr.transpose() * Sq;

            return tau_ur10_model_comp;
        }

        Vector3d SimpleEffortControl::tauObstacleAvoidance(const JointState &current_js, const Vector3d &Xd_ef_0, const Vector3d &Xdp_ef_0){
            /* ============= obstacle avoidance ============== */
            Vector3d tau = Vector3d::Zero();

            Vector6d tau_des = Vector6d::Zero();
            Vector6d tau_obs_avoid = Vector6d::Zero();

            // virtual repulsive Force and Moment
            VVector3d Fj_0;
            Fj_0.resize(STD_DOF);
            for( int jdx = 0; jdx < STD_DOF; jdx++ ) {
                Fj_0[jdx].setZero();
            }

            // constant value to avoid singularities when the distance between obstacle
            // and joints
            double epsi = 0.001;
            

            //  Force gain for the repulsive forces
            double mag;
            double n_vDis;
            
            // impedance control for first 3 joints
            // sum all the forces from all obstacles
            for (int i_obs = 0; i_obs < m_num_obstacles; i_obs++){

                // ROS_WARN_STREAM("Obstacle "<< i_obs << " 's distance to joints [m]: " << m_obs2joint_dis[i_obs].transpose() );
                // ROS_WARN_STREAM("Obstacle "<< i_obs << " act joints [bool]: " << m_obs2joint_act[i_obs].transpose());

                for( int j_joint = 0; j_joint < STD_DOF; j_joint++){

                    if (m_obs2joint_act[i_obs][j_joint]){

                        n_vDis = m_obs2joint_dis[i_obs][j_joint];

                        mag = 1 / std::pow( (epsi + n_vDis) , 2 );

                        // !!IMPORTANT the negative sign hence it is a repulsive force!!
                        Fj_0[j_joint] -= m_fac[j_joint] * mag / n_vDis * m_obs2joint_vDis[i_obs][j_joint]; 
                    }   
                }
            }
            
            // obstacle avoidance torque calculation
            for (int j_joint = 0; j_joint < STD_DOF; j_joint++){
                Vector6d Tau_joint_0 = m_ur10_model.J_j_0(current_js.q, j_joint).topLeftCorner(3,6).transpose() * Fj_0[j_joint];
                // ROS_WARN_STREAM("Repulsive Torque from Joint "<< j_joint << " [Nm]: " << Tau_joint_0.transpose());
                tau_obs_avoid += Tau_joint_0;
            }

            // Virtual attractive force to the desired target
            // current position
            Vector3d X = m_ur10_model.T_ef_0(current_js.q).pos();
            Vector3d Xp = m_ur10_model.J_ef_0(current_js.q).topLeftCorner(3,6) * current_js.qp.head(3);

            // errors
            Vector3d DX_ef_0 = Xd_ef_0 - X;
            Vector3d DXp_ef_0 = Xdp_ef_0 - Xp;

            // =================================================================================================
            // maximize attractive force
            // =================================================================================================

            double max_DX = 0.2;
            double max_DXp = 1.0;

            for(int idx = 0; idx < DX_ef_0.size(); idx++){
                
                if (DX_ef_0[idx] > max_DX){
                    DX_ef_0[idx] = max_DX;
                }  
                
                if (DX_ef_0[idx] < -max_DX){
                    DX_ef_0[idx] = -max_DX;
                }

                if (DXp_ef_0[idx] > max_DXp){
                    DXp_ef_0[idx] = max_DXp;
                } 

                if (DXp_ef_0[idx] < -max_DXp){
                    DXp_ef_0[idx] = -max_DXp;
                } 
            }

            Vector3d Fattr = m_ct_sm.getKpIM().topLeftCorner(3,3) * DX_ef_0 + m_ct_sm.getKdIM().topLeftCorner(3,3) * DXp_ef_0;

            tau_des = m_ur10_model.J_ef_0(current_js.q).topLeftCorner(3,6).transpose() * Fattr;

            // ROS_WARN_STREAM("DX_ef_0 [m]: " << DX_ef_0.transpose());
            // ROS_WARN_STREAM("DXp_ef_0 [m]: " << DXp_ef_0.transpose());
            // ROS_WARN_STREAM("Force attractive destination [Nm]: " << Fattr.transpose());
            
            tau.setZero();
            tau = tau_obs_avoid.head(3) + tau_des.head(3);
            // ROS_WARN_STREAM("Torque avoidance              [Nm]: " << tau_obs_avoid.transpose());
            // ROS_WARN_STREAM("Torque tracking               [Nm]: " << tau_des.transpose());
            // ROS_WARN_STREAM("Torque control (avoidance + tracking) [Nm]: " << tau.transpose());
            
            bool is_close_to_ef_traj = true;
            for(int idx = 0; idx < 3; idx++){

                if (std::abs(DX_ef_0[idx]) > 0.030){
                    is_close_to_ef_traj = false;
                }
            }

            if (is_close_to_ef_traj){
                m_ct_sm.setObstacleAvoidance(!is_close_to_ef_traj);
                m_sumDeltaQ.setZero();
                m_sumDeltaQp.setZero();
            }
            

            return tau;
        }

        Vector6d SimpleEffortControl::tau(const RobotTime &time,
                                          const JointState &current_js,
                                          const Vector6d &t_QXd,
                                          const Vector6d &t_QXdp,
                                          const Vector6d &t_QXrp,
                                          const Vector6d &t_QXrpp,
                                          const Vector6d &t_QXp,
                                          Vector6d &tau_ur10_model_comp) {
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

            Vector6d Sq, Sqx;

            // controller 
            Sqx = QXp - QXrp;

            Vector6d Qrp, Qrpp;
            Vector6d Xd_ef_0, Xdp_ef_0;
            Eigen::Matrix<double, 6, 6> J_ef_0;
            PartialPivLU<Tum::Matrix6d> J_ef_0_lu;

            switch (m_ct_sm.getControlMode()) {
                case ControlMode::JS: /* joint space */
                    Sq = Sqx;
                    Qrp = QXrp;
                    Qrpp = QXrpp;
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
                    Xd_ef_0 = t_QXd;
                    Xdp_ef_0 = t_QXdp;

                    break;

                default:
                    ROS_WARN_STREAM("Using DEFAULT control mode: Joint Space");
                    Sq = Sqx;
                    Qrp = QXrp;
                    Qrpp = QXrpp;
            }

            // ROS_WARN_STREAM("Sq =\n" << Sq);
            // calculate torque

            tau.setZero();
            Vector3d tau_gazing = Vector3d::Zero();

            Vector6d origSq = Sq;
            Vector6d origQrp = Qrp;
            Vector6d origQrpp = Qrpp;

            switch (m_ct_sm.getControlMode())
            {
            case ControlMode::JS:{
                tau += -m_ct_sm.getKd() * Sq;
            }
            break;

            case ControlMode::CS:{
                if(m_ct_sm.isObstacleAvoidanceOn()){

                    if(m_ct_sm.isGazing()){
                        // TODO gazing should modify Sq tail(3)
                        tau_gazing = tauGazing(current_js, m_prev_js, Qrp, Qrpp, Sq); // modifies the Qrp and Qrpp tail(3)
                        tau.tail(3) += tau_gazing;
                        ROS_WARN_STREAM("gazeing tau = " << tau_gazing.transpose());
                    }

                    // impedance control for first 3 joints
                    Vector3d tau_obs_avoid = tauObstacleAvoidance(current_js, Xd_ef_0.head(3), Xdp_ef_0.head(3));
                    tau.head(3) += tau_obs_avoid;
                }else{

                    if(m_ct_sm.isGazing()){
                        // TODO gazing should modify Sq tail(3)
                        tau_gazing = tauGazing(current_js, m_prev_js, Qrp, Qrpp, Sq); // modifies the Qrp and Qrpp tail(3)
                        tau.tail(3) += tau_gazing;
                        ROS_WARN_STREAM("gazeing tau = " << tau_gazing.transpose());
                        ROS_WARN_STREAM("tau = " << tau.transpose());
                    }
                    
                    // cs control for first 3 joints
                    tau.head(3) += -m_ct_sm.getKd().topLeftCorner(3,3) * Sq.head(3);
                }
            }
            break;

            default:
                ROS_ERROR_STREAM("tau() - NO SUCH control mode");
                break;
            }

            ROS_WARN_STREAM("tau: Sq  = " << origSq.transpose());
            ROS_WARN_STREAM("tau: Sq'  = " << Sq.transpose());
            ROS_WARN_STREAM("tau: Q    = " << Q.transpose());
            ROS_WARN_STREAM("tau: Qp   = " << Qp.transpose());
            ROS_WARN_STREAM("tau: Qrp  = " << origQrp.transpose());
            ROS_WARN_STREAM("tau: Qrp'  = " << Qrp.transpose());
            ROS_WARN_STREAM("tau: Qrpp = " << origQrpp.transpose());
            ROS_WARN_STREAM("tau: Qrpp' = " << Qrpp.transpose());

            Vector6d model_comp_orig = tauUR10Compensation(origSq, Q, Qp, origQrp, origQrpp);
            Vector6d model_comp = tauUR10Compensation(Sq, Q, Qp, Qrp, Qrpp);
            ROS_WARN_STREAM("tau model compensation  = " << model_comp_orig.transpose());
            ROS_WARN_STREAM("tau model compensation' = " << model_comp.transpose());
            tau += model_comp;

            return tau;
        }

        Vector3d T2eulerZYX(ow::HomogeneousTransformation target){
            tf::Matrix3x3 m(target.orien().toQuaternionTF());
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            Vector3d ypr;
            ypr << yaw, pitch, roll;

            return ypr;
        }

        Vector3d SimpleEffortControl::tauGazing(const JointState &current_js, const JointState &prev_js, Vector6d &commonQXrp, Vector6d &commonQXrpp, Vector6d &commonSq){

            /* ============= GAZING ============= */
            Vector3d EF_Xd_tm1_0 = m_ur10_model.T_ef_0(prev_js.q).pos();
            Vector3d EF_Xd_t_0 = m_ur10_model.T_ef_0(current_js.q).pos();

            ow::HomogeneousTransformation target;
            Vector3d Qd_tm1 = T2eulerZYX(getTargetHT_3(prev_js, EF_Xd_tm1_0, m_target_pos_tm1));
            Vector3d Qd_t =   T2eulerZYX(getTargetHT_3(current_js, EF_Xd_t_0, m_target_pos_t));

            Vector3d Qeuler, QeulerP;
            joint2eulerZYXatT3(current_js, Qeuler, QeulerP);
            // ROS_WARN_STREAM("gaze Qdes t-1: " << Qd_tm1.transpose());
            ROS_WARN_STREAM("gaze Qdes t  : " << Qd_t.transpose());
            ROS_WARN_STREAM("gaze Qcurrent: " << Qeuler.transpose());


            m_target_pos_tm1 = m_target_pos_t;
            
            Vector3d Qd = Qd_t;
            // simple numeric differentiation
            Vector3d Qdp = (Qd_t - Qd_tm1) / m_controlPeriod;
            Vector3d Qdpp = Vector3d::Zero();
            ROS_WARN_STREAM("gaze Qdp t:     " << Qdp.transpose());
            ROS_WARN_STREAM("gaze QcurrentP: " << QeulerP.transpose());


            // erros
            Vector3d gazingDeltaQ = Qeuler - Qd;
            Vector3d gazingDeltaQp = QeulerP - Qdp;
            ROS_WARN_STREAM("gazingDeltaQ:  " << gazingDeltaQ.transpose());
            ROS_WARN_STREAM("gazingDeltaQp: " << gazingDeltaQp.transpose());


            Vector3d sumDeltaQ_inc = m_anti_windup.tail(3).array() * (gazingDeltaQ * m_controlPeriod).array();
            Vector3d sumDeltaQp_inc = m_anti_windup.tail(3).array() * (gazingDeltaQp * m_controlPeriod).array();

            m_gazingSumDeltaQ += sumDeltaQ_inc;
            m_gazingSumDeltaQp += sumDeltaQp_inc;

            // reference
            Vector3d gazeQrp, gazeQrpp; // reference qr or xr
            gazeQrp = Qdp - m_ct_sm.getKp(ControlMode::CS).bottomRightCorner(3,3) * gazingDeltaQ - m_ct_sm.getKi(ControlMode::CS).bottomRightCorner(3,3) * m_gazingSumDeltaQ;
            gazeQrpp = Qdpp - m_ct_sm.getKp(ControlMode::CS).bottomRightCorner(3,3) * gazingDeltaQp - m_ct_sm.getKi(ControlMode::CS).bottomRightCorner(3,3) * m_gazingSumDeltaQp;
            ROS_WARN_STREAM("gaze Qrp: " << gazeQrp.transpose());
            ROS_WARN_STREAM("gaze Qrpp: " << gazeQrpp.transpose());

            // convert back euler space reference to joint space
            Vector3d QXrp, QXrpp;
            euler_p_ppZYXatT3_2_joint(gazeQrp, gazeQrpp, QXrp, QXrpp);
            commonQXrp.tail(3) = QXrp;
            commonQXrpp.tail(3) = QXrpp;

            ROS_WARN_STREAM("commonQXrp: " << commonQXrp.tail(3).transpose());
            ROS_WARN_STREAM("commonQXrpp: " << commonQXrpp.tail(3).transpose());

            // controller 
            Vector3d gazeSq;
            gazeSq = QeulerP - gazeQrp;
            // gazeSq = current_js.qp.tail(3) - QXrp;
            commonSq.tail(3) = current_js.qp.tail(3) - QXrp;

            // !!!missing robot model compensation!!! --> have to be added later on
            Vector3d gazeTau = -m_ct_sm.getKd(ControlMode::CS).bottomRightCorner(3,3) * gazeSq;
            ROS_WARN_STREAM("gazeSq: " << gazeSq.transpose());
            ROS_WARN_STREAM("gazeTau: " << gazeTau.transpose());

            return gazeTau;
        }

        Vector6d SimpleEffortControl::update(const RobotTime &time,
                                             const JointState &current) {

            // time
            double ellapsed_time = time.tD();

            m_ef_x = m_ur10_model.T_ef_0(current.q).pos();

            /* ============= variable initialization ============== */

            // desired qd,qdp,qdpp or xd,xdp,xdpp
            VectorDOFd Qd, Qdp, Qdpp;
            VectorDOFd Xd, Xdp, Xdpp;

            VectorDOFd QXd, QXdp, QXdpp;

            // control torque
            Vector6d tau;
            // Vector6d tau_control;
            Vector6d tau_model_comp;
            tau.setZero();
            // tau_control.setZero();
            // tau_model_comp.setZero();

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

                m_target_pos_tm1 = m_target_pos_t;
                m_prev_js = current;
            }

            /* ============= trajectory generation ============== */
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
                        double task_time = 3.0;
                        m_ct_sm.changeTask(ControlTask::MOVE_TO_CIRCULAR_TRAJECTORY_START, task_time, ellapsed_time);
                        state_changed = true;

                        m_ct_sm.startGazing();

                        m_xStart = tf2pose(m_ur10_model.T_ef_0(current.q));
                        m_xGoal = m_xStart;

                        // move down
                        m_xGoal.head(3) = sinusoid_traj_gen(m_circ_traj_radius,m_circ_traj_frequency, m_circ_traj_phase_shift, m_circ_traj_center, 0.0)[0];


                        // rotate upwards (euler angles ZYX)
                        // m_xGoal.tail(3) << 2.1715, -1.5, 0.9643;

                        ROS_WARN_STREAM("State Machine: Move to circular trajectory start point and start gazing!\n Task time = " << task_time
                                                                                                         << "\n Goal [3x m, 3x rad] = "
                                                                                                         << m_xGoal.transpose());
                    }
                }
                    break;

                case ControlTask::MOVE_TO_CIRCULAR_TRAJECTORY_START: {

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
                        m_ct_sm.changeTask(ControlTask::CIRCULAR_EF_TRAJECTORY_TRACKING, task_time, ellapsed_time);
                        state_changed = true;

                        m_xStart = tf2pose(m_ur10_model.T_ef_0(current.q));

                        ROS_WARN_STREAM("State Machine: Move in circle pointing upwards!\n Task time = " << task_time);
                    }
                }
                    break;

                case ControlTask::CIRCULAR_EF_TRAJECTORY_TRACKING: {

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

                default:
                    ROS_ERROR_STREAM("Unknown ControlTask!");
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

            // reference
            VectorDOFd QXrp, QXrpp; // reference qr or xr
            QXrp = QXdp - m_ct_sm.getKp() * m_DeltaQ - m_ct_sm.getKi() * m_sumDeltaQ;
            QXrpp = QXdpp - m_ct_sm.getKp() * m_DeltaQp - m_ct_sm.getKi() * m_sumDeltaQp;


            // torque calculation
            if ( isObstacleClose(current, m_radial_influence) ){
                // obstacle avoidance
                m_ct_sm.setObstacleAvoidance(true);                
            }


            // m_ct_sm.setObstacleAvoidance(true);                



            tau = SimpleEffortControl::tau(time, current, QXd, QXdp, QXrp, QXrpp, QXp, tau_model_comp);
            // ROS_WARN_STREAM("raw tau = " << tau.transpose());

            // anti-windup
            tau = SimpleEffortControl::antiWindUp(tau);
            ROS_WARN_STREAM("max tau = " << tau.transpose());

            // publishing path msgs
            // double update_hz = 30;
            // int max_path_size = 150;
            publishMsgs(current, Qd, Xd, ellapsed_time, m_update_hz, m_max_path_size);

            m_prev_js = current;

            return tau;
        }

        bool SimpleEffortControl::stop() {
            return true;
        }

    }  // namespace RobotControllers
}  // namespace tum_ics_ur_robot_lli
