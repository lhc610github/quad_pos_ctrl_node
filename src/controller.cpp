#include "controller.h"


void Controller::controller_loop() {
    already_running = true;
    ros::Rate ctrl_rate(100);
    ROS_INFO("controller run at 100Hz");
    status_ref.header = ros::Time::now();
    status_ref.pos_d << 10.0f, 0.0f, -0.5f;
    status_ref.cmd_mask = P_C_V;
    bool armed = true;
    while (ros::ok()) {
        if (ros::Time::now() - get_state().header < ros::Duration(0.5f)) {
            if (armed)
                one_step(); 
            else
                ctrl_core.reset();
        } else {
            ROS_INFO_THROTTLE(1.0,"State Timeout");
            ctrl_core.reset();
        }
        ctrl_rate.sleep();
    }
}

void Controller::one_step() {
        ctrl_core.set_ref(status_ref);
        PID_ctrl<cmd_s,State_s>::res_s ctrl_res;
        ctrl_core.run(get_state(), ctrl_res);
        //std::cout << ctrl_res.res.transpose() << std::endl;
        U_s U = cal_Rd_thrust(ctrl_res);
#ifdef USE_LOGGER
        if (ctrl_logger.is_open()) {
            ctrl_logger << ctrl_res.header.toNSec() << ',';
            ctrl_logger << ctrl_res.res(0) << ',';
            ctrl_logger << ctrl_res.res(1) << ',';
            ctrl_logger << ctrl_res.res(2) << ',';
            ctrl_logger << U.U1 << ',';
            ctrl_logger << U.q_d.w() << ',';
            ctrl_logger << U.q_d.x() << ',';
            ctrl_logger << U.q_d.y() << ',';
            ctrl_logger << U.q_d.z() << std::endl;
        }
#endif
}

Controller::U_s Controller::cal_Rd_thrust(const PID_ctrl<cmd_s,State_s>::res_s &ctrl_res) {
    U_s res;  
    res.header = ctrl_res.header;
    float _yaw_d = status_ref.yaw_d;
    if (ros::Time::now() - ctrl_res.header < ros::Duration(0.5f)) {
        /* get R */
        Eigen::Matrix3d _R;
        get_dcm_from_q(_R, get_state().att_q); 
        /* get U1 */
        double real_U1 = - ctrl_res.res.transpose() * _R.col(2);
        res.U1 = real_U1 / ONE_G * CTRL_K;
        
        /* get body_z */
        Eigen::Vector3d _body_z;
        if (ctrl_res.res.norm() > 0.001f) {
            _body_z = - ctrl_res.res.normalized();
        } else {
            _body_z << 0.0f , 0.0f , 1.0f;
        }

        /* get y_C */
        Eigen::Vector3d _y_C(-sin(_yaw_d), cos(_yaw_d), 0.0f);

        /* get body_x */
        Eigen::Vector3d _body_x;
        if ( fabsf(_body_z(2)) > 0.0001f ) {
            _body_x = _y_C.cross(_body_z);
            if (_body_z(2) < 0) {
                _body_x = - _body_x;
            }
            _body_x = _body_x.normalized();
        } else {
            _body_x << 0.0f , 0.0f , 1.0f;
        }

        /* get body_y */
        Eigen::Vector3d _body_y = _body_z.cross(_body_x);
        _body_y = _body_y.normalized();

        /* get R_d */
        Eigen::Matrix3d _R_d;
        _R_d.col(0) = _body_x;
        _R_d.col(1) = _body_y;
        _R_d.col(2) = _body_z;

        /* get q_d */
        get_q_from_dcm(res.q_d, _R_d);

    } else {
        res.reset();
    }
    return res; 
}