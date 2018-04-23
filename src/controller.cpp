#include "controller.h"


void Controller::controller_loop() {
    already_running = true;
    ros::Rate ctrl_rate(100);
    ROS_INFO("controller run at 100Hz");
    status_ref.header = ros::Time::now();
    status_ref.pos_d << 1.0f, 0.0f, -0.5f;
    status_ref.cmd_mask = status_ref.cmd_mask|1;
    while (ros::ok()) {
        if (ros::Time::now() - get_state().header < ros::Duration(0.5f)) {
            one_step(); 
        } else {
            ROS_INFO("State Timeout"); 
        }
        ctrl_rate.sleep();
    }
}

void Controller::one_step() {
        ctrl_core.set_ref(status_ref);
        PID_ctrl<cmd_s,State_s>::res_s ctrl_res;
        ctrl_core.run(get_state(), ctrl_res);
        std::cout << ctrl_res.res.transpose() << std::endl;
}