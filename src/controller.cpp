#include "controller.h"


void Controller::controller_loop() {
    already_running = true;
    ros::Rate ctrl_rate(100);
    ROS_INFO("controller run at 100Hz");
    status_ref.header = ros::Time::now();
    status_ref.pos_d << 1.0f, 0.0f, -0.5f;
    while (ros::ok()) {
        ctrl_core.set_ref(status_ref);
        PID_ctrl<cmd_s,State_s>::res_s ctrl_res;
        ctrl_core.run(get_state(), ctrl_res);
        std::cout << ctrl_res.res.transpose() << std::endl;
        ctrl_rate.sleep();
    }
}