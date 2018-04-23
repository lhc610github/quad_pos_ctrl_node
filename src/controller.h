#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "state_estimate.h"
//#include <boost/thread.hpp>
#include "PID_ctrl.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

void * start_controller_loop_thread(void *args);

class Controller : public State_Estimate {
    public:

        Controller (const ros::NodeHandle &nh, int rigid_id):
        State_Estimate(nh, rigid_id) {
            already_running = false;
        //controller_thread(boost::bind(Controller::controller_loop, this)) {
            int result = pthread_create( &ctrl_tid, NULL, &start_controller_loop_thread, this);
            if ( result ) throw result;
        }

        ~Controller() {
            pthread_join(ctrl_tid, NULL);
        }

        typedef struct ctrl_res {
            ros::Time header;
            Eigen::Quaterniond q_d;
            double U1;
            ctrl_res() {
                header = ros::Time::now();
                q_d = Eigen::Quaterniond::Identity();
                U1 = 0.0f;
            }
        }U_s;

        typedef struct ctrl_cmd {
            ros::Time header;
            Eigen::Vector3d pos_d;
            Eigen::Vector3d vel_d;
            Eigen::Vector3d acc_d;
            uint8_t cmd_mask;
            ctrl_cmd() {
                header = ros::Time::now();
                pos_d = Eigen::Vector3d::Zero();
                vel_d = Eigen::Vector3d::Zero();
                acc_d = Eigen::Vector3d::Zero();
                cmd_mask = 0;
            }
        }cmd_s;

        void start_controller_loop() {
            if( already_running ) {
                fprintf(stderr, "controller loop already running!\n");
            } else {
                controller_loop();
            }
        }

    private:
        void controller_loop();
        //void cal_U1_thrust(const );
        //boost::thread controller_thread;
        pthread_t ctrl_tid;
        cmd_s status_ref;
        bool already_running;

        /* choose the ctrl method */
        PID_ctrl<cmd_s,State_s> ctrl_core;
};

void *
start_controller_loop_thread(void *args) {
    Controller *controller = (Controller *)args;
    controller->start_controller_loop();
    return NULL;
}

#endif