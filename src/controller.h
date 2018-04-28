#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define USE_LOGGER 1

#include "state_estimate.h"
//#include <boost/thread.hpp>
#include "PID_ctrl.h"
#include "geometry_math_type.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef USE_LOGGER
//#include "logger.h"
#include <string>
#include <iostream>
#include <fstream>
#endif

#define ONE_G 9.78
#define CTRL_K 0.3

void * start_controller_loop_thread(void *args);

class Controller : public State_Estimate {
    public:

        Controller (const ros::NodeHandle &nh, int rigid_id):
        State_Estimate(nh, rigid_id) {
            already_running = false;

#ifdef USE_LOGGER
            std::string logger_file_name("/home/lhc/work/demo_ws/src/quad_pos_ctrl/src/logger/");
            logger_file_name += "ctrl_logger";
            char data[20];
            sprintf(data, "%lu", ros::Time::now().toNSec());
            logger_file_name += data;
            logger_file_name += ".csv";
            ctrl_logger.open(logger_file_name.c_str(), std::ios::out);
            if (!ctrl_logger.is_open()) {
                std::cout << "cannot open the logger." << std::endl;
            } else {
                ctrl_logger << "timestamp" << ',';
                ctrl_logger << "ctrl_output_x(ned)" << ',';
                ctrl_logger << "ctrl_output_y(ned)" << ',';
                ctrl_logger << "ctrl_output_z(ned)" << ',';
                ctrl_logger << "U1" << ',';
                ctrl_logger << "q_d_w" << ',';
                ctrl_logger << "q_d_x" << ',';
                ctrl_logger << "q_d_y" << ',';
                ctrl_logger << "q_d_z" << std::endl;
            }
#endif

            int result = pthread_create( &ctrl_tid, NULL, &start_controller_loop_thread, this);
            if ( result ) throw result;
        }

        ~Controller() {
            pthread_join(ctrl_tid, NULL);
        }

        typedef struct U_res {
            ros::Time header;
            Eigen::Quaterniond q_d;
            double U1;

            void reset() {
                q_d = Eigen::Quaterniond::Identity();
                U1 = 0.0f;
            }

            U_res() {
                header = ros::Time::now();
                reset();
            }
        }U_s;

        typedef struct ctrl_cmd {
            ros::Time header;
            Eigen::Vector3d pos_d;
            Eigen::Vector3d vel_d;
            Eigen::Vector3d acc_d;
            float yaw_d;
            uint8_t cmd_mask; /* |1: position ctrl valied |2: velocity ctrl valied |3: acc ctrl valied |..|8: */
            ctrl_cmd() {
                header = ros::Time::now();
                pos_d = Eigen::Vector3d::Zero();
                vel_d = Eigen::Vector3d::Zero();
                acc_d = Eigen::Vector3d::Zero();
                yaw_d = 0.0f;
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
        void one_step();
        U_s cal_Rd_thrust(const PID_ctrl<cmd_s,State_s>::res_s &ctrl_res);
        pthread_t ctrl_tid;
        cmd_s status_ref;
        bool already_running;

#ifdef USE_LOGGER
        //Logger ctrl_logger;
        std::ofstream ctrl_logger;
#endif
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