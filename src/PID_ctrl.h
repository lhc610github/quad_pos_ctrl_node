#ifndef PID_CTRL_H_
#define PID_CTRL_H_

#include "ros/ros.h"
//#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include "controller.h"

#ifdef USE_LOGGER
//#include "logger.h"
#include <string>
#include <iostream>
#include <fstream>
#endif

#define P_C_V 1 // position control valid
#define V_C_V 2 // velocity control valid
#define A_C_V 4 // acc      control valid

template<class T,class K>
class PID_ctrl {
    public:

        PID_ctrl():
        P_int(110.0f),
        V_diff(110.0f),
        V_diff2(110.0f) {
            P_int.reset();
            V_diff.reset();
            V_diff2.reset();
            g_vector << 0.0f , 0.0f ,-9.8f;

        /* ******************************************************* */
        /*                  all state ctrl param                   */
        /* ------------------------------------------------------- */
            all_state_ctrl_param.P_i << 0.6f, 0.6f, 0.6f;
            all_state_ctrl_param.P_p << 4.0f, 4.0f, 6.0f;
            all_state_ctrl_param.V_p << 4.0f, 4.0f, 6.0f;
            all_state_ctrl_param.V_d << 0.6f, 0.6f, 0.6f;
        /* ------------------------------------------------------- */
        /* ******************************************************* */

        /* ******************************************************* */
        /*                single state ctrl param                  */
        /* ------------------------------------------------------- */
            single_state_ctrl_param.P_i << 0.6f, 0.6f, 0.6f;
            single_state_ctrl_param.P_p << 1.2f, 1.2f, 1.2f;
            single_state_ctrl_param.V_p << 4.0f, 4.0f, 6.0f;
            single_state_ctrl_param.V_d << 0.4f, 0.4f, 0.04f;
        /* ------------------------------------------------------- */
        /* ******************************************************* */

        /* ******************************************************* */
        /*               limit param for controller                */
        /* ------------------------------------------------------- */
            ctrl_limit.vel_xy_limit = 1.0f;
            ctrl_limit.vel_z_limit = 0.5f;
            ctrl_limit.acc_xy_limit = 5.0f;
            ctrl_limit.acc_z_limit = 5.0f;
        /* ------------------------------------------------------- */
        /* ******************************************************* */
#ifdef USE_LOGGER
            std::string logger_file_name("/home/lhc/work/demo_ws/src/quad_pos_ctrl/src/logger/");
            logger_file_name += "PID_logger";
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
        }

        typedef struct CTRL_limit {
            double vel_xy_limit; // unit m/s
            double vel_z_limit; // unit m/s
            double acc_xy_limit; // unit m/ss
            double acc_z_limit; // unit m/ss
            CTRL_limit() {
                vel_xy_limit = 0.0f;
                vel_z_limit = 0.0f;
                acc_xy_limit = 0.0f;
                acc_z_limit = 0.0f;
            }
        } limit_s;

        typedef struct PID_ctrl_res {
            ros::Time header;
            Eigen::Vector3d res;
            PID_ctrl_res() {
                res = Eigen::Vector3d::Zero();
            }
        } res_s;

        typedef struct ctrl_param {
            Eigen::Vector3d P_i;
            Eigen::Vector3d P_p;
            Eigen::Vector3d V_p;
            Eigen::Vector3d V_d;
            ctrl_param() {
                P_i << 0.0f, 0.0f, 0.0f;
                P_p << 0.0f, 0.0f, 0.0f;
                V_p << 0.0f, 0.0f, 0.0f;
                V_d << 0.0f, 0.0f, 0.0f;
            }
        }param_s;

        void reset() {
            P_int.reset();
            V_diff.reset();
            V_diff2.reset();
        }

        void limit_func(Eigen::Vector3d &v, int order) {

            if (order == 1) {
                float temp_xy = v.block<2,1>(0,0).norm();
                if (temp_xy > ctrl_limit.vel_xy_limit) {
                    v(0) = v(0) / temp_xy * ctrl_limit.vel_xy_limit;
                    v(1) = v(1) / temp_xy * ctrl_limit.vel_xy_limit;
                }
                float temp_z = abs(v(2));
                if (temp_z > ctrl_limit.vel_z_limit) {
                    v(2) = v(2) / temp_z * ctrl_limit.vel_z_limit;
                }
            }

            if (order == 2) {
                float temp_xy = v.block<2,1>(0,0).norm();
                if (temp_xy > ctrl_limit.acc_xy_limit) {
                    v(0) = v(0) / temp_xy * ctrl_limit.acc_xy_limit;
                    v(1) = v(1) / temp_xy * ctrl_limit.acc_xy_limit;
                }
                float temp_z = abs(v(2));
                if (temp_z > ctrl_limit.acc_z_limit) {
                    v(2) = v(2) / temp_z * ctrl_limit.acc_z_limit;
                }
            }
        }

        T get_ref() { return state_ref; }

        void set_ref(const T &ref) { state_ref = ref; }

        Eigen::Vector3d all_state_ctrl(const Eigen::Vector3d &P_e,const Eigen::Vector3d &V_e, const K &state) {
                P_int.update(all_state_ctrl_param.P_i.array()*P_e.array(),state.header); 
                Eigen::Vector3d ctrl_P_int; 
                P_int.get_int(ctrl_P_int); 

                V_diff.update(V_e,state.header); 
                Eigen::Vector3d ctrl_V_diff; 
                V_diff.get_diff(ctrl_V_diff);

                Eigen::Vector3d res;
                res = ctrl_P_int.array() + all_state_ctrl_param.P_p.array()*P_e.array()
                    + all_state_ctrl_param.V_p.array()*V_e.array()
                    + all_state_ctrl_param.V_d.array()*ctrl_V_diff.array();
                limit_func(res, 2);
                res = res.array()
                    + g_vector.array();
                return res;
        }

        Eigen::Vector3d single_state_ctrl(const K &state) {
            if ( state_ref.cmd_mask & P_C_V == P_C_V) {
                state_ref.vel_d = single_state_ctrl_param.P_p.array()*(state_ref.pos_d - state.Pos).array(); 
                limit_func(state_ref.vel_d, 1);
            }
            if ( state_ref.cmd_mask & V_C_V == V_C_V || state_ref.cmd_mask & P_C_V == P_C_V) {
                if ( state_ref.cmd_mask & P_C_V != P_C_V && (ros::Time::now() - state_ref.header > ros::Duration(1.0f)) ) {
                    return Eigen::Vector3d::Zero();
                }
                Eigen::Vector3d e_V = state_ref.vel_d - state.Vel;
                P_int.update(single_state_ctrl_param.P_i.array()*e_V.array(),state.header);
                Eigen::Vector3d ctrl_P_int; 
                P_int.get_int(ctrl_P_int); 

                V_diff2.update(-state.Vel,state.header); 
                Eigen::Vector3d ctrl_V_diff; 
                V_diff2.get_diff(ctrl_V_diff);

                Eigen::Vector3d res;
                res = single_state_ctrl_param.V_p.array()*e_V.array()
                    + single_state_ctrl_param.V_d.array()*ctrl_V_diff.array()
                    + ctrl_P_int.array();
                limit_func(res, 2);
                res = res.array()
                    + g_vector.array();
                return res;
            }
            return Eigen::Vector3d::Zero();
        }

        bool run(const K &state, res_s &res) {
            res.header = state.header;
            if ( state_ref.cmd_mask != 0 ) {
                if ( state_ref.cmd_mask  == P_C_V|V_C_V|A_C_V ) {
                    Eigen::Vector3d e_P = state_ref.pos_d - state.Pos; 
                    Eigen::Vector3d e_V = state_ref.vel_d - state.Vel; 
                    res.res = all_state_ctrl(e_P,e_V,state); 
                } else {
                    res.res = single_state_ctrl(state);
                }
            } else {
                res.res << 0.0f, 0.0f, 0.0f;
            }
        }

    private:
        T state_ref;
        Intigrate_State<Eigen::Vector3d> P_int;
        Diff_State<Eigen::Vector3d> V_diff;
        Diff_State<Eigen::Vector3d> V_diff2;
        param_s all_state_ctrl_param; 
        param_s single_state_ctrl_param; 
        Eigen::Vector3d g_vector;
        limit_s ctrl_limit;
#ifdef USE_LOGGER
        //Logger ctrl_logger;
        std::ofstream ctrl_logger;
#endif
}; 

#endif