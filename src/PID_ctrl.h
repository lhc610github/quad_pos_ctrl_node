#ifndef PID_CTRL_H_
#define PID_CTRL_H_

#include "ros/ros.h"
//#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include "controller.h"

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
        }

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
                return res;
        }

        Eigen::Vector3d single_state_ctrl(const K &state) {
            if ( state_ref.cmd_mask & 1 == 1) {
                state_ref.vel_d = single_state_ctrl_param.P_p.array()*(state_ref.pos_d - state.Pos).array(); 
            }
            if ( state_ref.cmd_mask & 2 == 2 || state_ref.cmd_mask & 1 == 1) {
                if ( state_ref.cmd_mask & 1 != 1 && (ros::Time::now() - state_ref.header > ros::Duration(1.0f)) ) {
                    return Eigen::Vector3d::Zero();
                }
                Eigen::Vector3d e_V = state_ref.vel_d - state.Vel;
                P_int.update(single_state_ctrl_param.P_i.array()*e_V.array(),state.header);
                Eigen::Vector3d ctrl_P_int; 
                P_int.get_int(ctrl_P_int); 

                V_diff2.update(-state.Vel,state.header); 
                Eigen::Vector3d ctrl_V_diff; 
                V_diff2.get_diff(ctrl_V_diff);

                Eigen::Vector3d res = single_state_ctrl_param.V_p.array()*e_V.array()
                    + single_state_ctrl_param.V_d.array()*ctrl_V_diff.array()
                    + single_state_ctrl_param.P_i.array()*ctrl_P_int.array();
                return res;
            }
            return Eigen::Vector3d::Zero();
        }

        bool run(const K &state, res_s &res) {
            res.header = state.header;
            if ( state_ref.cmd_mask != 0 ) {
                if ( state_ref.cmd_mask & 7 == 7) {
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
}; 

#endif