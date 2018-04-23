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
        P_int(100.0f),
        V_diff(100.0f) {
            P_int.reset();
            V_diff.reset();
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
                P_i << 0.03f, 0.03f, 0.03f;
                P_p << 0.1f, 0.1f, 0.1f;
                V_p << 0.2f, 0.2f, 0.2f;
                V_d << 0.01f, 0.01f, 0.01f;
            }
        }param_s;


        T get_ref() { return state_ref; }

        void set_ref(const T &ref) { state_ref = ref; }

        bool run(const K &state, res_s &res) {

           Eigen::Vector3d e_P = state_ref.pos_d - state.Pos; 
           Eigen::Vector3d e_V = state_ref.vel_d - state.Vel; 
           
           P_int.update(e_P,state.header); 
           Eigen::Vector3d ctrl_P_int; 
           P_int.get_int(ctrl_P_int); 

           V_diff.update(e_V,state.header); 
           Eigen::Vector3d ctrl_V_diff; 
           V_diff.get_diff(ctrl_V_diff);

           res.header = state.header;
           res.res = ctrl_param.P_i.array()*ctrl_P_int.array()
                + ctrl_param.P_p.array()*e_P.array()
                + ctrl_param.V_p.array()*e_V.array()
                + ctrl_param.V_d.array()*ctrl_V_diff.array();
        }

    private:
        T state_ref;
        Intigrate_State<Eigen::Vector3d> P_int;
        Diff_State<Eigen::Vector3d> V_diff;
        param_s ctrl_param; 
}; 

#endif