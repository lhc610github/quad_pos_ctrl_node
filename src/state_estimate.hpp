#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>

template<class T>
class Diff_State{

    public:
    
        Diff_State() {
            init_flag = false;
            diff_valid = false;
        }

        void update(T input, ros::Time now) {
            if (init_flag) {
                if ((now-past).toSec() > 1.0f/120.0f) {
                    diff_status = (input-status)/((now-past).toSec());
                    diff_valid = true;
                }
            }
            status = input;
            past = now;
            init_flag = true;
        }

        bool get_diff(ros::Time &timestamp, T& diff) {
                timestamp = past;
                diff = diff_status;
                return diff_valid;
        }

        bool get_diff(T& diff) {
                diff = diff_status;
                return diff_valid;
        }

    private:

        bool init_flag;
        bool diff_valid;
        T status;
        T diff_status;
        ros::Time past;
};

class State_Estimate{
    public:

        typedef struct state_struction {
            ros::Time header;
            Eigen::Vector3d Pos;
            Eigen::Vector3d Vel;
            Eigen::Vector3d Acc;
        }State_s;

        void mocap_data_cb(const geometry_msgs::PoseStamped& mocap_data) {
            // bost::mutex::lock
            memcpy(&rigid_, &mocap_data, sizeof(mocap_data));
            state_.Pos << rigid_.pose.position.x, rigid_.pose.position.y, rigid_.pose.position.z; 

            Pos_differ_.update(state_.Pos, rigid_.header.stamp);
            Eigen::Vector3d temp;
            if (Pos_differ_.get_diff(temp)) {
                state_.Vel = temp;
            } else {
                state_.Vel = Eigen::Vector3d::Zero();
            }

            Vel_differ_.update(state_.Vel, rigid_.header.stamp);
            if (Vel_differ_.get_diff(temp)) {
                state_.Acc = temp;
            } else {
                state_.Acc = Eigen::Vector3d::Zero();
            }

            std::cout << "POS: " << state_.Pos << std::endl; 
            std::cout << "VEL: " << state_.Vel << std::endl; 
            std::cout << "ACC: " << state_.Acc << std::endl; 
            
        }
        
        State_Estimate(const ros::NodeHandle &nh, int id) : nh_(nh), rigidbody_id_(id) {
            char * base_channel;
            base_channel = new char[sizeof("/mocap_data_rigid")];
            strcpy(base_channel,"/mocap_data_rigid");
            char *child_channel = new char[2];
            sprintf(child_channel,"%d",rigidbody_id_);
            char * topic_channel = strcat(base_channel,child_channel);
            mocap_sub_ = nh_.subscribe(topic_channel,10,&State_Estimate::mocap_data_cb,this);
            //mocap_sub = nh_->subscribe("/mocap_data_rigid1",10,&State_Estimate::mocap_data_cb,this);
        }
        ~State_Estimate() {}

        State_s get_state() { return state_; }

        int get_rigidbody_id() { return rigidbody_id_; }

    private:
        ros::NodeHandle nh_;
        int rigidbody_id_;
        geometry_msgs::PoseStamped rigid_;
        Diff_State<Eigen::Vector3d> Pos_differ_;
        Diff_State<Eigen::Vector3d> Vel_differ_;
        State_s state_;
        ros::Subscriber mocap_sub_;
};