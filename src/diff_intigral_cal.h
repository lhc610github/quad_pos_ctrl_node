#ifndef DIFF_INTIGRAL_CAL_H_
#define DIFF_INTIGRAL_CAL_H_

#include "ros/ros.h"
#include <Eigen/Core>

template<class T>
class Intigrate_State{

    public:
        Intigrate_State(float sample_rate): sample_rate_(sample_rate) {
            reset();
        }

        void update(T input, ros::Time now) {
            if (init_flag) {
                if ((now-past).toSec() > 1.0f/sample_rate_) {
                    intigrate_status = (input*((now-past).toSec())) + intigrate_status;
                    intigrate_valid = true;
                }
            } else {
                intigrate_status = input*((now-past).toSec());
            }
            past = now;
            init_flag = true;
        }

        bool get_int(T &output) {
            output = intigrate_status;
            return intigrate_valid;
        }

        bool get_int(ros::Time &t, T &output) {
            output = intigrate_status;
            t = past;
            return intigrate_valid;
        }
        
        void reset() {
            init_flag = false;
            intigrate_valid = false;
        }
    
    private:
        bool intigrate_valid;
        bool init_flag;
        T intigrate_status;
        ros::Time past;
        float sample_rate_;
};

template<class T>
class Diff_State{

    public:
    
        Diff_State(float sample_rate): sample_rate_(sample_rate) {
            reset();
        }

        void update(T input, ros::Time now) {
            if (init_flag) {
                if ((now-past).toSec() > 1.0f/sample_rate_) {
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

        void reset() {
            init_flag = false;
            diff_valid = false;
        }

    private:

        bool init_flag;
        bool diff_valid;
        T status;
        T diff_status;
        ros::Time past;
        float sample_rate_;
};

#endif