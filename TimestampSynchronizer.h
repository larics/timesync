#ifndef TIMESTAMPSYNCHRONIZER_H
#define TIMESTAMPSYNCHRONIZER_H

#include <ros/ros.h>
#include "HoltWinters.h"
#include "Mediator.h"


class TimestampSynchronizer
{
public:
    TimestampSynchronizer() : mediator_(nullptr){
        init();
    };
    ~TimestampSynchronizer() {
        if(mediator_ != nullptr) {
            delete mediator_;
        }
    }
    

    void init() {
        first_frame_set_ = false;
        if(mediator_ != nullptr) {
            delete mediator_;
        }
        mediator_ = new Mediator<double>(5000);
    }


    ros::Time sync(double c_sensor, double c_ros_big, unsigned int frameID);
    double timeOffset_;

private:
    double p_sensor_, p_ros_, p_out_;
    unsigned int first_frame_id_;

    double start_ros_time_big_;
    double sensor0_;
    Mediator<double>* mediator_;
    HoltWintersSmoothFilter smooth_filter_;


    bool first_frame_set_;
};

ros::Time TimestampSynchronizer::sync(double c_sensor, double c_ros_big, unsigned int frameID){
    double running_t0_hypothesis;

    if(!first_frame_set_) {
        first_frame_id_ = frameID;
        first_frame_set_ = true;

        sensor0_ = c_sensor;
        start_ros_time_big_ = c_ros_big;
    }
    double c_ros = c_ros_big - start_ros_time_big_;
    unsigned int frame_count = frameID - first_frame_id_;

    double time_sensor = c_sensor - sensor0_;
    double current_t0_hypothesis = c_ros - time_sensor;
    mediator_->insert(current_t0_hypothesis);
    // recursive mean estimated hypothesis
    if(frame_count>500) {
        if(frame_count < 1500) {
            smooth_filter_.setAlfa(0.9);
            smooth_filter_.setBeta(0.99);
        }
        else {
            smooth_filter_.setAlfa(0.999);
            smooth_filter_.setBeta(0.9999);
        }

        smooth_filter_.insert(mediator_->getMedian());
        running_t0_hypothesis = smooth_filter_.getFiltered();
    }
    else {
        running_t0_hypothesis = mediator_->getMedian();
    }

    double c_out = running_t0_hypothesis + time_sensor;

    double c_err = c_out - c_ros;
    ROS_INFO("delta_sensor: %.10lf delta_ros: %.10lf delta_out: %.10lf frame_count_: %d", time_sensor - p_sensor_, c_ros - p_ros_, c_out - p_out_, frame_count);
    ROS_INFO("SENSOR_TIMESTAMP: %.10lf CUR_HYP: %.10lf", time_sensor, current_t0_hypothesis);
    ROS_INFO("ROS_TIME: %.10lf STAMP: %.10lf ERR: %+.10lf RUN_HYP: %.10lf", c_ros, c_out, c_err, running_t0_hypothesis);

    p_out_ = c_out;
    p_sensor_ = time_sensor;
    p_ros_ = c_ros;

    return ros::Time(c_out + timeOffset_ + start_ros_time_big_);

}
#endif
