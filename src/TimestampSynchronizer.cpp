#include "TimestampSynchronizer.h"


TimestampSynchronizer::TimestampSynchronizer(Options defaultOptions) {

    pn_ = std::make_unique<ros::NodeHandle>("~timestamp_synchronizer" + (options.nameSuffix.empty() ? std::string() :  "_" + options.nameSuffix));

    options.useMedianFilter_ = pn_->param("useMedianFilter", defaultOptions.useMedianFilter_);
    options.medianFilterWindow_ = pn_->param("medianFilterWindow", defaultOptions.medianFilterWindow_);
    options.useHoltWinters_ = pn_->param("useHoltWinters", defaultOptions.useHoltWinters_);
    options.alfa_HoltWinters_ = pn_->param("alfa_HoltWinters", defaultOptions.alfa_HoltWinters_);
    options.beta_HoltWinters_ = pn_->param("beta_HoltWinters", defaultOptions.beta_HoltWinters_ );
    options.alfa_HoltWinters_early_ = pn_->param("alfa_HoltWinters_early", defaultOptions.alfa_HoltWinters_early_);
    options.beta_HoltWinters_early_ = pn_->param("beta_HoltWinters_early", defaultOptions.beta_HoltWinters_early_);
    options.earlyClamp_ = pn_->param("earlyClamp", defaultOptions.earlyClamp_);
    options.earlyClampWindow_ = pn_->param("earlyClampWindow", defaultOptions.earlyClampWindow_);
    options.timeOffset_ = pn_->param("timeOffset", defaultOptions.timeOffset_);
    options.initialB_HoltWinters_ = pn_->param("initialB_HoltWinters", defaultOptions.initialB_HoltWinters_);

    ROS_INFO("--------------------------");
    ROS_INFO("TimestampSynchronizer init");
    ROS_INFO("useMedianFilter: %d", options.useMedianFilter_ );
    ROS_INFO("medianFilterWindow: %d", options.medianFilterWindow_ );
    ROS_INFO("useHoltWinters: %d", options.useHoltWinters_ );
    ROS_INFO("alfa_HoltWinters: %lf", options.alfa_HoltWinters_ );
    ROS_INFO("beta_HoltWinters: %lf", options.beta_HoltWinters_ );
    ROS_INFO("alfa_HoltWinters_early: %lf", options.alfa_HoltWinters_early_ );
    ROS_INFO("beta_HoltWinters_early: %lf", options.beta_HoltWinters_early_ );
    ROS_INFO("earlyClamp: %d", options.earlyClamp_ );
    ROS_INFO("earlyClampWindow: %d", options.earlyClampWindow_ );
    ROS_INFO("timeOffset: %lf", options.timeOffset_ );
    ROS_INFO("initialB_HoltWinters: %.10lf", options.initialB_HoltWinters_ );
    ROS_INFO("--------------------------");

    init();
}

ros::Time TimestampSynchronizer::sync(double c_sensor, double c_ros_big, unsigned int frameID){
    double running_t0_hypothesis;

    if(!firstFrameSet_) {
        firstFrameID_ = frameID;
        firstFrameSet_ = true;

        startSensorTime_ = c_sensor;
        startROSTimeBig_ = c_ros_big;
    }

    double c_ros = c_ros_big - startROSTimeBig_;
    unsigned int frame_count = frameID - firstFrameID_;

    double time_sensor = c_sensor - startSensorTime_;
    double current_t0_hypothesis = c_ros - time_sensor;

    running_t0_hypothesis = current_t0_hypothesis;

    if(options.useMedianFilter_) {
        pmediator_->insert(running_t0_hypothesis);
        running_t0_hypothesis = pmediator_->getMedian();
    }

    if(options.useHoltWinters_) {
        if(options.earlyClamp_ && frame_count < options.earlyClampWindow_) {
            // smooth interpolation between early and regular alfa/beta
            double progress = ((double) frame_count) / options.earlyClampWindow_;
            // logistic curve with derivation 0 at finish
            double p = 1-std::exp(0.5*(1-1/(1-progress)));

            holtWinters_.setAlfa(p*options.alfa_HoltWinters_ + (1-p)*options.alfa_HoltWinters_early_);
            holtWinters_.setBeta(p*options.beta_HoltWinters_ + (1-p)*options.beta_HoltWinters_early_);
        }
        else {
            holtWinters_.setAlfa(options.alfa_HoltWinters_);
            holtWinters_.setBeta(options.beta_HoltWinters_);
        }
        holtWinters_.insert(running_t0_hypothesis);
        running_t0_hypothesis = holtWinters_.getFiltered();
    }

    double c_out = running_t0_hypothesis + time_sensor;

    double c_err = c_out - c_ros;
    ROS_INFO("delta_sensor: %.10lf delta_ros: %.10lf delta_out: %.10lf frame_count: %d", time_sensor - p_sensor_, c_ros - p_ros_, c_out - p_out_, frame_count);
    ROS_INFO("SENSOR_TIMESTAMP: %.10lf CUR_HYP: %.10lf", time_sensor, current_t0_hypothesis);
    ROS_INFO("ROS_TIME: %.10lf STAMP: %.10lf ERR: %+.10lf RUN_HYP: %.10lf", c_ros, c_out, c_err, running_t0_hypothesis);

    p_out_ = c_out;
    p_sensor_ = time_sensor;
    p_ros_ = c_ros;

    return ros::Time(c_out + options.timeOffset_ + startROSTimeBig_);

}

void TimestampSynchronizer::setTimeOffset(double timeOffset) {
    TimestampSynchronizer::options.timeOffset_ = timeOffset;
}

void TimestampSynchronizer::setMedianFilter(bool useMedianFilter, int medianFilterWindow) {
    TimestampSynchronizer::options.useMedianFilter_ = useMedianFilter;
    TimestampSynchronizer::options.medianFilterWindow_ = medianFilterWindow;
    initMediator();
}

void TimestampSynchronizer::setUseHoltWinters(bool useHoltWinters) {
    TimestampSynchronizer::options.useHoltWinters_ = useHoltWinters;
}

void TimestampSynchronizer::setAlfa_HoltWinters(double alfa_HoltWinters) {
    TimestampSynchronizer::options.alfa_HoltWinters_ = alfa_HoltWinters;
}

void TimestampSynchronizer::setBeta_HoltWinters(double beta_HoltWinters) {
    TimestampSynchronizer::options.beta_HoltWinters_ = beta_HoltWinters;
}

void TimestampSynchronizer::setAlfa_HoltWinters_early(double alfa_HoltWinters_early) {
    TimestampSynchronizer::options.alfa_HoltWinters_early_ = alfa_HoltWinters_early;
}

void TimestampSynchronizer::setBeta_HoltWinters_early(double beta_HoltWinters_early) {
    TimestampSynchronizer::options.beta_HoltWinters_early_ = beta_HoltWinters_early;
}

void TimestampSynchronizer::reset() {
    init();
}

void TimestampSynchronizer::initMediator() {
    pmediator_ = std::make_unique<Mediator<double> >(options.medianFilterWindow_);
}

void TimestampSynchronizer::init() {
    firstFrameSet_ = false;
    initMediator();
    holtWinters_.reset(options.initialB_HoltWinters_);
}

void TimestampSynchronizer::setEarlyClamp(bool earlyClamp, int earlyClampWindow) {
    TimestampSynchronizer::options.earlyClamp_ = earlyClamp;
    TimestampSynchronizer::options.earlyClampWindow_ = earlyClampWindow;
}