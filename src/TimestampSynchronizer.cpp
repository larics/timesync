#include "TimestampSynchronizer.h"


TimestampSynchronizer::TimestampSynchronizer(Options defaultOptions) {

    options_.nameSuffix = defaultOptions.nameSuffix;

    // TimestampSynchronizer gets its own namespace, which may have a prefix passed in the Options class
    pn_ = std::make_unique<ros::NodeHandle>("~timestamp_synchronizer" + (options_.nameSuffix.empty() ? std::string() :  "_" + options_.nameSuffix));

    // advertise the debug topic, which publishes the values passed to the sync() method
    debugPublisher_ = pn_->advertise<timesync::TimesyncDebug>("debug_info", 10);

    
    // load options from ROS params if available, if not, from default options passed by caller 
    options_.useMedianFilter = pn_->param("useMedianFilter", defaultOptions.useMedianFilter);
    options_.medianFilterWindow = pn_->param("medianFilterWindow", defaultOptions.medianFilterWindow);
    options_.useHoltWinters = pn_->param("useHoltWinters", defaultOptions.useHoltWinters);
    options_.alfa_HoltWinters = pn_->param("alfa_HoltWinters", defaultOptions.alfa_HoltWinters);
    options_.beta_HoltWinters = pn_->param("beta_HoltWinters", defaultOptions.beta_HoltWinters);
    options_.alfa_HoltWinters_early = pn_->param("alfa_HoltWinters_early", defaultOptions.alfa_HoltWinters_early);
    options_.beta_HoltWinters_early = pn_->param("beta_HoltWinters_early", defaultOptions.beta_HoltWinters_early);
    options_.earlyClamp = pn_->param("earlyClamp", defaultOptions.earlyClamp);
    options_.earlyClampWindow = pn_->param("earlyClampWindow", defaultOptions.earlyClampWindow);
    options_.timeOffset = pn_->param("timeOffset", defaultOptions.timeOffset);
    options_.initialB_HoltWinters = pn_->param("initialB_HoltWinters", defaultOptions.initialB_HoltWinters);

    ROS_DEBUG("--------------------------");
    ROS_DEBUG("TimestampSynchronizer init");
    ROS_DEBUG("useMedianFilter: %d", options_.useMedianFilter);
    ROS_DEBUG("medianFilterWindow: %d", options_.medianFilterWindow);
    ROS_DEBUG("useHoltWinters: %d", options_.useHoltWinters);
    ROS_DEBUG("alfa_HoltWinters: %lf", options_.alfa_HoltWinters);
    ROS_DEBUG("beta_HoltWinters: %lf", options_.beta_HoltWinters);
    ROS_DEBUG("alfa_HoltWinters_early: %lf", options_.alfa_HoltWinters_early);
    ROS_DEBUG("beta_HoltWinters_early: %lf", options_.beta_HoltWinters_early);
    ROS_DEBUG("earlyClamp: %d", options_.earlyClamp);
    ROS_DEBUG("earlyClampWindow: %d", options_.earlyClampWindow);
    ROS_DEBUG("timeOffset: %lf", options_.timeOffset);
    ROS_DEBUG("initialB_HoltWinters: %.10lf", options_.initialB_HoltWinters);
    ROS_DEBUG("--------------------------");

    init();
}

double TimestampSynchronizer::sync(double currentSensorTime, double currentRosTime, unsigned int seqNumber){
    double mean_estimated_t0_hypothesis;

    if(!firstFrameSet_) {
        firstSeqNumber_ = seqNumber;
        firstFrameSet_ = true;

        startSensorTime_ = currentSensorTime;
        startRosTimeBig_ = currentRosTime;
    }

    double c_ros = currentRosTime - startRosTimeBig_;
    unsigned int seq_count = seqNumber - firstSeqNumber_;

    double time_sensor = currentSensorTime - startSensorTime_;
    double current_t0_hypothesis = c_ros - time_sensor;

    mean_estimated_t0_hypothesis = current_t0_hypothesis;
    
    if(options_.useMedianFilter) {
        pmediator_->insert(mean_estimated_t0_hypothesis);
        mean_estimated_t0_hypothesis = pmediator_->getMedian();
    }

    if(options_.useHoltWinters) {
        if(options_.earlyClamp && seq_count < options_.earlyClampWindow) {
            // smooth interpolation between early and regular alfa/beta
            double progress = ((double) seq_count) / options_.earlyClampWindow;
            // logistic curve with derivation 0 at finish, p goes from 0 to 1
            double p = 1-std::exp(0.5*(1-1/(1-progress)));

            holtWinters_.setAlfa(p*options_.alfa_HoltWinters + (1-p)*options_.alfa_HoltWinters_early);
            holtWinters_.setBeta(p*options_.beta_HoltWinters + (1-p)*options_.beta_HoltWinters_early);
        }
        else {
            holtWinters_.setAlfa(options_.alfa_HoltWinters);
            holtWinters_.setBeta(options_.beta_HoltWinters);
        }
        holtWinters_.insert(mean_estimated_t0_hypothesis);
        mean_estimated_t0_hypothesis = holtWinters_.getFiltered();
    }

    double c_out = mean_estimated_t0_hypothesis + time_sensor;

    double c_err = c_out - c_ros;
    timesync::TimesyncDebug debugMessage;
    debugMessage.seq = seqNumber;
    debugMessage.sensorTime = currentSensorTime;
    debugMessage.rosTime = currentRosTime;

    debugPublisher_.publish(debugMessage);
    ROS_DEBUG("delta_sensor: %.10lf delta_ros: %.10lf delta_out: %.10lf frame_count: %d", time_sensor - p_sensor_, c_ros - p_ros_, c_out - p_out_, seq_count);
    ROS_DEBUG("SENSOR_TIMESTAMP: %.10lf CUR_HYP: %.10lf", time_sensor, current_t0_hypothesis);
    ROS_DEBUG("ROS_TIME: %.10lf STAMP: %.10lf ERR: %+.10lf RUN_HYP: %.10lf", c_ros, c_out, c_err, mean_estimated_t0_hypothesis);

    // update previous values, used for calculating deltas
    p_out_ = c_out;
    p_sensor_ = time_sensor;
    p_ros_ = c_ros;

    return c_out + options_.timeOffset + startRosTimeBig_;

}

void TimestampSynchronizer::setTimeOffset(double timeOffset) {
    TimestampSynchronizer::options_.timeOffset = timeOffset;
}

void TimestampSynchronizer::setMedianFilter(bool useMedianFilter, int medianFilterWindow) {
    TimestampSynchronizer::options_.useMedianFilter = useMedianFilter;
    TimestampSynchronizer::options_.medianFilterWindow = medianFilterWindow;
    initMediator();
}

void TimestampSynchronizer::setUseHoltWinters(bool useHoltWinters) {
    TimestampSynchronizer::options_.useHoltWinters = useHoltWinters;
}

void TimestampSynchronizer::setAlfa_HoltWinters(double alfa_HoltWinters) {
    TimestampSynchronizer::options_.alfa_HoltWinters = alfa_HoltWinters;
}

void TimestampSynchronizer::setBeta_HoltWinters(double beta_HoltWinters) {
    TimestampSynchronizer::options_.beta_HoltWinters = beta_HoltWinters;
}

void TimestampSynchronizer::setAlfa_HoltWinters_early(double alfa_HoltWinters_early) {
    TimestampSynchronizer::options_.alfa_HoltWinters_early = alfa_HoltWinters_early;
}

void TimestampSynchronizer::setBeta_HoltWinters_early(double beta_HoltWinters_early) {
    TimestampSynchronizer::options_.beta_HoltWinters_early = beta_HoltWinters_early;
}

void TimestampSynchronizer::reset() {
    init();
}

// allocate a Mediator object
void TimestampSynchronizer::initMediator() {
    pmediator_ = std::make_unique<Mediator<double> >(options_.medianFilterWindow);
}

void TimestampSynchronizer::init() {
    firstFrameSet_ = false;
    initMediator();
    holtWinters_.reset(options_.initialB_HoltWinters);
}

void TimestampSynchronizer::setEarlyClamp(bool earlyClamp, int earlyClampWindow) {
    TimestampSynchronizer::options_.earlyClamp = earlyClamp;
    TimestampSynchronizer::options_.earlyClampWindow = earlyClampWindow;
}