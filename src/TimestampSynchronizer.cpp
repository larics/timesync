#include "TimestampSynchronizer.h"


TimestampSynchronizer::TimestampSynchronizer(std::string name) {

    pn_ = new ros::NodeHandle("~timestamp_synchronizer" + (name.empty() ? std::string() :  "_" + name));
    mediator_ = nullptr;

    useMedianFilter_ = pn_->param("useMedianFilter", true);
    medianFilterWindow_ = pn_->param("medianFilterWindow", 3000);
    useHoltWinters_ = pn_->param("useHoltWinters", true);
    alfa_HoltWinters_ = pn_->param("alfa_HoltWinters", 1e-3);
    beta_HoltWinters_ = pn_->param("beta_HoltWinters", 1e-4);
    alfa_HoltWinters_early_ = pn_->param("alfa_HoltWinters_early", 1e-1);
    beta_HoltWinters_early_ = pn_->param("beta_HoltWinters_early", 1e-2);
    earlyClamp_ = pn_->param("earlyClamp", true);
    earlyClampWindow_ = pn_->param("earlyClampWindow", 500);
    timeOffset_ = pn_->param("timeOffset", 0.0);
    initialB_HoltWinters_ = pn_->param("initialB_HoltWinters",-3e-7);

    ROS_INFO("--------------------------");
    ROS_INFO("TimestampSynchronizer init");
    ROS_INFO("useMedianFilter: %d", useMedianFilter_ );
    ROS_INFO("medianFilterWindow: %d", medianFilterWindow_ );
    ROS_INFO("useHoltWinters: %d", useHoltWinters_ );
    ROS_INFO("alfa_HoltWinters: %lf", alfa_HoltWinters_ );
    ROS_INFO("beta_HoltWinters: %lf", beta_HoltWinters_ );
    ROS_INFO("alfa_HoltWinters_early: %lf", alfa_HoltWinters_early_ );
    ROS_INFO("beta_HoltWinters_early: %lf", beta_HoltWinters_early_ );
    ROS_INFO("earlyClamp: %d", earlyClamp_ );
    ROS_INFO("earlyClampWindow: %d", earlyClampWindow_ );
    ROS_INFO("timeOffset: %lf", timeOffset_ );
    ROS_INFO("initialB_HoltWinters: %.10lf", initialB_HoltWinters_ );
    ROS_INFO("--------------------------");

    init();
}

TimestampSynchronizer::~TimestampSynchronizer() {
    if(mediator_ != nullptr) {
        delete mediator_;
        mediator_ = nullptr;
    }
    delete pn_;
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

    if(useMedianFilter_) {
        mediator_->insert(running_t0_hypothesis);
        running_t0_hypothesis = mediator_->getMedian();
    }

    if(useHoltWinters_) {
        if(earlyClamp_ && frame_count < earlyClampWindow_) {
            // smooth interpolation between early and regular alfa/beta
            double progress = ((double) frame_count) / earlyClampWindow_;
            // logistic curve with derivation 0 at finish
            double p = 1-std::exp(0.5*(1-1/(1-progress)));

            holtWinters_.setAlfa(p*alfa_HoltWinters_ + (1-p)*alfa_HoltWinters_early_);
            holtWinters_.setBeta(p*beta_HoltWinters_ + (1-p)*beta_HoltWinters_early_);
        }
        else {
            holtWinters_.setAlfa(alfa_HoltWinters_);
            holtWinters_.setBeta(beta_HoltWinters_);
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

    return ros::Time(c_out + timeOffset_ + startROSTimeBig_);

}

void TimestampSynchronizer::setTimeOffset(double timeOffset) {
    TimestampSynchronizer::timeOffset_ = timeOffset;
}

void TimestampSynchronizer::setMedianFilter(bool useMedianFilter, int medianFilterWindow) {
    TimestampSynchronizer::useMedianFilter_ = useMedianFilter;
    TimestampSynchronizer::medianFilterWindow_ = medianFilterWindow;
    initMediator();
}

void TimestampSynchronizer::setUseHoltWinters(bool useHoltWinters) {
    TimestampSynchronizer::useHoltWinters_ = useHoltWinters;
}

void TimestampSynchronizer::setAlfa_HoltWinters(double alfa_HoltWinters) {
    TimestampSynchronizer::alfa_HoltWinters_ = alfa_HoltWinters;
}

void TimestampSynchronizer::setBeta_HoltWinters(double beta_HoltWinters) {
    TimestampSynchronizer::beta_HoltWinters_ = beta_HoltWinters;
}

void TimestampSynchronizer::setAlfa_HoltWinters_early(double alfa_HoltWinters_early) {
    TimestampSynchronizer::alfa_HoltWinters_early_ = alfa_HoltWinters_early;
}

void TimestampSynchronizer::setBeta_HoltWinters_early(double beta_HoltWinters_early) {
    TimestampSynchronizer::beta_HoltWinters_early_ = beta_HoltWinters_early;
}

void TimestampSynchronizer::reset() {
    init();
}

void TimestampSynchronizer::initMediator() {
    if(mediator_ != nullptr) {
        delete mediator_;
    }
    mediator_ = new Mediator<double>(medianFilterWindow_);
}

void TimestampSynchronizer::init() {
    firstFrameSet_ = false;
    initMediator();
    holtWinters_.reset(initialB_HoltWinters_);
}

void TimestampSynchronizer::setEarlyClamp(bool earlyClamp, int earlyClampWindow) {
    TimestampSynchronizer::earlyClamp_ = earlyClamp;
    TimestampSynchronizer::earlyClampWindow_ = earlyClampWindow;
}