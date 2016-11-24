#ifndef TIMESTAMPSYNCHRONIZER_H
#define TIMESTAMPSYNCHRONIZER_H

#include <ros/ros.h>
#include "HoltWinters.h"
#include "Mediator.h"
#include <cmath>
#include <string>

class TimestampSynchronizer
{
public:
    TimestampSynchronizer(std::string name = std::string() );
    ~TimestampSynchronizer();

    ros::Time sync(double c_sensor, double c_ros_big, unsigned int frameID);

    void setTimeOffset(double timeOffset);
    void setMedianFilter(bool useMedianFilter, int medianFilterWindow = 0);
    void setUseHoltWinters(bool useHoltWinters);
    void setAlfa_HoltWinters(double alfa_HoltWinters);
    void setBeta_HoltWinters(double beta_HoltWinters);

    // gain scheduling in the Holt-Winters filter for faster initialization
    void setAlfa_HoltWinters_early(double alfa_HoltWinters_early);
    void setBeta_HoltWinters_early(double beta_HoltWinters_early);

    void setEarlyClamp(bool earlyClamp, int earlyClampWindow = 0);

    void reset();

private:
    double p_sensor_, p_ros_, p_out_;
    unsigned int firstFrameID_;

    double startROSTimeBig_;
    double startSensorTime_;
    Mediator<double>* mediator_;
    HoltWintersSmoothFilter holtWinters_;

    double timeOffset_;
    bool firstFrameSet_;
    bool useMedianFilter_;
    bool useHoltWinters_;
    bool earlyClamp_;
    int earlyClampWindow_;

    double alfa_HoltWinters_;
    double beta_HoltWinters_;
    double alfa_HoltWinters_early_;
    double beta_HoltWinters_early_;
    double initialB_HoltWinters_;
    int medianFilterWindow_;

    void init();
    void initMediator();
    ros::NodeHandle *pn_;
};

#endif
