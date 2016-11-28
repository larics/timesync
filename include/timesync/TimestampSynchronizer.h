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
    class Options;

    TimestampSynchronizer(const Options defaultOptions = Options());

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

    class Options {
    public:
        Options() { };
        bool useMedianFilter_ = true;
        int medianFilterWindow_ = 3000;
        bool useHoltWinters_ = true;
        double alfa_HoltWinters_ = 1e-3;
        double beta_HoltWinters_ = 1e-4;
        double alfa_HoltWinters_early_ = 1e-1;
        double beta_HoltWinters_early_ = 1e-2;
        bool earlyClamp_ = true;
        int earlyClampWindow_ = 500;
        double timeOffset_ = 0.0;
        double initialB_HoltWinters_ = -3e-7;
        std::string nameSuffix = std::string();
    };


private:
    double p_sensor_, p_ros_, p_out_;
    unsigned int firstFrameID_;

    double startROSTimeBig_;
    double startSensorTime_;
    std::unique_ptr<Mediator<double> > pmediator_;
    HoltWintersSmoothFilter holtWinters_;

    bool firstFrameSet_;

    void init();
    void initMediator();
    std::unique_ptr<ros::NodeHandle> pn_;
    Options options;
};

#endif
