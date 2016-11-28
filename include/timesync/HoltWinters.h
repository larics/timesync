#ifndef HOLTWINTERS_H
#define HOLTWINTERS_H
#include <ros/ros.h>

class HoltWintersSmoothFilter
{
public:
    HoltWintersSmoothFilter( double initialVel = defaultInitialVel_, double alfa = defaultAlfa_, double beta = defaultBeta_);
    // Inserts item
    void insert(const double& v );

    // Returns current smoothed value
    double getFiltered();

    void reset(double initialVel = defaultInitialVel_);
    void setAlfa(double alfa);
    void setBeta(double beta);

private:

    static constexpr double defaultAlfa_ = 1e-3;
    static constexpr double defaultBeta_ = 1e-4;
    static constexpr double defaultInitialVel_ = -3e-7;

    bool gotFirst_;
    double b_, s_;
    double alfa_, beta_;
};

#endif
