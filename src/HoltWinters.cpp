#include "HoltWinters.h"

HoltWintersSmoothFilter::HoltWintersSmoothFilter( double initialVel, double alfa, double beta) {
    alfa_ = alfa;
    beta_ = beta;
    reset(initialVel);
}

// Inserts item
void HoltWintersSmoothFilter::insert(const double& v )
{
    double s_prev = s_;
    if(!gotFirst_) {
        s_ = v;
        gotFirst_ = true;
    }
    else {
        s_ = alfa_ * v + (1 - alfa_) * (s_ + b_);
        b_ = beta_ * (s_ - s_prev) + (1 - beta_) * b_;
    }
}

// Returns current smoothed value
double HoltWintersSmoothFilter::getFiltered() {
    return s_;
}

void HoltWintersSmoothFilter::reset(double initialVel) {
    gotFirst_ = false;
    b_ = initialVel;
}

void HoltWintersSmoothFilter::setAlfa(double alfa) {
    HoltWintersSmoothFilter::alfa_ = alfa;
}

void HoltWintersSmoothFilter::setBeta(double beta) {
    HoltWintersSmoothFilter::beta_ = beta;
}
