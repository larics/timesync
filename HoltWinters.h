#ifndef HOLTWINTERS_H
#define HOLTWINTERS_H

class HoltWintersSmoothFilter
{
public:
    HoltWintersSmoothFilter(double _alfa = 0.999, double _beta = 0.9999, double _initialVel = -3e-7) :
            alfa(_alfa), beta(_beta), b(_initialVel), gotFirst(false){ };

    // Inserts item
    void insert(const double& v )
    {
        double s_prev = s;
        if(!gotFirst) {
            s = v;
            gotFirst = true;
        }
        else {
            s = (1 - alfa) * v + alfa * (s + b);
            b = (1 - beta) * (s - s_prev) + beta * b;
        }
    };

    // Returns current smoothed value
    double getFiltered() {
        return s;
    }

    void setAlfa(double alfa) {
        HoltWintersSmoothFilter::alfa = alfa;
    }

    void setBeta(double beta) {
        HoltWintersSmoothFilter::beta = beta;
    }

private:
    bool gotFirst;
    double b, s;
    double alfa, beta;
};

#endif
