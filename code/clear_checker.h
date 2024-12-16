#ifndef CLEAR_CHECKER
#define CLEAR_CHECKER

#include <vector>
#include <cmath>

struct Segment1
{
    Segment1(double p0_x, double p0_y, double p1_x, double p1_y) : x0(p0_x), y0(p0_y), x1(p1_x), y1(p1_y)
    {
    }
    double x0, y0, x1, y1;
};


///struct I guess we already have 
double centre_clear(const double x, const double y);
double line_to_line(const Segment1 seg);
double link_clearance(double x, double y, double theta0, double theta1, double theta2, double theta3, double theta4);
double kammkabanda(const Segment1 &seg1 , const Segment1 &segbase);
double pointDistance(double x1, double y1, double x2, double y2);
#endif