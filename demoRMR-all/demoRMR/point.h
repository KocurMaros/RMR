#ifndef POINT_H
#define POINT_H

class Point {
public:
    Point(); // Default constructor
    Point(double x, double y, double theta){actual_x = x; actual_y = y;actual_theta = theta;}; // Parameterized constructor

    void setPointDesire(double x, double y, double theta){desire_x = x; desire_y = y; desire_theta = theta;}; // Setter for both x and y coordinates
    void setPointActual(double x, double y, double theta){actual_x = x; actual_y = y; actual_theta = theta;}; // Setter for both x and y coordinates
    void getPointDesire(double *x, double *y, double *theta){x = desire_x; y = desire_y; theta = desire_theta;}; // Getter for both x and y coordinates
    void getPointActual(double *x, double *y, double *theta){x = actual_x; y = actual_y; theta = actual_theta;}; // Getter for both x and y coordinates
private:
    double desire_x; // x coordinate
    double desire_y; // y coordinate
    double desire_theta; // angle
    double actual_x; // x coordinate
    double actual_y; // y coordinate
    double actual_theta; // angle
};
#endif
