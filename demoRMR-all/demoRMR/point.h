#ifndef POINT_H
#define POINT_H

class Point {
public:
    Point(){}; // Default constructor
    Point(double x, double y, double theta){this->x = x; this->y = y;this->theta = theta;}; // Parameterized constructor

    void setPoint(double x, double y, double theta){this->x = x; this->y = y; this->theta = theta;}; // Setter for both x and y coordinates
    void getPoint(double *x, double *y, double *theta){*x = this->x; *y = this->y; *theta = this->theta;}; // Getter for both x and y coordinates
    void setPoint(Point *p){this->x = p->getX(); this->y = p->getY(); this->theta = p->getTheta();}; // Setter for both x and y coordinates
    double getX(){return x;}; // Getter for the difference in x coordinates
    double getY(){return y;}; // Getter for the difference in y coordinates
    double getTheta(){return  theta;}; // Getter for the difference in angle

private:
    double x; // x coordinate
    double y; // y coordinate
    double theta; // angle
};
#endif
