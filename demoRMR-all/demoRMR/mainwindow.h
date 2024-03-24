#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include<windows.h>
#include "collision_detection.h"
#endif
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
//#include "ckobuki.h"
//#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "robot.h"
#include <QJoysticks.h>

#include "controller.h"
#include "point.h"
#include "ramp.h"

typedef struct
{
    int speed;
    int radius;
}MovementsParam;

namespace Ui {
class MainWindow;
}

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    
    MovementsParam param;

    bool useCamera1;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];

    cv::Mat frame[3];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);

int processThisCamera(cv::Mat cameraData);

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();
    void on_pushButton_7_clicked();


    void on_pushButton_clicked();
    void getNewFrame();

    void on_pushButton_10_clicked();

    void on_pushButton_11_clicked();

private:

    void addPointAtStart(Point p);
    double calculateEncoderDelta(int prev, int actual);
    bool isThereObstacleInZone();
    //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
    Ui::MainWindow *ui;
    void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
    int updateLaserPicture;
    LaserMeasurement copyOfLaserData;
    std::string ipaddress;
    Robot robot;

    std::shared_ptr<PIController> controller;
    std::shared_ptr<Point> actual_point;
    std::shared_ptr<Point> set_point;
    std::shared_ptr<Point> desired_point;
    CollisionDetection collision_detection;
    

    std::vector<Point> points_vector;
    //vektor bude mat v sebe body, ktore, ked ich budes pridavat manualne tak sa pridaju appendom nakoniec
    //robot bude prechadzat bodmi tak, ze vzdy pojde na nulty bod vo vektore, akonahle sa tam dostane sa tento bod odstrani z vektora
    //robot bude chodit na body, len v pripade, ze vektor nie je prazdny
    //v pripade, ze by sa robot na bod nemohol dostat a chceli by sme mu dat nejaky medzibod tak sa prida pred neho (na zaciatok)
    //do controllera stale bude vstupovat actual point (ten je stale updatovany v callbacku) a desired point (ten sa bude menit podla bodov vektora)
     
    TKobukiData robotdata;
    int datacounter;
    QTimer *timer;

    QJoysticks *instance;

    double forwardspeed;//mm/s
    double rotationspeed;//omega/s

    int prev_x;
    int prev_y;
    int prev_gyro;
    bool first_run;

    bool bruh;
    int start_left;
    int start_right;
    int start_gyro;

    int prev_left;
    int prev_right;

    double delta_wheel_left;
    double delta_wheel_right;

    double robotX;
    double robotY;
    double robotFi;
    double prev_fi;

    bool rot_only;

    bool line_following;
    bool trajectory_clear;
    double shortest_distance_to_goal;
    double current_distance_to_goal;

public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo


};

#endif // MAINWINDOW_H
