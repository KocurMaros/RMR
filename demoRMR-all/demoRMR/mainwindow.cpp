#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>


#define WHEELBASE 0.23 // [m]
#define WHEELRADIUS 0.035
#define TICKTOMETER 0.000085292090497737556558
#define TICKTORAD 0.002436916871363930187454
#define ENCODEROVERFLOW 65535 //dumbass :O

#define WITHIN_TOLERANCE 30
#define WITHIN_TOLERANCE_THETA 0.0174533

///TOTO JE DEMO PROGRAM...AK SI HO NASIEL NA PC V LABAKU NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
/// AK HO MAS Z GITU A ROBIS NA LABAKOVOM PC, TAK SI HO VLOZ DO FOLDERA KTORY JE JASNE ODLISITELNY OD TVOJICH KOLEGOV
/// NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
/// POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
/// KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
/// AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
/// AK SA DOSTANES NA SKUSKU



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";//192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    bruh = false;
    actIndex=-1;
    useCamera1=false;
    first_run = true;
    controller = make_shared<PIController>(3,0.1,2);
    actual_point = make_shared<Point>(0,0,0);
    set_point = make_shared<Point>(0,0,0);
    desired_point = make_shared<Point>(0,0,0);
    obstacle_avoidance_point = make_shared<Point>(0,0,0);

    robotX = 0;
    robotY = 0;
    robotFi = 0;

    prev_x = 0;
    prev_y = 0;
    prev_gyro = 0;
    prev_left = 0;
    prev_right = 0;
    datacounter=0;
    rot_only = false;
    line_following = false;
    trajectory_clear = true;
    controller->clearIntegral();
    shortest_distance_to_goal = 0;
    current_distance_to_goal = 0;
    left_point_distance = 0;
    right_point_distance = 0;
    left_point_angle = 0;
    right_point_angle = 0;
    obstacle_point_set = false;
    collision_detection.getObstacle()->setFoundObstacle(false);

    wall_following = false;
    test_collision = false;
    shortest_distance_to_goal = 10000000;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);

    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            pero.setColor(Qt::green);//farba je zelena
            painter.setPen(pero);
            updateLaserPicture=0;
            int x = rect.width()-(rect.width()/2)+rect.topLeft().x();
            int y = rect.height()-(rect.height()/2)+rect.topLeft().y();

            if (rect.contains(x,y)){
                painter.drawEllipse(QPoint(x, y),16,16);
            }
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                if (copyOfLaserData.Data[k].scanDistance!=0){
                    int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                    int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                    int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                    if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                        painter.drawEllipse(QPoint(xp, yp),2,2);
                }

                }
            }
        if(collision_detection.getObstacle()->isFoundObstacle()){
            if(collision_detection.getObstacle()->getLeftEdge()->isFoundEdge()){
                int xp= rect.width()-(rect.width()/2+collision_detection.getObstacle()->getLeftEdge()->getDistance()*100*sin((collision_detection.getObstacle()->getLeftEdge()->getAngle())*3.14159/180.0))+rect.topLeft().x();
                int yp = rect.height()-(rect.height()/2+collision_detection.getObstacle()->getLeftEdge()->getDistance()*100*cos((collision_detection.getObstacle()->getLeftEdge()->getAngle())*3.14159/180.0))+rect.topLeft().y();
                pero.setColor(Qt::blue); //lavy point bude modry
                painter.setPen(pero);
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);

                pero.setColor(Qt::red); //v lavom bude cerveny
                painter.setPen(pero);
                xp = rect.width()-(rect.width()/2+left_point_distance*100*sin(left_point_angle*3.14159/180.0))+rect.topLeft().x();
                yp = rect.height()-(rect.height()/2+left_point_distance*100*cos(left_point_angle*3.14159/180.0))+rect.topLeft().y();
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
        if(collision_detection.getObstacle()->isFoundObstacle()){
            if(collision_detection.getObstacle()->getRightEdge()->isFoundEdge()){
                int xp=rect.width()-(rect.width()/2+collision_detection.getObstacle()->getRightEdge()->getDistance()*100*sin((collision_detection.getObstacle()->getRightEdge()->getAngle())*3.14159/180.0))+rect.topLeft().x();
                int yp = rect.height()-(rect.height()/2+collision_detection.getObstacle()->getRightEdge()->getDistance()*100*cos((collision_detection.getObstacle()->getRightEdge()->getAngle())*3.14159/180.0))+rect.topLeft().y();
                pero.setColor(Qt::white);//pravy roh bude zlty
                painter.setPen(pero);
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);

                pero.setColor(Qt::yellow);//v pravom bude biely
                painter.setPen(pero);
                xp = rect.width()-(rect.width()/2+right_point_distance*100*sin(right_point_angle*3.14159/180.0))+rect.topLeft().x();
                yp = rect.height()-(rect.height()/2+right_point_distance*100*cos(right_point_angle*3.14159/180.0))+rect.topLeft().y();
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

double MainWindow::calculateEncoderDelta(int prev, int actual) {
    int delta;
    if (actual > 60000 && prev < 5000) {
        delta = actual - prev - ENCODEROVERFLOW;
    }
    else if (actual < 5000 && prev > 60000) {
        delta = actual - prev + ENCODEROVERFLOW;
    }
    else {
        delta = actual - prev;
    }
    return TICKTOMETER*delta;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{


    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;
    return 0;
}
void MainWindow::on_pushButton_9_clicked() //start button
{
    //ziskanie joystickov
    instance = QJoysticks::getInstance();
    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));

    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();






    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
        );
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robot.setTranslationSpeed(500);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);

}

void MainWindow::on_pushButton_6_clicked() //left
{
    robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
    robot.setRotationSpeed(-3.14159/2);

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);
    bruh = false;

}
void MainWindow::on_pushButton_7_clicked()
{
    bruh = true;
}



void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}

//testcollision button
void MainWindow::on_pushButton_11_clicked()
{
    test_collision = true;
    collision_detection.resetCollisionDetection();
}


void MainWindow::getNewFrame()
{

}



///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_click`ed
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
    if(first_run){
        start_left = robotdata.EncoderLeft;
        start_right = robotdata.EncoderRight;
        start_gyro = robotdata.GyroAngle;
        first_run = false;
        prev_left = start_left;
        prev_right = start_right;
    }
    

    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, aktualne to blokuje gombiky cize tak
    // if(instance->count()>0)
    // {
    //     if(forwardspeed==0 && rotationspeed!=0)
    //         robot.setRotationSpeed(rotationspeed);
    //     else if(forwardspeed!=0 && rotationspeed==0)
    //         robot.setTranslationSpeed(forwardspeed);
    //     else if((forwardspeed!=0 && rotationspeed!=0))
    //         robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
    //     e-0.000741283lse
    //         robot.setTranslationSpeed(0);

    // }
///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX



  //  if(datacounter%5)
    {
        delta_wheel_right = calculateEncoderDelta(prev_right, robotdata.EncoderRight); //TODO: vyhodit funkciu kvoli speed a dat kod napriamo sem? 
        delta_wheel_left = calculateEncoderDelta(prev_left, robotdata.EncoderLeft);
        robotFi = robotFi + (delta_wheel_right - delta_wheel_left) / WHEELBASE/PI*180.0;
        if (robotFi >= 180){
            robotFi = robotFi - 360;
        }
        else if (robotFi < -180) {
            robotFi = robotFi + 360;
        }
        if (delta_wheel_left == delta_wheel_right) {
            robotX = robotX + (delta_wheel_left + delta_wheel_right)/2*cos(robotFi*PI/180.0);
            robotY = robotY + (delta_wheel_left + delta_wheel_right)/2*sin(robotFi*PI/180.0);
        }
        else {
            robotX = robotX + (delta_wheel_right+delta_wheel_left)/(delta_wheel_right-delta_wheel_left)*WHEELBASE/2*(sin(robotFi*PI/180.0)-sin(prev_fi*PI/180.0));
            robotY = robotY - (delta_wheel_right+delta_wheel_left)/(delta_wheel_right-delta_wheel_left)*WHEELBASE/2*(cos(robotFi*PI/180.0)-cos(prev_fi*PI/180.0));
        }




        emit uiValuesChanged(robotX,robotY,robotFi);
        prev_right=robotdata.EncoderRight;
        prev_left=robotdata.EncoderLeft;
        prev_fi = robotFi;

        actual_point->setPoint(robotX*1000, robotY*1000, robotFi*PI/180.0);

        if (bruh) {

            double rot_speed;
            int trans_speed, radius;

            if (!points_vector.empty()){
                desired_point->setPoint(points_vector[0].getX(),points_vector[0].getY(),0);
            }
            else {
                bruh = false;
                return 0;
            }
            //toto vzdy nastavi ciel, ak je vo vektore bodov aspon jeden bod
            controller->computeErrors(*actual_point,*desired_point);
            current_distance_to_goal = controller->error_distance;

            // TODO: if (wall_following) ..., else: tato srandicka
            // prvykrat najdes minimum, ked bolo po lavej strane od teba followujes stenu vlavo, inak v pravo...
            // ked najdes to minimum tak vypocias bod kam sa ma dostat a natocis sa 90 stupnov vpravo/vlavo, vynulujes flag na first_wall alebo tak


            //TODO: threshold for shortest distance
            if (current_distance_to_goal < shortest_distance_to_goal){
                shortest_distance_to_goal = current_distance_to_goal;
            }
            else {
                //TODO: logic for wall following
                // wall_following = true;
                // return 0;
            }

            //TODO: ak je prejazdovy bod true, tak do controllera pacni ten, ak ne tak pacni aktualny
            if (obstacle_point_set){
                controller->computeErrors(*actual_point,*obstacle_avoidance_point);
            }

            //checknut ci sa da ist na cielovy bod a ci neni wall following
            //ak ano chod na ciel

            //ak si vo wall followingu, followuj stenu az dokym nevidis na ciel a zaroven nejsi blizsie od naposledy zapamatanej pozicie

            //ked je pred tebou prekazka a nie si vo wall followingu, smeruj na jej hranu, zapamataj si aktualnu vzdialenost od ciela
            //checkuj si vzdialenost, ak sa zacne niekedy zvacsovat prepni sa na sledovanie steny

            //check if the path is right

            if (!collision_detection.getObstacle()->isFoundObstacle()){
                // std::cout<< "Obstacle: " << collision_detection.getObstacle()->isFoundObstacle() << std::endl;
                if(isThereObstacleInZone(controller->error_angle/PI*180,controller->error_distance/1000.0)){

                    // std::cout<< "Obstacle has been found!" << std::endl;
                    findEdgeLeft();

                    if(collision_detection.getObstacle()->getLeftEdge()->isFoundEdge()){
                        // std::cout << "Obstacle left edge has been found!" << std::endl;
                        collision_detection.getObstacle()->calculateLeftEdgePoint(robotX,robotY,robotFi);
                        if (!checkLeftEdgePointObstacle()){
                            // std::cout << "there is no obstacle brother" << std::endl;
                            collision_detection.getObstacle()->getLeftEdge()->setPointFree(true);
                            collision_detection.getObstacle()->getLeftEdge()->setDistanceToGoal(calculateDistanceToGoal(actual_point,desired_point,collision_detection.getObstacle()->getLeftEdge()->getPoint()));
                        }
                        else{
                            collision_detection.getObstacle()->getLeftEdge()->setPointFree(false);
                        }
                    }

                    findEdgeRight();
                    if(collision_detection.getObstacle()->getRightEdge()->isFoundEdge()){
                        // std::cout << "Obstacle right edge has been found!" << std::endl;
                        collision_detection.getObstacle()->calculateRightEdgePoint(robotX,robotY,robotFi);
                        if (!checkRightEdgePointObstacle()){
                            // std::cout << "there is no obstacle brother2" << std::endl;
                            collision_detection.getObstacle()->getRightEdge()->setPointFree(true);
                            collision_detection.getObstacle()->getRightEdge()->setDistanceToGoal(calculateDistanceToGoal(actual_point,desired_point,collision_detection.getObstacle()->getRightEdge()->getPoint()));
                        }
                        else{
                            collision_detection.getObstacle()->getRightEdge()->setPointFree(false);
                        }
                    }

                    if(collision_detection.getObstacle()->getRightEdge()->isPointFree() && collision_detection.getObstacle()->getLeftEdge()->isPointFree()){
                        if(collision_detection.getObstacle()->getRightEdge()->getDistanceToGoal() < collision_detection.getObstacle()->getLeftEdge()->getDistanceToGoal()){
                            //set right point
                            obstacle_avoidance_point->setPoint(collision_detection.getObstacle()->getRightEdge()->getPoint());
                            obstacle_point_set = true;
                        }
                        else {
                            //set left point
                            obstacle_avoidance_point->setPoint(collision_detection.getObstacle()->getLeftEdge()->getPoint());
                            obstacle_point_set = true;
                        }
                        collision_detection.resetCollisionDetection();
                    }
                    else if(collision_detection.getObstacle()->getLeftEdge()->isPointFree()){
                        //set left point
                        obstacle_avoidance_point->setPoint(collision_detection.getObstacle()->getLeftEdge()->getPoint());
                        obstacle_point_set = true;
                        collision_detection.resetCollisionDetection();
                    }
                    else if(collision_detection.getObstacle()->getRightEdge()->isPointFree()){
                        //set right point
                        obstacle_avoidance_point->setPoint(collision_detection.getObstacle()->getRightEdge()->getPoint());
                        obstacle_point_set = true;
                        collision_detection.resetCollisionDetection();
                    }
                    else {
                        // follow wall TODO:
                    }
                    std::cout << "finished this checking" << std::endl;
                    return 0;
                }
            }
            else{
                //TODO: ?
                return 0;
            }

            if (!obstacle_point_set){
                controller->compute(*actual_point,*desired_point,(double)1/40, &trans_speed, &rot_speed);

                if(abs(controller->error_distance) < WITHIN_TOLERANCE){
                    controller->clearIntegral();
                    robot.setTranslationSpeed(0);
                    controller->ramp.clear_time_hard();
                    obstacle_point_set = false;
                    collision_detection.resetCollisionDetection();
                    if (!points_vector.empty()){
                        //toto asi nemusi byt v ife - just to be sure
                        points_vector.erase(points_vector.begin());
                    }
                    std::cout << "clear integral" << std::endl;
                    return 0;
                }
            }
            else {
                controller->compute(*actual_point,*obstacle_avoidance_point,(double)1/40, &trans_speed, &rot_speed);

                if(abs(controller->error_distance) < WITHIN_TOLERANCE){
                    controller->clearIntegral();
                    controller->ramp.clear_time_hard();
                    collision_detection.resetCollisionDetection();
                    obstacle_point_set = false;
                    return 0;
                }
            }



            if (abs(controller->error_angle) >= PI/4 && !rot_only){
                //ak je uhol moc velky nastavi sa flag na rotaciu na mieste
                rot_only = true;
                controller->ramp.clear_time_hard();
                collision_detection.resetCollisionDetection();
                controller->clearIntegral();
                std::cout << "ONLY ROT: " << controller->error_angle << std::endl;
                std::cout << "Actual Theta: " << actual_point->getTheta() << std::endl;
                std::cout << "Desired Theta: " << atan2(desired_point->getY()-actual_point->getY(),desired_point->getX()-actual_point->getX()) << std::endl;
            }

            if (rot_only){
                //ROTATION
                robot.setRotationSpeed(rot_speed);

            }else{
                //ARC
                radius = trans_speed/rot_speed;
                if(radius > 32767)
                    radius = 32767;
                else if(radius < -32767)
                    radius = -32767;
                robot.setArcSpeed(trans_speed,radius);
            }
            if (rot_only && abs(controller->error_angle)<=4*PI/180){
                rot_only = false;
                controller->ramp.clear_time_hard();
                controller->clearIntegral();
            }



        } //if bruh koniec

        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    datacounter++;


    return 0;

}



void MainWindow::addPointAtStart(Point p) {
    points_vector.insert(points_vector.begin(),p);
}

void MainWindow::on_pushButton_10_clicked()
{
    // ui->lineEdit_5 is X
    // ui->lineEdit_6 is Y
    bool xOK = true,yOK = true;
    double x = ui->lineEdit_5->text().toDouble(&xOK);
    double y = ui->lineEdit_6->text().toDouble(&yOK);
    if (xOK && yOK){
        Point point(x*1000,y*1000,0*PI/180);
        points_vector.push_back(point);
        std::cout << "vector: ";
        for (auto &p : points_vector) {
            std::cout<< "X: " << p.getX() << " Y:" << p.getY() << std::endl;
        }
    }
    else {
        std::cout << "incorrect input!" << std::endl;
    }

}

bool MainWindow::isThereObstacleInZoneStatic(double zoneAngle, double zoneDistance){
    for(int k=0;k<collision_detection.getLaserData().numberOfScans/*360*/;k++){
        if(CollisionDetection::isObstacleInPathStatic(collision_detection.getLaserData().Data[k].scanDistance/1000.0,collision_detection.getLaserData().Data[k].scanAngle,zoneAngle,zoneDistance)){
            std::cout << "Obstacle in the zone" << std::endl;
            return true;
        }
    }
    return false;
}

bool MainWindow::isThereObstacleInZone(double zoneAngle, double zoneDistance) {
    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++){
        if(collision_detection.isObstacleInPath(copyOfLaserData.Data[k].scanDistance/1000.0,copyOfLaserData.Data[k].scanAngle,zoneAngle,zoneDistance)){
            collision_detection.getObstacle()->setIndex(k);
            collision_detection.getObstacle()->setFoundObstacle(true);
            std::cout << "THERE IS AN OBSTACLE, :" << collision_detection.getObstacle()->isFoundObstacle() << std::endl;
            collision_detection.setLaserData(copyOfLaserData);
            return true;
        }
    }
    return false;
}

double MainWindow::angDiff(double alpha, double beta) {
    double diff = beta - alpha;
    // Normalize the difference
    diff = fmod(diff + 180, 360);
    if (diff < 0)
        diff += 360;
    // diff -= 180;
    return fabs(diff);
}


void MainWindow::findEdgeLeft(){
    collision_detection.getObstacle()->getLeftEdge()->setPointFree(false);

    double angle_difference = 0;
    double prev_distance = collision_detection.getObstacle()->getDistance();
    double prev_angle = collision_detection.getObstacle()->getAngle();
    //zaciname uz o jeden dalej preto index - 1
    collision_detection.getObstacle()->getLeftEdge()->setFoundEdge(false);
    int lidar_index = collision_detection.getObstacle()->getIndex()-1;
    //lidar je pravotocivy -> ked hladam nalavo tak musim zmensit index
    //distance je v metroch

    double obstacle_angle = collision_detection.getObstacle()->getAngle();

    std::cout << "CHECKING LEFT EDGE" << std::endl;

    while(angle_difference < 180){
        if(lidar_index < 0){
            lidar_index = collision_detection.getLaserData().numberOfScans - 1;
        }

        double lidar_angle = CollisionDetection::normalizeLidarAngle(collision_detection.getLaserData().Data[lidar_index].scanAngle);
        double lidar_distance = collision_detection.getLaserData().Data[lidar_index].scanDistance/1000.0;

        if(((lidar_distance - prev_distance) > Obstacle::distanceThreshold) || (lidar_distance == 0.0)){

            //TODO: vratit roh predoslej vzdialenosti a predosleho uhla asi zejo
            collision_detection.getObstacle()->getLeftEdge()->setDistance(prev_distance);
            collision_detection.getObstacle()->getLeftEdge()->setAngle(prev_angle);
            collision_detection.getObstacle()->getLeftEdge()->setFoundEdge(true);
            std::cout << "LEFT EDGE FOUND" << std::endl;
            break;
        }

        prev_distance = lidar_distance;
        prev_angle = lidar_angle;

        angle_difference = angDiff(obstacle_angle, lidar_angle);
        std::cout << "angle_difference: " << angle_difference << std::endl;
        lidar_index--;
    }
    std::cout << "left edge not found" << std::endl;
}

void MainWindow::findEdgeRight(){
    collision_detection.getObstacle()->getRightEdge()->setPointFree(false);

    double angle_difference = 0;
    double prev_distance = collision_detection.getObstacle()->getDistance();
    double prev_angle = collision_detection.getObstacle()->getAngle();
    //zaciname uz o jeden dalej preto index - 1
    collision_detection.getObstacle()->getRightEdge()->setFoundEdge(false);
    int lidar_index = collision_detection.getObstacle()->getIndex()+1;
    //lidar je pravotocivy -> ked hladam napravo tak musim zvacsit index
    //distance je v metroch
    double obstacle_angle = collision_detection.getObstacle()->getAngle();

    std::cout << "CHECKING RIGHT EDGE" << std::endl;

    while(angle_difference < 180){
        if(lidar_index > (collision_detection.getLaserData().numberOfScans - 1)){
            lidar_index = 0;
        }
        //posunut o 2PI nech sa nestane ze by to prestalo checkovat ked to prejde cez 180
        double lidar_angle = CollisionDetection::normalizeLidarAngle(collision_detection.getLaserData().Data[lidar_index].scanAngle);
        double lidar_distance = collision_detection.getLaserData().Data[lidar_index].scanDistance/1000.0;

        if((lidar_distance - prev_distance > Obstacle::distanceThreshold) || lidar_distance == 0.0){

            //TODO: vratit roh predoslej vzdialenosti a predosleho uhla asi zejo
            collision_detection.getObstacle()->getRightEdge()->setDistance(prev_distance);
            collision_detection.getObstacle()->getRightEdge()->setAngle(prev_angle);
            collision_detection.getObstacle()->getRightEdge()->setFoundEdge(true);
            std::cout << "RIGHT EDGE FOUND" << std::endl;
            break;
        }

        prev_distance = lidar_distance;
        prev_angle = lidar_angle;

        angle_difference = angDiff(obstacle_angle, lidar_angle);
        lidar_index++;
    }
}

bool MainWindow::checkLeftEdgePointObstacle(){
    double distance = sqrt(pow(collision_detection.getObstacle()->getLeftEdge()->getPoint()->getX()/1000.0- robotX, 2) + pow(collision_detection.getObstacle()->getLeftEdge()->getPoint()->getY()/1000.0 - robotY, 2));

    double angle = atan2(collision_detection.getObstacle()->getLeftEdge()->getPoint()->getY()/1000.0 - robotY, collision_detection.getObstacle()->getLeftEdge()->getPoint()->getX()/1000.0 - robotX) - robotFi*PI/180;
    if (angle > PI) {
        angle -= 2 * PI;
    } else if (angle <= -PI) {
        angle += 2 * PI;
    }
    left_point_distance = distance;
    left_point_angle = angle/PI*180;
    std::cout << "angle: " << angle/PI*180 << " distance: " << distance << std::endl;
    return isThereObstacleInZoneStatic(angle/PI*180,distance);
}

bool MainWindow::checkRightEdgePointObstacle(){
    double distance = sqrt(pow(collision_detection.getObstacle()->getRightEdge()->getPoint()->getX()/1000.0- robotX, 2) + pow(collision_detection.getObstacle()->getRightEdge()->getPoint()->getY()/1000.0 - robotY, 2));

    double angle = atan2(collision_detection.getObstacle()->getRightEdge()->getPoint()->getY()/1000.0 - robotY, collision_detection.getObstacle()->getRightEdge()->getPoint()->getX()/1000.0 - robotX) - robotFi*PI/180;
    if (angle > PI) {
        angle -= 2 * PI;
    } else if (angle <= -PI) {
        angle += 2 * PI;
    }
    right_point_distance = distance;
    right_point_angle = angle/PI*180;
    std::cout << "angle: " << angle/PI*180 << " distance: " << distance << std::endl;
    return isThereObstacleInZoneStatic(angle/PI*180,distance);}

double MainWindow::calculateDistanceToGoal(std::shared_ptr<Point> currentPoint, std::shared_ptr<Point> goalPoint, Point *midPoint){
    double first_distance = sqrt(pow(currentPoint->getX()-midPoint->getX(), 2) + pow(currentPoint->getY()-midPoint->getY(), 2));
    double second_distance = sqrt(pow(midPoint->getX()-goalPoint->getX(), 2) + pow(midPoint->getY()-goalPoint->getY(), 2));
    return first_distance+second_distance;
}

double MainWindow::calculateDistanceToGoal(std::shared_ptr<Point> currentPoint, std::shared_ptr<Point> goalPoint){
    return sqrt(pow(currentPoint->getX()-goalPoint->getX(), 2) + pow(currentPoint->getY()-goalPoint->getY(), 2));
}




