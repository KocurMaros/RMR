#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>

#define WHEELBASE 0.23
#define WHEELRADIUS 0.035
#define TICKTOMETER 0.000085292090497737556558
#define TICKTORAD 0.002436916871363930187454
#define ENCODEROVERFLOW 65536

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
    // ipaddress="192.168.1.15";//192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
    ipaddress="127.0.0.1";
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
    prev_x_map = -1000;
    prev_y_map = -1000;
    set_point = make_shared<Point>(0,0,0);
    desired_point = make_shared<Point>(0,0,0);
    maps = make_shared<Mapping>();
    // path = make_shared<Pathfinding>();
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
    controller->clearIntegral();
    mapping = 0;
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
    if(!mapping_start){
        if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
        {
            // std::cout<<actIndex<<std::endl;
            QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
            painter.drawImage(rect,image.rgbSwapped());
        }
        else
        {
            if(updateLaserPicture==1) ///ak mam nove data z lidaru
            {
                updateLaserPicture=0;

                painter.setPen(pero);
                //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
            //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
                for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
                {
                    int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                    int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                    int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                    if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                        painter.drawEllipse(QPoint(xp, yp),2,2);
                }
                int xrobot = rect.width() / 2;
                int yrobot = rect.height() / 2;
                int xpolomer = 20;
                int ypolomer = 20;

                painter.drawEllipse(QPoint(rect.x() + xrobot, rect.y() + yrobot), xpolomer, ypolomer);
                painter.drawLine(rect.x() + xrobot, rect.y() + yrobot, rect.x() + xrobot + xpolomer * cos((360 - 90) * 3.14159 / 180),
                                rect.y() + ((yrobot + ypolomer * sin((360 - 90) * 3.14159 / 180))));
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

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
    if(first_run){
        start_left = robotdata.EncoderLeft;
        start_right = robotdata.EncoderRight;
        start_gyro = 1.0/100.0*robotdata.GyroAngle;
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
        robotFi = 1.0*robotdata.GyroAngle/100.0 - start_gyro;
        if (robotFi >= 180){
            robotFi = robotFi - 360;
        }
        else if (robotFi < -180) {
            robotFi = robotFi + 360;
        }
        robotX = robotX + (delta_wheel_left + delta_wheel_right)/2*cos(robotFi*PI/180.0);
        robotY = robotY + (delta_wheel_left + delta_wheel_right)/2*sin(robotFi*PI/180.0);



        // std::cout << "encoder: " << robotdata.EncoderLeft << std::endl;

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
                // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
                //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
                //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
                /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
                /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
                /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit uiValuesChanged(robotX,robotY,robotFi);
        prev_right=robotdata.EncoderRight;
        prev_left=robotdata.EncoderLeft;
        prev_fi = robotFi;

        actual_point->setPoint(robotX*1000, robotY*1000, robotFi*PI/180.0);
        if (bruh || mapping_start) {
            double rot_speed;
            int trans_speed, radius;
            if (!points_vector.empty()){
                desired_point->setPoint(points_vector[0].getX(),points_vector[0].getY(),0);
            }
            else{
                bruh = false;
                return 0; //break maybe?
            }

            double error_distance = sqrt(pow(actual_point->getX() - prev_x_map, 2) + pow(actual_point->getY() - prev_y_map, 2));
            if(m_rot_speed < 0.01 && error_distance > 500 ){
                m_can_map = true;
                prev_x_map = actual_point->getX();
                prev_y_map = actual_point->getY();
            }
 
            controller->compute(*actual_point,*desired_point,(double)1/40, &trans_speed, &rot_speed);
            m_rot_speed = rot_speed;

            
            if(abs(controller->error_distance) < WITHIN_TOLERANCE){
                mapping++;
                controller->clearIntegral();
                robot.setTranslationSpeed(0);
                controller->ramp.clear_time_hard();
                if (!points_vector.empty()){
                    //toto asi nemusi byt v ife - just to be sure
                    points_vector.erase(points_vector.begin());
                }
                return 0;   
            }

            if (abs(controller->error_angle) >= PI/4 && !rot_only){
                rot_only = true;
                controller->ramp.clear_time_hard();
                controller->clearIntegral();
            }
            if (rot_only){
                robot.setRotationSpeed(rot_speed);
            }else{
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
        }

        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    // if(mapping == 10 && mapping_start){

    // if(mapping == 9 && mapping_start){
    if(save_map){
        cout << "Mapping finished" << endl;
        mapping_start = false;
        maps->save_map();
        cout << "Mapping saved" << endl;
        maps->print_map();
        save_map = false;
    }
    datacounter++;
    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
 int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    // double error_distance = sqrt(pow(actual_point->getX() - prev_x_map, 2) + pow(actual_point->getY() - prev_y_map, 2));
    if(mapping_start){
        if(m_can_map){
            m_can_map = false;
            // cout << "Rot speed " << m_rot_speed << " dis " << error_distance << endl;
            maps->Gmapping(copyOfLaserData, robotX, robotY, robotFi);
            cout << "Updating map from lidar " << endl;
            // prev_x_map = actual_point->getX();
            // prev_y_map = actual_point->getY();
        }
    }
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
    if(ipaddress=="127.0.0.1")
        robot.setCameraParameters("http://"+ipaddress+":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
    else
        robot.setCameraParameters("http://"+ipaddress+":8000/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
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
void MainWindow::on_pushButton_mapping_clicked(){
    if(!mapping_start){
        ui->pushButton_mapping->setText("stop mapping");
        mapping_start = true;
        save_map = false;
        points_vector.push_back(Point(0*1000,0*1000,0));
        points_vector.push_back(Point(0*1000,3.8*1000,0));
        points_vector.push_back(Point(4*1000,3.8*1000,0));
        points_vector.push_back(Point(3*1000,3.8*1000,0));
        points_vector.push_back(Point(3*1000,0.8*1000,0));
        points_vector.push_back(Point(5*1000,0.8*1000,0));
        points_vector.push_back(Point(3*1000,0.8*1000,0));
        points_vector.push_back(Point(3*1000,-1.0*1000,0));
        points_vector.push_back(Point(2*1000,-1.0*1000,0));
        points_vector.push_back(Point(1.5*1000,-1.0*1000,0));
    }else{
        ui->pushButton_mapping->setText("start mapping");
        mapping_start = false;
        save_map = true;
    }
    // std::cout << "Mapping started" << std::endl;
    // mapping = 0;
    // mapping_start = true;
}
void MainWindow::on_pushButton_loadMap_clicked(){

    // maps->load_map();

    
    bool xOK = true,yOK = true;
    double x = ui->lineEdit_5->text().toDouble(&xOK);
    double y = ui->lineEdit_6->text().toDouble(&yOK);
    if (xOK && yOK){
        cout << "Loading map" << endl;
        maps->load_map();
        maps->print_map();
        cout << "Map loaded" << endl;
        Point point(x*100,y*100,0*PI/180);
        std::vector<Point> trajectory = maps->flood_fill(Point(robotX*100,robotY*100,0),point);
        for(auto &p : trajectory){
            points_vector.push_back(p);
            cout << "X: " << p.getX() << " Y: " << p.getY() << endl;
        }
        maps->print_map();
        bruh = true;
    }
    else {
        std::cout << "incorrect input!" << std::endl;
    }
}
void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robot.setTranslationSpeed(200);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-200);

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
void MainWindow::on_pushButton_7_clicked() //arc left
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

void MainWindow::getNewFrame()
{

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

