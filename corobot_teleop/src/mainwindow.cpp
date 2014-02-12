/*
 * Copyright (c) 2009, CoroWare
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Stanford U. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

#ifdef Q_WS_WIN
#include "windows.h"
#include "mmsystem.h"
#endif


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //set the parent widgets to the different widget created
    r.setParent(this);
    ui->setupUi(this);
    arm.setParent(ui->frame);
    arm_rotation.setParent(ui->frame_6);
    gripper.setParent(ui->frame_7);
    wrist.setParent(ui->frame_8);
    gps.setParent(ui->webView);
    gps.setList(ui->listWidget);
    hokuyo.setParent(ui->frame_hokuyo);
    gpsUrlTImer.invalidate();

    ui->tabWidget_5->setFocus();

    LRF_lines_show = false;
    IR_data_show = false;


    //connect signals to slots.

    connect(ui->connect,SIGNAL(clicked()),this,SLOT(connect_clicked()));//connect button pushed


    //***************************************************************************
    // Arm
    connect(&arm,SIGNAL(shoulderAngle_rad(double)),&r,SLOT(moveShoulderArm(double)));
    connect(&arm_rotation,SIGNAL(armAngle_rad(double)),&r,SLOT(rotateArm(double)));
    connect(&arm,SIGNAL(elbowAngle_rad(double)),&r,SLOT(moveElbowArm(double)));
    connect(ui->ArmResetButton,SIGNAL(clicked()),&r,SLOT(ResetArm()));
    connect(&r,SIGNAL(arm_model(bool,bool,bool,bool)),&arm,SLOT(setModel(bool,bool,bool,bool)));
    connect(&arm,SIGNAL(theta1(double)),ui->lcdNumber,SLOT(display(double))); //display shoulder angle
    connect(&arm,SIGNAL(theta2(double)),ui->lcdNumber_2,SLOT(display(double))); //display elbow angle
    connect(&wrist,SIGNAL(angle(double)),ui->lcdNumber_3,SLOT(display(double))); //display wrist angle
    connect(ui->radioButton_5,SIGNAL(toggled(bool)),&arm,SLOT(shoulder_degree(bool))); //choose to display shoulder angle in degree or radian
    connect(ui->radioButton_7,SIGNAL(toggled(bool)),&arm,SLOT(elbow_degree(bool)));//choose to display elbow angle in degree or radian
    connect(ui->radioButton_3,SIGNAL(toggled(bool)),&wrist,SLOT(degree(bool))); //choose to display the wrist angle in degree or radian
    connect(ui->radioButton,SIGNAL(toggled(bool)),&r,SLOT(moveGripper(bool)));//open or close the gripper
    connect(ui->radioButton_9,SIGNAL(toggled(bool)),&arm,SLOT(Corobot(bool)));//gives robot information(Corobot or Explorer) to the arm widget
    connect(&wrist,SIGNAL(angle_rad(float)),&r,SLOT(turnWrist(float))); //turn wrist if the wrist widget angle changed

    //***************************************************************************
        //MISC tab
    //***************************************************************************
    //Bumper information
    connect(&r,SIGNAL(bumper_update(int,int,int,int)),this,SLOT(bumper_update_slot(int,int,int,int)));

    //IR data
    connect(&r,SIGNAL(irData(double,double)),this,SLOT(irdata_update_slot(double,double)));
    connect(&r,SIGNAL(irData(double,double)),&hokuyo,SLOT(IR_update(double,double)));

    //Spatial data
    connect(&r,SIGNAL(imu_data(double,double,double,double,double,double)),this,SLOT(imu_update_slot(double,double,double,double,double,double)));
    connect(&r,SIGNAL(magnetic_data(double,double,double)),this,SLOT(magnetic_update_slot(double,double,double)));

    //Battery
    connect(&r,SIGNAL(battery_percent(int)),ui->progressBar,SLOT(setValue(int))); //display the battery percent
    connect(&r,SIGNAL(battery_volts(double)),ui->lcdNumber_8,SLOT(display(double)));//display the battery voltage

    //****************************************************************************
    // GPS
    connect(ui->spinBox_2,SIGNAL(valueChanged(int)),(QObject*)&gps,SLOT(set_zoom(int))); //set zoom value for the gps map
    connect(&gps,SIGNAL(url_changed(QUrl)),this,SLOT(change_url(QUrl))); //set the rl to the web viewer
    connect(ui->comboBox,SIGNAL(currentIndexChanged(QString)),&gps,SLOT(set_map_type(QString)));//set the gps map type
    connect(ui->save_gps,SIGNAL(clicked()),&gps,SLOT(save_clicked())); //save button clicked
    connect(ui->start_gps,SIGNAL(clicked()),&gps,SLOT(start_clicked()));//start button clicked
    connect(ui->sto_gps,SIGNAL(clicked()),&gps,SLOT(stop_clicked()));//stop button clicked
    connect(ui->load_gps,SIGNAL(clicked()),&gps,SLOT(load_clicked())); //load button clicked
    connect(&r,SIGNAL(gps_lat(double)),ui->lcdNumber_4,SLOT(display(double))); //display the latitude
    connect(&r,SIGNAL(gps_lon(double)),ui->lcdNumber_5,SLOT(display(double)));//display the longitude
    connect(&r,SIGNAL(gps_coord(double,double)),&gps,SLOT(update_coord(double,double))); //gives new coordinate to the gps widget
    connect(this,SIGNAL(size_changed()),&gps,SLOT(check_size()));//adjust the size of the map with the size of the window
    connect(ui->listWidget,SIGNAL(itemSelectionChanged()),&gps,SLOT(selection_changed()));//selection of the itinerary changed


    //****************************************************************************
    //Pan/Tilt control

    connect(ui->Pan_control_bar,SIGNAL(valueChanged(int)),&r,SLOT(Pan_control(int)));
    connect(ui->Tilt_control_bar,SIGNAL(valueChanged(int)),&r,SLOT(Tilt_control(int)));
    connect(ui->PanTilt_Reset,SIGNAL(clicked()),this,SLOT(Pan_Tilt_reset()));

    //****************************************************************************
    //Cameras
    connect(&r,SIGNAL(update_ptzcam(QImage)),this,SLOT(update_ptz(QImage))); //force to update the ptz cam
    connect(&r,SIGNAL(update_mapimage(QImage)),this,SLOT(update_map(QImage))); //force to update the ptz cam
    connect(&r,SIGNAL(update_rearcam(QImage)),this,SLOT(update_rear(QImage))); //force to update the reat cam scene
    connect(&r,SIGNAL(update_kinectRGBcam(QImage)),this,SLOT(update_kinectRGB(QImage))); //force to update the kinect RGB scene
    connect(&r,SIGNAL(update_kinectDepthcam(QImage)),this,SLOT(update_kinectDepth(QImage))); //force to update the kinect Depth scene
    connect(ui->camera, SIGNAL(currentChanged(int)),&r, SLOT(currentCameraTabChanged(int))); //Subscribe only to the camera topic we are interested in

    //***************************************************************************
    //Main motor control

    connect(ui->Forward, SIGNAL(pressed()), &r, SLOT(increase_speed()));
    connect(ui->Forward, SIGNAL(released()), &r, SLOT(decrease_speed()));
    connect(ui->BACKWARD, SIGNAL(pressed()), &r, SLOT(increase_backward_speed()));
    connect(ui->BACKWARD, SIGNAL(released()), &r, SLOT(decrease_speed()));

    connect(ui->TURNLEFT,SIGNAL(pressed()),&r,SLOT(turn_left()));
    connect(ui->TURNRIGHT,SIGNAL(pressed()),&r,SLOT(turn_right()));
    connect(ui->TURNLEFT,SIGNAL(released()),&r,SLOT(stop_turn()));
    connect(ui->TURNRIGHT,SIGNAL(released()),&r,SLOT(stop_turn()));
    connect(ui->STOP,SIGNAL(clicked()),&r,SLOT(motor_stop()));

    connect(ui->speed_fast, SIGNAL(toggled(bool)), &r, SLOT(setSpeedFast(bool)));
    connect(ui->speed_moderate, SIGNAL(toggled(bool)), &r, SLOT(setSpeedModerate(bool)));
    connect(ui->speed_slow, SIGNAL(toggled(bool)), &r, SLOT(setSpeedSlow(bool)));

    connect(ui->Forward_2, SIGNAL(pressed()), &r, SLOT(increase_speed()));
    connect(ui->Forward_2, SIGNAL(released()), &r, SLOT(decrease_speed()));
    connect(ui->BACKWARD_2, SIGNAL(pressed()), &r, SLOT(increase_backward_speed()));
    connect(ui->BACKWARD_2, SIGNAL(released()), &r, SLOT(decrease_speed()));

    connect(ui->TURNLEFT_2,SIGNAL(pressed()),&r,SLOT(turn_left()));
    connect(ui->TURNRIGHT_2,SIGNAL(pressed()),&r,SLOT(turn_right()));
    connect(ui->TURNLEFT_2,SIGNAL(released()),&r,SLOT(stop_turn()));
    connect(ui->TURNRIGHT_2,SIGNAL(released()),&r,SLOT(stop_turn()));
    connect(ui->STOP_2,SIGNAL(clicked()),&r,SLOT(motor_stop()));

    connect(ui->speed_fast_2, SIGNAL(toggled(bool)), &r, SLOT(setSpeedFast(bool)));
    connect(ui->speed_moderate_2, SIGNAL(toggled(bool)), &r, SLOT(setSpeedModerate(bool)));
    connect(ui->speed_slow_2, SIGNAL(toggled(bool)), &r, SLOT(setSpeedSlow(bool)));


    //*******************************************************************************
    //Hokuyo update Tab
    connect(&r,SIGNAL(hokuyo_update(Hokuyo_Points*)),&hokuyo,SLOT(hokuyo_update(Hokuyo_Points*)));

    //buttons configuration
    connect(ui->ShowLRFlines,SIGNAL(clicked()),this,SLOT(showLRFlines()));
    connect(ui->ShowIRdata,SIGNAL(stateChanged(int)),this,SLOT(showIRdata(int)));
    connect(ui->showLRF,SIGNAL(stateChanged(int)),this,SLOT(showLRFdata(int)));

    //*******************************************************************************

    connect(&r,SIGNAL(velocity_info(double,double)),this,SLOT(encoder_info_update(double,double)));

    connect(ui->quitButton,SIGNAL(clicked()),this,SLOT(close()));

    //*******************************************************************************
    //Motor Control Toggle
    connect(ui->MotorControlToggle,SIGNAL(stateChanged(int)),this,SLOT(motor_control_toggle(int)));

    //***********************************************************************************


    //create different scenes
    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene->setSceneRect(0, 0, 640, 480);

    QGraphicsScene *scene3 = new QGraphicsScene(this);
    scene3->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene3->setSceneRect(0, 00, 640, 480);

    QGraphicsScene *scene4 = new QGraphicsScene(this);
    scene4->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene4->setSceneRect(0, 00, 640, 480);

    QGraphicsScene *scene10 = new QGraphicsScene(this);
    scene10->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene10->setSceneRect(0, 00, 640, 480);

    QGraphicsScene *scene2 = new QGraphicsScene(this);
    scene2->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene2->setSceneRect(0, 00, 2048, 2048);

    QGraphicsScene *scene5 = new QGraphicsScene(this);
    scene5->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene5->setSceneRect(0, 00, 640, 480);


    //set scenes to the graphicsView widget

	scene10->setSceneRect(0,0,640,480);
	image_rear_cam.setPos(0,0);
	scene10->addItem(&image_rear_cam);

	scene2->setSceneRect(0,0,2048,2048);
	image_map_image.setPos(0,0);
	scene2->addItem(&image_map_image);

	scene5->setSceneRect(0,0,640,480);
	image_front_cam.setPos(0,0);
	scene5->addItem(&image_front_cam);

	scene->setSceneRect(0,0,640,480);
	image_ptz_cam.setPos(0,0);
	scene->addItem(&image_ptz_cam);

	scene4->setSceneRect(0,0,640,480);
	image_kinect_rgb.setPos(0,0);
	scene4->addItem(&image_kinect_rgb);

	scene3->setSceneRect(0,0,640,480);
	image_kinect_depth.setPos(0,0);
	scene3->addItem(&image_kinect_depth);

	ui->graphicsView->setScene(scene);
    	ui->graphicsView_2->setScene(scene4);
    	ui->graphicsView_3->setScene(scene3);
    	ui->graphicsView_10->setScene(scene10);
    	ui->mapView->setScene(scene2);
    	ui->frontView->setScene(scene5);

    connect(&r,SIGNAL(save_image(bool)), &image_ptz_cam, SLOT(saveImage(bool))); //order to save the current image from the front camera.


next_gripper_state = true;


//change focus policy to always be able to move robot with the keyboard
ui->camera->setFocusPolicy(Qt::NoFocus);
this->setFocusPolicy(Qt::StrongFocus);

ui->tabWidget_5->setFocus();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setArguments(int argc_, char *argv_[])
{
	argc = argc_;
	argv = argv_;
}


void MainWindow::connect_clicked(){ // executed when the connect button is pushed
    ui->connect->setText("Disconnect");
    ui->connect->setEnabled(false);
    // Initialize the ROS node
    if ( ui->environmentcheckbox->isChecked() ) {
            r.init(argc, argv);
    } else {
            r.init(argc, argv, ui->Master->text().toStdString(),ui->Host->text().toStdString());
            ui->Master->setReadOnly(true);
            ui->Host->setReadOnly(true);
    }
}

void MainWindow::resizeEvent(QResizeEvent *){//execute this function when the window size changes
    emit size_changed();
}

void MainWindow::change_url(QUrl url)//set url to the web viewer
{
	if(gpsUrlTImer.isValid() == false || gpsUrlTImer.elapsed()>3000) //We don't want to do more than one querry every 3s
	{
		gpsUrlTImer.restart();
		gpsUrlTImer.start();
                //ui->webView->setUrl(url); Removed because we don't want to break the Google's terms and conditions
	}
}



void MainWindow::update_ptz(QImage image) //order the ptz camera scene to update
{
        ((Image*)(ui->graphicsView->scene()->items().at(0)))->setImage(image);
    ui->graphicsView->scene()->update(0,0,ui->graphicsView->scene()->width(),ui->graphicsView->scene()->height());

    ((Image*)(ui->frontView->scene()->items().at(0)))->setImage(image);
    ui->frontView->scene()->update(0,0,ui->frontView->scene()->width(),ui->frontView->scene()->height());
}

void MainWindow::update_map(QImage image) //order the map image scene to update
{
    ((Image*)(ui->mapView->scene()->items().at(0)))->setImage(image);
    ui->mapView->scene()->setSceneRect(0,0,image.height(),image.width());
    ui->mapView->scene()->update(0,0,ui->mapView->scene()->width(),ui->mapView->scene()->height());
}

void MainWindow::update_rear(QImage image) //order the rear camera scene to update
{
        ((Image*)(ui->graphicsView_10->scene()->items().at(0)))->setImage(image);
    ui->graphicsView_10->scene()->update(0,0,ui->graphicsView_10->scene()->width(),ui->graphicsView_10->scene()->height());
}

void MainWindow::update_kinectRGB(QImage image) //order the kinect RGB camera scene to update
{
        ((Image*)(ui->graphicsView_2->scene()->items().at(0)))->setImage(image);
    ui->graphicsView_2->scene()->update(0,0,ui->graphicsView_2->scene()->width(),ui->graphicsView_2->scene()->height());
}

void MainWindow::update_kinectDepth(QImage image) //order the kinect Depth camera scene to update
{
        ((Image*)(ui->graphicsView_3->scene()->items().at(0)))->setImage(image);
    ui->graphicsView_3->scene()->update(0,0,ui->graphicsView_3->scene()->width(),ui->graphicsView_3->scene()->height());
}



void MainWindow::keyReleaseEvent(QKeyEvent *event){//executed each time a keyboard key is release, to control the robot movements

    switch (event->key()) {
        case Qt::Key_W:
            if(!event->isAutoRepeat())
            {
            	r.decrease_speed();
            }
            break;
        case Qt::Key_S:
            if(!event->isAutoRepeat())
            {
            	r.decrease_speed();
            }
            break;
        case Qt::Key_D:
            if(!event->isAutoRepeat())
            {
            	r.stop_turn();
	    }
            break;
        case Qt::Key_A:
            if(!event->isAutoRepeat())
            {
            	r.stop_turn();
	    }
            break;
        default:
            QWidget::keyReleaseEvent(event);
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event){//executed each time a keyboard key is pushed, to control the robot movements
    switch (event->key()) {
    case Qt::Key_W:
        if(!event->isAutoRepeat())
        {
		r.increase_speed();
        }
        break;
    case Qt::Key_S:
        if(!event->isAutoRepeat())
        {
		r.increase_backward_speed();
        }
        break;
    case Qt::Key_D:
        if(!event->isAutoRepeat())
        {
        	r.turn_right();
        }
        break;
    case Qt::Key_A:
        if(!event->isAutoRepeat())
        {
        	r.turn_left();
        }
        break;
    case Qt::Key_Left:
	if(r.pan >= -65) 
        	ui->Pan_control_bar->setValue(r.pan - 5);
	else if (r.pan > -70)
		ui->Pan_control_bar->setValue(-70);
        break;
    case Qt::Key_Right:
	if(r.pan <= 65) 
        	ui->Pan_control_bar->setValue(r.pan + 5);
	else if (r.pan < 70)
		ui->Pan_control_bar->setValue(70);
        break;
    case Qt::Key_Up:
	if(r.tilt <= 25) 
        	ui->Tilt_control_bar->setValue(r.tilt + 5);
	else if (r.tilt < 30)
		ui->Tilt_control_bar->setValue(30);
        break;
    case Qt::Key_Down:
	if(r.tilt >= -25) 
        	ui->Tilt_control_bar->setValue(r.tilt - 5);
	else if (r.tilt > -30)
		ui->Tilt_control_bar->setValue(-30);
        break;
    case Qt::Key_J:
        arm.moveArmLeft();
        break;
    case Qt::Key_L:
        arm.moveArmRight();
        break;
    case Qt::Key_I:
        arm.moveArmUp();
        break;
    case Qt::Key_K:
        arm.moveArmDown();
        break;
    case Qt::Key_Space:
        r.motor_stop();
        if(next_gripper_state)
            r.closeGripper();
        else
            r.openGripper();
        next_gripper_state = !next_gripper_state;
        break;
    default:
        QWidget::keyPressEvent(event);
    }
}


void MainWindow::bumper_update_slot(int bumper1, int bumper2, int bumper3, int bumper4) // update bumper data
{
    if(bumper1 != 0) ui->label_19->setText("Bumper 1 HIT!"); else ui->label_19->setText("Bumper 1 SAFE");
    if(bumper2 != 0) ui->label_24->setText("Bumper 2 HIT!"); else ui->label_24->setText("Bumper 2 SAFE");
    if(bumper3 != 0) ui->label_26->setText("Bumper 3 HIT!"); else ui->label_26->setText("Bumper 3 SAFE");
    if(bumper4 != 0) ui->label_40->setText("Bumper 4 HIT!"); else ui->label_40->setText("Bumper 4 SAFE");

}



void MainWindow::encoder_info_update(double linearVelocity,double angularVelocity) // update the encoder info on the interface
{
    ui->linear_velocity->display(linearVelocity);
    ui->angular_velocity->display(angularVelocity);
}


void MainWindow::motor_control_toggle(int value) // interface the toggle button "Motor Control Disable"
{
    if(value != 0)
    {
        ui->Forward->setEnabled(false);
        ui->BACKWARD->setEnabled(false);
        ui->TURNLEFT->setEnabled(false);
        ui->TURNRIGHT->setEnabled(false);
        ui->STOP->setEnabled(false);
        ui->frame->setEnabled(false);
        ui->frame_7->setEnabled(false);
        ui->frame_8->setEnabled(false);
    }

    else
    {
        ui->Forward->setEnabled(true);
        ui->BACKWARD->setEnabled(true);
        ui->TURNLEFT->setEnabled(true);
        ui->TURNRIGHT->setEnabled(true);
        ui->STOP->setEnabled(true);
        ui->frame->setEnabled(true);
        ui->frame_7->setEnabled(true);
        ui->frame_8->setEnabled(true);
    }

}

void MainWindow::irdata_update_slot(double ir01, double ir02) //update the onscreen value of the infrared sensor
{
    ui->lcdNumber_IR01->display(ir01);
    ui->lcdNumber_IR02->display(ir02);
}

void MainWindow::imu_update_slot(double acc_x, double acc_y, double acc_z, double ang_x, double ang_y, double ang_z) //update the imu values on the interface
{
    ui->lcdNumber_acc_x->display(acc_x);
    ui->lcdNumber_acc_y->display(acc_y);
    ui->lcdNumber_acc_z->display(acc_z);
    ui->lcdNumber_ang_x->display(ang_x);
    ui->lcdNumber_ang_y->display(ang_y);
    ui->lcdNumber_any_z->display(ang_z);
}

void MainWindow::magnetic_update_slot(double mag_x, double mag_y, double mag_z) //update the imu values on the interface
{
    ui->lcdNumber_mag_x->display(mag_x);
    ui->lcdNumber_mag_y->display(mag_y);
    ui->lcdNumber_mag_z->display(mag_z);
}

void MainWindow::showLRFlines() // Interface the show line button in the rangefinding interface
{
    QString string1 = "No Lines";
    QString string2 = "Show Lines";

    if(LRF_lines_show == false)
    {
        ui->ShowLRFlines->setText(string1);
        LRF_lines_show = true;
        hokuyo.showlines = true;
    }
    else
    {
        LRF_lines_show = false;
        ui->ShowLRFlines->setText(string2);
        hokuyo.showlines = false;
    }

}

void MainWindow::showIRdata(int value) // Interface the show ir data button in the rangefinder interface
{

    if(value != 0)
    {
        hokuyo.showIR = true;
        hokuyo.showLRF = false;
        ui->showLRF->setEnabled(false);
    }

    else
    {
        hokuyo.showIR = false;
        //hokuyo.showLRF = false;
        ui->showLRF->setEnabled(true);
    }

}

void MainWindow::showLRFdata(int value) // Interface the show LRF button in the rangefinder interface
{
    if(value != 0)
    {
        hokuyo.showIR = false;
        hokuyo.showLRF = true;
        ui->ShowIRdata->setEnabled(false);
    }

    else
    {
        //hokuyo.showIR = false;
        hokuyo.showLRF = false;
        ui->ShowIRdata->setEnabled(true);
    }


}

void MainWindow::Pan_Tilt_reset()// ask the ros node to reset the pan and tilt position of the camera
{
	r.Pan_control(0);
	r.Tilt_control(0);
    	ui-> Pan_control_bar->setValue(0);
    	ui-> Tilt_control_bar ->setValue(0);
}
