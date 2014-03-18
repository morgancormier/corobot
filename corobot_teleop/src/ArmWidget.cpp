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

#include "ArmWidget.h"

#include <math.h>
#include <QtGui>
#include "corobot.h"

double inchesToMeters(const double inches)
//convert from inches to meters
{
    return inches * 2.54 / 100.0;
}

double PixeltoMeter(const double pixel)
//convert from pixel to meter
{
    return (pixel / PIXELS_PER_METER);
}

double MetersToPixels(const double meters)
//convert from meters to pixels
{
    return (meters * PIXELS_PER_METER);
}

//Shoulder and Elbow width constant for different arms. Some constants are not known so they are given a false value
//Old corobot Arm
const double COROBOT_ARM_SHOULDER_SEGMENT_METERS = inchesToMeters(6.25);
const double COROBOT_ARM_ELBOW_SEGMENT_METERS = inchesToMeters(8);

//PhantomX Pincher
const double PINCHER_ARM_SHOULDER_SEGMENT_METERS = inchesToMeters(6.25);
const double PINCHER_ARM_ELBOW_SEGMENT_METERS = inchesToMeters(8);

//PhantomX Reactor
const double REACTOR_ARM_SHOULDER_SEGMENT_METERS = inchesToMeters(6.25);
const double REACTOR_ARM_ELBOW_SEGMENT_METERS = inchesToMeters(8);

//Lynxmotion al5a
const double AL5A_ARM_SHOULDER_SEGMENT_METERS = inchesToMeters(3.75);
const double AL5A_ARM_ELBOW_SEGMENT_METERS = inchesToMeters(7.62);


double armShoulderSegmentMeters=COROBOT_ARM_SHOULDER_SEGMENT_METERS;
double armElbowSegmentMeters=COROBOT_ARM_ELBOW_SEGMENT_METERS;

//Constant related to the position of the arm in the widget
const int ARM_X = 120;
const int ARM_Y = 120;
const int ARM_CENTER_X = ARM_X / 2;
const int ARM_BODY_TOP = ARM_X * 7 / 10;
const int ARM_SHOULDER_X = ARM_CENTER_X - ARM_X / 7;


ArmWidget::ArmWidget(QWidget *parent)
    : QGraphicsView(parent), timerId(0)
{

    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene->setSceneRect(0, 00, 210, 110);

	shoulder = true;
	elbow = true;
    setScene(scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    scale(qreal(1), qreal(1));
    setMinimumSize(210, 110);
    setWindowTitle(tr("Elastic Nodes"));

    joint *joint1 = new joint(this); // shoulder joint, fixed
    joint *joint2 = new joint(this); // elbow joint, not fixed but not moveable directly
    joint *joint3 = new joint(this); // gripper, moveable by the user


    scene->addItem(joint1);
    scene->addItem(joint2);
    scene->addItem(joint3);

    joint3->setFlag(joint3->ItemIsMovable);

    double x = armShoulderSegmentMeters;
    double y = armElbowSegmentMeters;

    joint1->setPos(this->sceneRect().left()+ARM_SHOULDER_X+5, this->sceneRect().bottom()-ARM_Y+ARM_BODY_TOP+5);
    joint2->setPos(this->sceneRect().left()+ARM_SHOULDER_X+5, -MetersToPixels(y)-ARM_Y+ARM_BODY_TOP+5+this->sceneRect().bottom());


    joint3->setPos(MetersToPixels(x)+this->sceneRect().left()+ARM_SHOULDER_X+5,
                   -MetersToPixels(y)-ARM_Y+ARM_BODY_TOP+5+this->sceneRect().bottom());
    end_effector = QPointF(0,0);

    QGraphicsLineItem *line = new QGraphicsLineItem(joint1->pos().x()-5,joint1->pos().y()-5,joint2->pos().x()-5,joint2->pos().y()-5); //line between shoulder and elbow
    scene->addItem(line);
    QGraphicsLineItem *line2 = new QGraphicsLineItem(joint2->pos().x()-5,joint2->pos().y()-5,joint3->pos().x()-5,joint3->pos().y()-5); // line between the elbow and the gripper
    scene->addItem(line2);

 }
/**
 * @brief computes the inverse kinematics for the arm
 * @param x the desired distance of the end effector in front of the robot (meters)
 * @param y the desired distance of the end effector above the ground plane (meters)
 * @param theta1 [out] the joint angle of the shoulder to reach the desired pose
 * @param theta2 [out] the joint angle of the elbow to reach the desired pose
 **/
bool doArmIK(const double x, const double y, double& theta1, double& theta2)
//compute the inverse kinematics for the arm
{
    const double LOWER_LINK_LEN = armShoulderSegmentMeters;
    const double UPPER_LINK_LEN = armElbowSegmentMeters;
    const double LOWER_SQUARED = armShoulderSegmentMeters * armShoulderSegmentMeters;
    const double UPPER_SQUARED = armElbowSegmentMeters * armElbowSegmentMeters;

    double c2;                  // cosine of theta2

        c2 = ((x * x) + (y * y) - LOWER_SQUARED - UPPER_SQUARED) /
            (2 * LOWER_LINK_LEN * UPPER_LINK_LEN);
        if ((c2 < -1.0) || (c2 > 1.0))
        {
            return false;
        }

        double magS2;               // magnitude of sine of theta2 (could be + or - this).

        magS2 = sqrt(1 - c2 * c2);
        double posS2 = magS2;
        double negS2 = -magS2;
        double posTheta2,
            negTheta2;

        posTheta2 = atan2(posS2, c2);
        negTheta2 = atan2(negS2, c2);

        double posS1;               // sine of theta1

        posS1 =
            ((LOWER_LINK_LEN + UPPER_LINK_LEN * c2) * y -
             (UPPER_LINK_LEN * posS2 * x)) / (x * x + y * y);
        double posC1;

        posC1 =
            ((LOWER_LINK_LEN + UPPER_LINK_LEN * c2) * x +
             (UPPER_LINK_LEN * posS2 * y)) / (x * x + y * y);
        double posTheta1;

        posTheta1 = atan2(posS1, posC1);

        double negS1;               // sine of theta1

        negS1 =
            ((LOWER_LINK_LEN + UPPER_LINK_LEN * c2) * y -
             (UPPER_LINK_LEN * negS2 * x)) / (x * x + y * y);
        double negC1;

        negC1 =
            ((LOWER_LINK_LEN + UPPER_LINK_LEN * c2) * x +
             (UPPER_LINK_LEN * negS2 * y)) / (x * x + y * y);
        double negTheta1;

        negTheta1 = atan2(negS1, negC1);

        // bounds checking - make sure the joints are within ranges.
        if ((posTheta1 <= M_PI * 0.75) && (posTheta1 >= 0.0) && (posTheta2 <= 0.0))
        {
            theta1 = posTheta1;
            theta2 = posTheta2;
            return true;
        }

        // bounds checking - make sure the joints are within ranges.
        if ((negTheta1 <= M_PI * 0.75) && (negTheta1 >= 0.0) && (negTheta2 <= 0.0))
        {
            theta1 = negTheta1;
            theta2 = negTheta2;
            return true;
        }
        return false;
}

void ArmWidget::Corobot(bool value)
// value is true of a Corobot, false if is an Explorer, Explorer doesn't have an arm therefore the Widget is white
{
        foreach (QGraphicsItem *item, scene()->items()) {
            if(value == true){
                item->show();
            }
            else{
                item->hide();
            }
        }
}

void ArmWidget::setModel(bool arm_al5a,bool arm_pincher,bool arm_reactor,bool arm_old_corobot)
// set the number of Degree of Freedom of the arm and therefore the dimensions.
{
    if (arm_old_corobot)
    {
        armShoulderSegmentMeters= COROBOT_ARM_SHOULDER_SEGMENT_METERS;
        armElbowSegmentMeters= COROBOT_ARM_ELBOW_SEGMENT_METERS;
	arm_type = OldCorobot;
    }
    else if (arm_pincher)
    {
        armShoulderSegmentMeters= PINCHER_ARM_SHOULDER_SEGMENT_METERS;
        armElbowSegmentMeters= PINCHER_ARM_ELBOW_SEGMENT_METERS;
	arm_type = Pincher;
    }
    else if (arm_reactor)
    {
        armShoulderSegmentMeters= REACTOR_ARM_SHOULDER_SEGMENT_METERS;
        armElbowSegmentMeters= REACTOR_ARM_ELBOW_SEGMENT_METERS;
	arm_type = Reactor;
    }
    else if (arm_al5a)
    {
        armShoulderSegmentMeters= AL5A_ARM_SHOULDER_SEGMENT_METERS;
        armElbowSegmentMeters= AL5A_ARM_ELBOW_SEGMENT_METERS;
	arm_type = Al5a;
    }

    printf("ARM_LENGTH %f, %f", armShoulderSegmentMeters, armShoulderSegmentMeters);

    QGraphicsScene *scene = this->scene();

    joint* joint1 = (joint*)scene->items().at(0);
    joint* joint2 = (joint*)scene->items().at(1);
    joint* joint3 = (joint*)scene->items().at(2);

    double x = armShoulderSegmentMeters;
    double y = armElbowSegmentMeters;

    joint3->setPos(MetersToPixels(x)+this->sceneRect().left()+ARM_SHOULDER_X+5,-MetersToPixels(y)-ARM_Y+ARM_BODY_TOP+5+this->sceneRect().bottom());
    joint2->setPos(this->sceneRect().left()+ARM_SHOULDER_X+5, -MetersToPixels(y)-ARM_Y+ARM_BODY_TOP+5+this->sceneRect().bottom());

    ((QGraphicsLineItem*)(scene->items().at(3)))->setLine(joint1->pos().x()-5,joint1->pos().y()-5,joint2->pos().x()-5,joint2->pos().y()-5);
    ((QGraphicsLineItem*)(scene->items().at(4)))->setLine(joint2->pos().x()-5,joint2->pos().y()-5,joint3->pos().x()-5,joint3->pos().y()-5);


}


void ArmWidget::itemMoved()
//timer starts if a movement is detected
{
    if (!timerId)
        timerId = startTimer(1000 / 25);
}

void ArmWidget::shoulder_degree(bool value)
//if value is true, the signal theta1 will be emited in degrees
{
    shoulder = value;

}

void ArmWidget::elbow_degree(bool value)
//if value is true, the signal theta2 will be emited in degrees
{
    elbow = value;

}

void ArmWidget::moveArmUp()
//moves the arm up
{
    this->scene()->items().at(2)->setPos(end_effector + QPointF(0,-5));
}

void ArmWidget::moveArmDown()
//moves the arm down
{
    this->scene()->items().at(2)->setPos(end_effector + QPointF(0,5));
}

void ArmWidget::moveArmLeft()
//moves the arm toward the robot
{
    this->scene()->items().at(2)->setPos(end_effector + QPointF(-5,0));
}

void ArmWidget::moveArmRight()
//moves the arm far from the robot
{
    this->scene()->items().at(2)->setPos(end_effector + QPointF(5,0));
}

void ArmWidget::received_pos(double x, double y)
//set the position ofthe arm. x,y if he position in meters
{

    QList<joint *> joints;
        foreach (QGraphicsItem *item, scene()->items()) {
            if (joint *j = qgraphicsitem_cast<joint *>(item))
                joints << j;
        }

    joints.at(2)->setPos(MetersToPixels(x)+this->sceneRect().left()+ARM_SHOULDER_X+5,
                -MetersToPixels(y)-ARM_Y+ARM_BODY_TOP+5+this->sceneRect().bottom());


}


void ArmWidget::timerEvent(QTimerEvent *event)
//execute this function when the timer is up. the timer prevents impossible movements to the arm.
 {
     Q_UNUSED(event);

	// get all the joints
     QList<joint *> joints;
         foreach (QGraphicsItem *item, scene()->items()) {
             if (joint *j = qgraphicsitem_cast<joint *>(item))
                 joints << j;
         }


         const double LINK_1_LENGTH = MetersToPixels(armShoulderSegmentMeters);
         const double LINK_2_LENGTH = MetersToPixels(armElbowSegmentMeters);

         float x,y;

             x = PixeltoMeter(joints.at(2)->pos().x()-5-(this->sceneRect().left()+ARM_SHOULDER_X));
             y = PixeltoMeter(this->sceneRect().bottom()-ARM_Y+ARM_BODY_TOP-joints.at(2)->pos().y()+5);

         if(joints.at(2)->x()!=end_effector.x() || joints.at(2)->y()!=end_effector.y()){ // if the gripper moved
             double t1,t2;
             if(doArmIK(x,y, t1, t2)){ // do the inverse kinematic with the new gripper position

            double x = LINK_1_LENGTH * cos(t1);
            double y = LINK_1_LENGTH * sin(t1);

            int elbowX = ARM_SHOULDER_X + (int)x;
            int elbowY = -ARM_Y+ARM_BODY_TOP - (int)y;

            joints.at(1)->setPos(this->sceneRect().left()+elbowX+5, this->sceneRect().bottom()+elbowY+5); // set the position to the elbow


            if(shoulder)
                emit theta1(t1/M_PI*180);
            else
                emit theta1(t1);
            if(elbow)
                emit theta2(t2/M_PI*180+180);
            else
                emit theta2(t2 + M_PI);

	    if (arm_type == Al5a)
	    {
		emit shoulderAngle_rad(180 - t1);
	    	emit elbowAngle_rad(180 + t2);
	    }
	    else
	    {
	        emit shoulderAngle_rad(t1);
	        emit elbowAngle_rad(-t2);
	    }
	
		// Draw the arm lines 
                 QList<QGraphicsLineItem *> lines;
                     foreach (QGraphicsItem *item, scene()->items()) {
                         if (QGraphicsLineItem *l = qgraphicsitem_cast<QGraphicsLineItem *>(item))
                             lines << l;
                     }

                         lines.at(0)->setLine(joints.at(0)->pos().x()-5,joints.at(0)->pos().y()-5,joints.at(1)->pos().x()-5,joints.at(1)->pos().y()-5);
                         lines.at(1)->setLine(joints.at(1)->pos().x()-5,joints.at(1)->pos().y()-5,joints.at(2)->pos().x()-5,joints.at(2)->pos().y()-5);

                 end_effector = joints.at(2)->pos();
                 }
             else{
                joints.at(2)->setPos(end_effector);
             }
         }



 }




 void ArmWidget::drawBackground(QPainter *painter, const QRectF &rect)
 // draw the widget background
 {
     Q_UNUSED(rect);

     // Shadow
     QRectF sceneRect = this->sceneRect();
     QRectF rightShadow(sceneRect.right(), sceneRect.top() + 5, 5, sceneRect.height());
     QRectF bottomShadow(sceneRect.left() + 5, sceneRect.bottom(), sceneRect.width(), 5);
     if (rightShadow.intersects(rect) || rightShadow.contains(rect))
         painter->fillRect(rightShadow, Qt::darkGray);
     if (bottomShadow.intersects(rect) || bottomShadow.contains(rect))
         painter->fillRect(bottomShadow, Qt::darkGray);


     // Fill
     QLinearGradient gradient(sceneRect.topLeft(), sceneRect.bottomRight());
     gradient.setColorAt(0, Qt::white);
     gradient.setColorAt(1, Qt::lightGray);
     painter->fillRect(rect.intersect(sceneRect), gradient);
     painter->setBrush(Qt::NoBrush);
     painter->drawRect(sceneRect);

     //Robot
      QRectF robot(sceneRect.left(), sceneRect.bottom()-ARM_Y+ARM_BODY_TOP, ARM_X / 2, ARM_Y - ARM_BODY_TOP);
      painter->fillRect(robot, Qt::gray);
      painter->setBrush(Qt::NoBrush);
      painter->drawRect(robot);


     // Text
     QRectF textRect(sceneRect.left() + 4, sceneRect.top() + 4,
                     sceneRect.width() - 4, sceneRect.height() - 4);

     QFont font = painter->font();
     font.setBold(true);
     font.setPointSize(14);
     painter->setFont(font);
     painter->setPen(Qt::lightGray);
     painter->setPen(Qt::black);
 }
