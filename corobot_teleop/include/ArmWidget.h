/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef ARMWIDGET_H
#define ARMWIDGET_H

#include <QtGui/QGraphicsView>
#include "joint.h"

static const double PIXELS_PER_METER = 200.0;


double inchesToMeters(const double inches);
double PixeltoMeter(const double pixel);
double MetersToPixels(const double meters);


typedef enum
{
    Al5a, Pincher, Reactor, OldCorobot
} armType;

class ArmWidget : public QGraphicsView
 {
     Q_OBJECT

 public:
     ArmWidget(QWidget *parent = 0);

     void itemMoved();//timer starts if a movement is detected
     void moveArmUp();//moves the arm up
     void moveArmDown();//moves the arm down
     void moveArmLeft();//moves the arm toward the robot
     void moveArmRight();//moves the arm far from the robot



 public slots:
     void shoulder_degree(bool value);//if value is true, the signal theta1 will be emited in degrees
     void elbow_degree(bool value);//if value is true, the signal theta2 will be emited in degrees
     void received_pos(double x, double y);//set the position ofthe arm. x,y if he position in meters
     void Corobot(bool value);// value is true of a Corobot, false if is an Explorer, Explorer doesn't have an arm therefore the Widget is white
     void setModel(bool arm_al5a,bool arm_pincher,bool arm_reactor,bool arm_old_corobot); //set the model of the arm, and so the constants for the length of the parts of the arm
 signals:
     void theta1(double value); //shoulder angle in either degree or radian
     void theta2(double value); //elbow angle in either degree or radian
     void shoulderAngle_rad(double value);//shoulder angle in radian
     void elbowAngle_rad(double value); //elbow angle in radian
     void posarm(float x, float y); //position of the arm in meters

 protected:
     void timerEvent(QTimerEvent *event);//execute this function when the timer is up.
     void drawBackground(QPainter *painter, const QRectF &rect); // draw the widget background

 private:
     int timerId;
     QPointF end_effector;
     bool shoulder;
     bool elbow;
     armType arm_type;

 };



#endif // ARMWIDGET_H

