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

#include "GripperWidget.h"

#include <QtGui>

/** Constant used for the position of items on the widget window
 * As of now the widget displays two red lines representing the gripper and its state, plus a background. The lines are not moveable. 
 * Maybe the widget can be improved in the future, since as of now it is not very useful
 */

const int GRIPPER_X = 48;
const int GRIPPER_Y = 48;
const int GRIPPER_CENTER_X = GRIPPER_X / 2;
const int GRIPPER_CENTER_Y = GRIPPER_Y / 2;
const int GRIPPER_BOTTOM_LINE = GRIPPER_Y * 8 / 10;
const int GRIPPER_RIGHT_LINE = GRIPPER_X * 8 / 10;
const int GRIPPER_LEFT_LINE = GRIPPER_X - GRIPPER_RIGHT_LINE;
const int GRIPPER_LINE_WIDTH = 3;
const int GRIPPER_LINE_TOP = GRIPPER_Y / 6;
const int GRIPPER_LINE_BOTTOM = GRIPPER_Y * 5 / 6;
const int GRIPPER_CIRCLE_RADIUS = GRIPPER_X / 3;


GripperWidget::GripperWidget(QWidget *parent)
    : QGraphicsView(parent), timerId(0)
{

    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene->setSceneRect(0, 0, 50, 50);


    setScene(scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    scale(qreal(1), qreal(1));
    setMinimumSize(50, 50);
    setWindowTitle(tr("Elastic Nodes"));

    percent = 1.0;


    int x = GRIPPER_CENTER_X - (GRIPPER_CENTER_X - GRIPPER_LEFT_LINE) * percent;
    QGraphicsLineItem *line = new QGraphicsLineItem(
            x, GRIPPER_LINE_TOP, x, GRIPPER_LINE_BOTTOM);
    x = GRIPPER_CENTER_X + (GRIPPER_RIGHT_LINE - GRIPPER_CENTER_X) * percent;
    QGraphicsLineItem *line2 = new QGraphicsLineItem(
            x, GRIPPER_LINE_TOP, x, GRIPPER_LINE_BOTTOM);

    line->setPen(QPen(Qt::red,GRIPPER_LINE_WIDTH));
    line2->setPen(QPen(Qt::red,GRIPPER_LINE_WIDTH));

    scene->addItem(line);
    scene->addItem(line2);

    timerId = startTimer(1000 / 25);


 }

void GripperWidget::setState(int value)
//set the state of the gripper, 1 = open, 2 = closed, 3 = moving
{
    if(value ==1)
        percent = 1.0;
    else if(value ==2)
        percent = 0.0;
    else if(value==3)
        percent = 0.5;

    /*
     receive ROS state value*/
}


void GripperWidget::timerEvent(QTimerEvent *event)
 {
     Q_UNUSED(event);


     QList<QGraphicsLineItem  *> lines;
         foreach (QGraphicsItem  *item, scene()->items()) {
             if (QGraphicsLineItem  *l = qgraphicsitem_cast<QGraphicsLineItem  *>(item))
                 lines << l;
         }



     int x = GRIPPER_CENTER_X - (GRIPPER_CENTER_X - GRIPPER_LEFT_LINE) * percent;

     lines.at(0)->setLine(x, GRIPPER_LINE_TOP, x, GRIPPER_LINE_BOTTOM);

     x = GRIPPER_CENTER_X + (GRIPPER_RIGHT_LINE - GRIPPER_CENTER_X) * percent;

     lines.at(1)->setLine(x, GRIPPER_LINE_TOP, x, GRIPPER_LINE_BOTTOM);
 }


 void GripperWidget::drawBackground(QPainter *painter, const QRectF &rect)
 //draw the scene background
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


     painter->setPen(Qt::blue);
     painter->drawLine(0, GRIPPER_BOTTOM_LINE, GRIPPER_X, GRIPPER_BOTTOM_LINE);
     painter->drawLine(GRIPPER_LEFT_LINE, 0, GRIPPER_LEFT_LINE, GRIPPER_Y);
     painter->drawLine(GRIPPER_RIGHT_LINE, 0, GRIPPER_RIGHT_LINE, GRIPPER_Y);

     painter->setPen(QPen(Qt::green,2));
     painter->drawArc(GRIPPER_CENTER_X - GRIPPER_CIRCLE_RADIUS,
                              GRIPPER_CENTER_Y - GRIPPER_CIRCLE_RADIUS,
                              GRIPPER_CIRCLE_RADIUS * 2,GRIPPER_CIRCLE_RADIUS  * 2,
                              0, 360 * 64);
 }



