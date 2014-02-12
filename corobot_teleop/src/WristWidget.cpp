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

#include "WristWidget.h"
#include <math.h>
#include <QtGui>

/**
 * Display the wrist orientation by a pair of red lines. The lines can be moved to change the orientation of the wrist
 */

WristWidget::WristWidget(QWidget *parent)
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


    wristAngle = 0;


    int x = WRIST_WIDTH / 2 * cos(wristAngle);
    int y = WRIST_WIDTH / 2 * sin(wristAngle);
    Line *line = new Line(
                    WRIST_CENTER_X - x, WRIST_CENTER_Y + y,
                    WRIST_CENTER_X + x, WRIST_CENTER_Y - y);

    line->setTransformOriginPoint(WRIST_CENTER_X,WRIST_CENTER_Y);
    x = WRIST_HEIGHT * cos(wristAngle + M_PI / 2);
    y = WRIST_HEIGHT * sin(wristAngle + M_PI / 2);
    QGraphicsLineItem *line2 = new QGraphicsLineItem(
                    WRIST_CENTER_X, WRIST_CENTER_Y,
                    WRIST_CENTER_X + x, WRIST_CENTER_Y - y);

    line->setPen(QPen(Qt::red,WRIST_LINE_WIDTH));
    line2->setPen(QPen(Qt::red,WRIST_LINE_WIDTH));

    // make the line be able to rotate around the origin point given as parameters
    line->setFlag(line->ItemIsMovable);
    line->setTransformOriginPoint(WRIST_CENTER_X, WRIST_CENTER_Y);
    scene->addItem(line);
    scene->addItem(line2);

    timerId = startTimer(1000 / 25);

 }

void WristWidget::degree(bool value)
//if value is true, emit the signal angle in degree,else in radian
{
    angle_type = value;
}


void WristWidget::angleReceived(double value)
//set a new value for the wrist angle
{
    QList<Line  *> lines;
    foreach (QGraphicsItem  *item, scene()->items()) {
        if (Line  *l = qgraphicsitem_cast<Line  *>(item))
            lines << l;
    }
  lines.at(0)->setRotation(-value);

}

void WristWidget::turnClockwise()
//turn the wrist clockwise
{
    QList<Line  *> lines;
    foreach (QGraphicsItem  *item, scene()->items()) {
        if (Line  *l = qgraphicsitem_cast<Line  *>(item))
            lines << l;
    }
  lines.at(0)->setRotation(lines.at(0)->rotation-5);
}

void WristWidget::turnCounterClockwise()
//turn the wrist counter clockwise
{
    QList<Line  *> lines;
    foreach (QGraphicsItem  *item, scene()->items()) {
        if (Line  *l = qgraphicsitem_cast<Line  *>(item))
            lines << l;
    }
  lines.at(0)->setRotation((lines.at(0)->rotation)+5);
}

void WristWidget::timerEvent(QTimerEvent *event)
// get the new wrist angle if the user interacted with the widget
 {
     Q_UNUSED(event);

     // get the lines from the scene and get the new angle if it has moved
     QList<Line  *> lines;
     foreach (QGraphicsItem  *item, scene()->items()) {
         if (Line  *l = qgraphicsitem_cast<Line  *>(item))
             lines << l;
     }
     if(wristAngle != -lines.at(0)->rotation){
            wristAngle = -lines.at(0)->rotation;

        if(angle_type)
            emit angle(wristAngle);
        else
            emit angle(wristAngle/180*M_PI);

        emit angle_rad(wristAngle/180*M_PI);

        QList<QGraphicsLineItem  *> lines2;
            foreach (QGraphicsItem  *item, scene()->items()) {
                if (QGraphicsLineItem  *l = qgraphicsitem_cast<QGraphicsLineItem  *>(item))
                    lines2 << l;
            }

          int x = WRIST_HEIGHT * cos(wristAngle*M_PI/180 + M_PI / 2);
          int y = WRIST_HEIGHT * sin(wristAngle*M_PI/180 + M_PI / 2);

          lines2.at(1)->setLine(WRIST_CENTER_X, WRIST_CENTER_Y,WRIST_CENTER_X + x, WRIST_CENTER_Y - y);
      }
 }


 void WristWidget::drawBackground(QPainter *painter, const QRectF &rect)
// Draw the background white/ grey color plus the background lines
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


     painter->setPen(Qt::gray);
     painter->drawLine(0, WRIST_CENTER_Y,WRIST_X, WRIST_CENTER_Y);
     painter->drawLine( WRIST_CENTER_X, WRIST_CENTER_Y, WRIST_CENTER_X, 0);

     painter->setPen((Qt::blue));
     painter->drawArc(WRIST_CENTER_X - WRIST_CIRCLE_RADIUS, WRIST_CENTER_Y - WRIST_CIRCLE_RADIUS,
                                     WRIST_CIRCLE_RADIUS * 2, WRIST_CIRCLE_RADIUS * 2,
                                     0, 360 * 64);

 }


