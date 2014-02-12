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

//this class is a little circle made to represent the shoulder, elbow or gripper of an arm
//a flag(ItemIsMovable) can be set to make the joint move.


 #include <QGraphicsScene>
 #include <QGraphicsSceneMouseEvent>
 #include <QPainter>
 #include <QStyleOption>

 #include "joint.h"
 #include "ArmWidget.h"
#include "ArmRotationWidget.h"

 joint::joint(ArmWidget *graphWidget)
     : graph(graphWidget)
 {
     graph_rotation = NULL;
     setFlag(ItemSendsGeometryChanges);
     setCacheMode(DeviceCoordinateCache);
     setZValue(-1);

 }

joint::joint(ArmRotationWidget *graphWidget)
     : graph_rotation(graphWidget)
 {
     graph = NULL;
     setFlag(ItemSendsGeometryChanges);
     setCacheMode(DeviceCoordinateCache);
     setZValue(-1);

 }


 bool joint::advance()
 //update the position of the joint
 {
     if (newPos == pos())
         return false;

     setPos(newPos);
     return true;
 }

 QRectF joint::boundingRect() const
 {
     qreal adjust = 2;
     return QRectF(-10 - adjust, -10 - adjust,
                   23 + adjust, 23 + adjust);
 }

 QPainterPath joint::shape() const
 {
     QPainterPath path;
     path.addEllipse(-10, -10, 20, 20);
     return path;
 }

 void joint::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
 {
     painter->setPen(Qt::NoPen);
     painter->setBrush(Qt::darkGray);
     painter->drawEllipse(-7, -7, 10, 10);



     QRadialGradient gradient(-3, -3, 10);
     if (option->state & QStyle::State_Sunken) {
         gradient.setCenter(3, 3);
         gradient.setFocalPoint(3, 3);
         gradient.setColorAt(1, QColor(Qt::yellow).light(120));
         gradient.setColorAt(0, QColor(Qt::darkYellow).light(120));
     } else {
         gradient.setColorAt(0, Qt::yellow);
         gradient.setColorAt(1, Qt::darkYellow);
     }
     painter->setBrush(gradient);
     painter->setPen(QPen(Qt::black, 0));
     painter->drawEllipse(-10, -10, 10, 10);
 }

 QVariant joint::itemChange(GraphicsItemChange change, const QVariant &value)
// Request update of the parent graph
 {
     switch (change) {
	 if(graph != NULL)
         	graph->itemMoved();
	  if(graph_rotation != NULL)
         	graph_rotation->itemMoved();
         break;
     default:
         break;
     };

     return QGraphicsItem::itemChange(change, value);
 }

 void joint::mousePressEvent(QGraphicsSceneMouseEvent *event)
 {
     update();
     QGraphicsItem::mousePressEvent(event);
 }

 void joint::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
 {
     update();
     QGraphicsItem::mouseReleaseEvent(event);
 }
