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

#include "Camera.h"

#include <QtGui>

const int LINE_POS = 70;

// Widget capable not only of displaying the camera image but also to zoom in and out on the image. 
// Of course the zoom is a software zoom, even using a camera with hardware zoom.

CameraWidget::CameraWidget(QWidget *parent)
    : QGraphicsView(parent), timerId(0)
{

    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene->setSceneRect(0, 00, 440,380);


    setScene(scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    scale(qreal(1), qreal(1));
    setMinimumSize(210, 110);
    setWindowTitle(tr("Camera"));


    QGraphicsLineItem *line = new QGraphicsLineItem(this->sceneRect().right()-LINE_POS ,this->sceneRect().top(),this->sceneRect().right()-LINE_POS ,this->sceneRect().bottom());
    scene->addItem(line);


timerId = startTimer(1000 / 25); // Used to animate the zooming in and out to have this fluid animation
 }


void CameraWidget::keyPressEvent(QKeyEvent *event)
 {
     switch (event->key()) {

     case Qt::Key_Plus:
         scaleView(qreal(1.2));
         break;
     case Qt::Key_Minus:
         scaleView(1 / qreal(1.2));
         break;
     default:
         QGraphicsView::keyPressEvent(event);
     }
 }
void CameraWidget::timerEvent(QTimerEvent *event)
// Animate the zooming in and out to have this fluid animation
 {
     Q_UNUSED(event);

     QList<QGraphicsPixmapItem *> pictures;
         foreach (QGraphicsItem *item, scene()->items()) {
             if (QGraphicsPixmapItem *p = qgraphicsitem_cast<QGraphicsPixmapItem *>(item))
                 pictures << p;
         }


         foreach (QGraphicsPixmapItem *p, pictures){

             if(p->scale()>0.1){
             if((p->pixmap().size().width()*0.5+p->pos().x()) > (this->sceneRect().right() - LINE_POS)){
                 p->setPos(this->sceneRect().right()-70,p->pos().y());
                p->setScale(0.1);

                }
             }
             else{
             if((p->pixmap().size().width()*0.1+p->pos().x()) < (this->sceneRect().right() - LINE_POS)){
                  p->setScale(0.5);
                 p->setPos(p->pos().x()+(p->pixmap().size().width()*0.1)-p->pixmap().size().width()*0.5,p->pos().y());
                }
            }

         }
 }



 void CameraWidget::wheelEvent(QWheelEvent *event)
 // Use the wheel to zoom in and out
 {
     scaleView(pow((double)2, -event->delta() / 240.0));
 }

 void CameraWidget::drawBackground(QPainter *painter, const QRectF &rect)
 // Draw the background of the view, seem when zoomed out.
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



 }

 void CameraWidget::scaleView(qreal scaleFactor)
 //method used to change the scale of the view
 {
     qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
     if (factor < 0.07 || factor > 100)
         return;

     scale(scaleFactor, scaleFactor);
 }


