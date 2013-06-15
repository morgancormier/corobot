#include "Camera.h"

#include <QtGui>

const int LINE_POS = 70;

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


timerId = startTimer(1000 / 25);
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
 {
     scaleView(pow((double)2, -event->delta() / 240.0));
 }

 void CameraWidget::drawBackground(QPainter *painter, const QRectF &rect)
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
 {
     qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
     if (factor < 0.07 || factor > 100)
         return;

     scale(scaleFactor, scaleFactor);
 }


