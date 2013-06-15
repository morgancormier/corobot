
#include "ArmCircleWidget.h"

#include <QtGui>


ArmCircleWidget::ArmCircleWidget(QWidget *parent)
    : QGraphicsView(parent)
{

    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene->setSceneRect(0, 0, 40, 40);


    setScene(scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    scale(qreal(1), qreal(1));
    setMinimumSize(49, 49);
    setWindowTitle(tr("Arm Position"));


    QGraphicsEllipseItem *circle = new QGraphicsEllipseItem(25-5,25-5,10,10);

    scene->addItem(circle);



 }

void ArmCircleWidget::Corobot(bool value)
// value is true of a Corobot, false if is an Explorer, Explorer doesn't have an arm therefore the Widget is white
{
            if(value == true){
                scene()->items().at(0)->show();
            }
            else{
                scene()->items().at(0)->hide();
            }
}

void ArmCircleWidget::setpos(float x, float y)
// set the position of the circle in the widget. x,y is the position of the gripper, in meters.
{
    QGraphicsEllipseItem * e = qgraphicsitem_cast<QGraphicsEllipseItem  *>(scene()->items().at(0));
    float r = 18/(2*x+1);
    float ybis = 22 - 47*y;
    e->setRect(20-r/2,ybis-r/2,r,r);
}




 void ArmCircleWidget::drawBackground(QPainter *painter, const QRectF &rect)
 //draw the background of the widget
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

 void ArmCircleWidget::scaleView(qreal scaleFactor)
 //change the scale of the view
 {
     qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
     if (factor < 0.07 || factor > 100)
         return;

     scale(scaleFactor, scaleFactor);
 }



