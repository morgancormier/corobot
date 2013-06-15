#ifndef ARMCIRCLEWIDGET_H
#define ARMCIRCLEWIDGET_H

#include <QtGui/QGraphicsView>


class ArmCircleWidget : public QGraphicsView
 {
     Q_OBJECT

 public:
     ArmCircleWidget(QWidget *parent = 0);

 public slots:
      void setpos(float x, float y); // set the position of the circle in the widget. x,y is the position of the gripper, in meters.
      void Corobot(bool value); // value is true of a Corobot, false if is an Explorer, Explorer doesn't have an arm therefore the Widget is white

 protected:
     void drawBackground(QPainter *painter, const QRectF &rect); //draw the background of the widget
     void scaleView(qreal scaleFactor); //change the scale of the view.

 };

#endif // ARMCIRCLEWIDGET_H
