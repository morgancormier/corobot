#ifndef CAMERA_H
#define CAMERA_H

#include <QtGui/QGraphicsView>

class CameraWidget : public QGraphicsView
 {
     Q_OBJECT

 public:
     CameraWidget(QWidget *parent = 0);


 protected:
     void keyPressEvent(QKeyEvent *event);
     void timerEvent(QTimerEvent *event);
     void wheelEvent(QWheelEvent *event);
     void drawBackground(QPainter *painter, const QRectF &rect);
     void scaleView(qreal scaleFactor);

 private:
     int timerId;

 };
#endif // CAMERA_H
