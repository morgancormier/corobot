#ifndef CAMERA_H
#define CAMERA_H

#include <QtGui/QGraphicsView>

// Widget capable not only of displaying the camera image but also to zoom in and out on the image. 
// Of course the zoom is a software zoom, even using a camera with hardware zoom.
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
