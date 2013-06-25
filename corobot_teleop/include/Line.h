#ifndef LINE_H
#define LINE_H
#include <QtGui>

// Constants necessary for the position of the lines in the wrist widget
const int WRIST_X = 48;
const int WRIST_Y = 48;
const int WRIST_CENTER_X = WRIST_X / 2;
const int WRIST_CENTER_Y = WRIST_Y / 2;
const int WRIST_CIRCLE_RADIUS = WRIST_X / 4;
const int WRIST_WIDTH = WRIST_X * 9 / 10;
const int WRIST_HEIGHT = WRIST_Y / 5;
const int WRIST_LINE_WIDTH = 3;

class Line : public QGraphicsLineItem
{
public:
    Line(int x1, int y1, int x2, int y2) : QGraphicsLineItem(x1,y1,x2,y2)

  {
    rotation = 0;

  }

  void mousePressEvent(QGraphicsSceneMouseEvent *event)
  {
    initialPos = mapToScene(event->pos());
    QGraphicsItem::mousePressEvent(event);
  }

  void mouseMoveEvent(QGraphicsSceneMouseEvent *event)
	// calculate the rotation angle the user want the line to be at and draw the line with the new rotation angle
  {

      //rotate around the center of the line

    QPointF pos = mapToScene(event->pos());

    int x1 = initialPos.x()- WRIST_CENTER_X;
    int x2 = pos.x()- WRIST_CENTER_X;
    int y1 = initialPos.y()- WRIST_CENTER_Y;
    int y2 = pos.y()- WRIST_CENTER_Y;

    rotation +=  (atan2(y2,x2) - atan2(y1,x1))/M_PI*180;


    setRotation(rotation);
    initialPos = pos;
  }
  QPointF initialPos;
  qreal rotation;

};
#endif // LINE_H
