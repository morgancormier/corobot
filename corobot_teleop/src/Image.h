#ifndef IMAGE_H
#define IMAGE_H

#include <QGraphicsItem>
#include <QImage>


 class Image : public QGraphicsItem
         //transform a Qimage in a qGraphicsItem
 {
 public:
     Image();

     enum { Type = UserType + 1 };
     int type() const { return Type; }


     QRectF boundingRect() const;
//     QPainterPath shape() const;
     void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);


    void setImage(QImage& image);//set the QImage
    void saveImage();
 private :
         QImage image;

 };

#endif // IMAGE_H
