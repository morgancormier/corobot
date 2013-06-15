 #include <QGraphicsScene>
 #include <QPainter>
 #include <QStyleOption>

 #include "Image.h"

 Image::Image()
 {



 }


 QRectF Image::boundingRect() const
 {
     qreal adjust = 2;
     int height = image.height();
     int width = image.width();
     return QRectF(0, 0,width + adjust, width + adjust);
 }

 void Image::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
 {
     painter->drawImage(0,0,image);
 }

 void Image::setImage(QImage& image)
 //set the QImage
 {
    this->image = image;

 }


 void Image::saveImage()
 {

     QString multifn;
     multifn ="../pictest.jpg";

     this->image.save(multifn,"JPG");

 }
