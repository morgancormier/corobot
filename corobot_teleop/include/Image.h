#ifndef IMAGE_H
#define IMAGE_H

#include <QGraphicsItem>
#include <QImage>


 class Image : public QGraphicsObject
         //transform a Qimage in a qGraphicsItem
 {
	Q_OBJECT

 public:
    Image();
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void setImage(QImage& image);//set the QImage

 private :
    QImage image;

    
 public slots:
    void saveImage(bool save); // save a picture of the current image stream on the hardrive. The date and time is given in the file name

 };

#endif // IMAGE_H
