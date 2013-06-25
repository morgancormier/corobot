#ifndef MYQGRAPHICSVIEW_H
#define MYQGRAPHICSVIEW_H

#include <QGraphicsView>
#include <QTimeLine>
#include <QWheelEvent>
class MyQGraphicsView: public QGraphicsView
{
	Q_OBJECT

	int _numScheduledScalings;
	void wheelEvent ( QWheelEvent * event );
	
	public slots:
	void scalingTime(qreal x);
	void animFinished();

	public: 
	qreal scale_factor;
	MyQGraphicsView(QWidget*);
	
};

#endif
