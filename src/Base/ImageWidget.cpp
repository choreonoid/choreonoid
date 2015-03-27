/**
   @author Shin'ichiro Nakaoka
*/

#include "ImageWidget.h"
#include <cnoid/Image>
#include <QImage>
#include <QPaintEvent>
#include <QPainter>
#include <iostream>

using namespace std;
using namespace cnoid;


ImageWidget::ImageWidget(QWidget* parent) :
    QWidget(parent)
{
    QPalette p(palette());
    p.setColor(QPalette::Background, Qt::black);
    setPalette(p);
    setAutoFillBackground(true);

    isScalingEnabled_ = false;
    fitted = false;
    settedT = false;
}


void ImageWidget::setScalingEnabled(bool on)
{
    if(on != isScalingEnabled_){
        isScalingEnabled_ = on;
        update();
    }
}

    
bool ImageWidget::isScalingEnabled() const
{
    return isScalingEnabled_;
}


void ImageWidget::setPixmap(const QPixmap& pixmap)
{
    pixmap_ = pixmap;
    fitCenter();
    update();
}


void ImageWidget::setImage(const QImage& image)
{
    pixmap_ = QPixmap::fromImage(image);
    fitCenter();
    update();
}


void ImageWidget::setImage(const Image& image)
{
    static QImage::Format componentSizeToFormat[] = {
        QImage::Format_Invalid,
        QImage::Format_Invalid, //! \todo convert a gray scale image to RGB888
        QImage::Format_Invalid,
        QImage::Format_RGB888
    };

    QImage::Format f = componentSizeToFormat[image.numComponents()];

    if(f != QImage::Format_Invalid){
        pixmap_ = QPixmap::fromImage(
            QImage(image.pixels(), image.width(), image.height(), f));
    }

    fitCenter();
    update();
}


void ImageWidget::zoom(double scale)
{
	if(pixmap_.isNull()){
		return;
	}

	QSize r = rect().size();
	QTransform invT = transform_.inverted();
	double x,y;
	invT.map(r.width()/2,r.height()/2,&x,&y);

	transform_.translate(x,y);
	transform_.scale(scale, scale);
	transform_.translate(-x,-y);

	update();
}


void ImageWidget::translate(QPoint pos)
{
	if(pixmap_.isNull()){
		return;
	}

	QTransform T(transform_.m11(), transform_.m12(),transform_.m21(), transform_.m22(), 0,0);
	QTransform invT = T.inverted();
	double x,y;
	invT.map(pos.x(), pos.y(),&x,&y);
	transform_.translate(x,y);

	update();
}


void ImageWidget::rotate(double rotation)
{
	if(pixmap_.isNull()){
		return;
	}

	QSize r = rect().size();
	QTransform invT = transform_.inverted();
	double x,y;
	invT.map(r.width()/2,r.height()/2,&x,&y);

	transform_.translate(x,y);
	transform_.rotate(rotation);
	transform_.translate(-x,-y);

	update();
}


void ImageWidget::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);

    if(pixmap_.isNull()){
        return;
    }

    //for(int i=0; i<r.width(); i+=50)
    //   	painter.drawLine(i,0,i,r.height());
    //for( int j=0; j<r.height(); j+=50)
    //   	painter.drawLine(0,j,r.width(),j);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setWorldTransform(transform_);

    painter.drawPixmap(0, 0, pixmap_);
}


void ImageWidget::fitCenter()
{
    if(fitted)
        return;

    if(settedT){
        resize(rect().size());
        return;
    }

    QSize r = rect().size();
    QSize s = pixmap_.size();
    double scale = 1.0;
    if(isScalingEnabled_){
        s.scale(r, Qt::KeepAspectRatio);
        scale = (double)s.width() / (double)pixmap_.size().width();
        transform_.scale(scale, scale);
    }
    double x = (r.width() - s.width()) / 2;
    double y = (r.height() - s.height()) / 2;
    transform_.translate(x/scale, y/scale);

    oldSize = r;
    fitted = true;
}


QSize ImageWidget::sizeHint() const
{
    if(isScalingEnabled_){
        return QSize(-1, -1);
    } else {
        return pixmap_.size();
    }
}


void ImageWidget::resizeEvent(QResizeEvent *event)
{
    if(pixmap_.isNull())
            return;

    resize(event->size());
}


void ImageWidget::resize(const QSize& size)
{
    if(isScalingEnabled_){
        if(size.width() <= 0 || size.height() <= 0){
            return;
        }
        double wscale = (double)size.width() / (double)oldSize.width();
        double hscale = (double)size.height() / (double)oldSize.height();

        QRect rect(0, 0, pixmap_.width(), pixmap_.height());
        QRect trect = transform_.mapRect(rect);
        if(trect.width() < size.width() && wscale < 1.0)
            wscale = 1.0;
        if(trect.height() < size.height() && hscale < 1.0)
            hscale = 1.0;

        QSize difSize = size - oldSize;
        double scale = 1;
        if(!difSize.width())
            scale = hscale;
        else if(!difSize.height())
            scale = wscale;
        else
            scale = wscale < hscale ? wscale : hscale;

        QTransform invT = transform_.inverted();
        double cx = (double)oldSize.width()/2.0;
        double cy = (double)oldSize.height()/2.0;
        double x,y;
        invT.map(cx, cy, &x, &y);
        transform_.translate(x,y);
        transform_.scale(scale, scale);
        transform_.translate(-x,-y);

        QTransform T(transform_.m11(), transform_.m12(),transform_.m21(), transform_.m22(), 0,0);
        invT = T.inverted();
        double dx = ((double)size.width()-(double)oldSize.width())/2.0;
        double dy = ((double)size.height()-(double)oldSize.height())/2.0;
        invT.map(dx, dy,&x,&y);
        transform_.translate(x, y);

        oldSize = size;
    }
}


void ImageWidget::setTransform(const QTransform& transform)
{
    std::cout << "setTransform" << std::endl;
    transform_ = transform;
    oldSize = rect().size();
    settedT = true;
}
