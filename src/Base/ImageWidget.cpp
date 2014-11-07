/**
   @author Shin'ichiro Nakaoka
*/

#include "ImageWidget.h"
#include <cnoid/Image>
#include <QImage>
#include <QPaintEvent>
#include <QPainter>

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
    update();
}


void ImageWidget::setImage(const QImage& image)
{
    pixmap_ = QPixmap::fromImage(image);
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

    QSize r = event->rect().size();
    QSize s = pixmap_.size();
    if(isScalingEnabled_){
        s.scale(r, Qt::KeepAspectRatio);
        QPixmap scaled = pixmap_.scaled(s, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        QPoint o((r.width() - scaled.width()) / 2, (r.height() - scaled.height()) / 2);
        painter.drawPixmap(o, scaled);
    } else {
        QPoint o((r.width() - pixmap_.width()) / 2, (r.height() - pixmap_.height()) / 2);
        painter.drawPixmap(o, pixmap_);
    }


}


QSize ImageWidget::sizeHint() const
{
    if(isScalingEnabled_){
        return QSize(-1, -1);
    } else {
        return pixmap_.size();
    }
}
