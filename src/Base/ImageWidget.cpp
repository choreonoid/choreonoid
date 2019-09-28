/**
   @author Shin'ichiro Nakaoka
*/

#include "ImageWidget.h"
#include <cnoid/Image>
#include <QImage>
#include <QPaintEvent>
#include <QPainter>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cnoid;


ImageWidget::ImageWidget(QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f)
{
    QPalette p(palette());
    p.setColor(QPalette::Background, Qt::black);
    setPalette(p);
    setAutoFillBackground(true);

    isScalingEnabled_ = false;
    fitted = false;
    settedT = false;
    initialTransform_.reset();
    transform_.reset();
}


void ImageWidget::setScalingEnabled(bool on)
{
    if(on != isScalingEnabled_){
        isScalingEnabled_ = on;

        if(pixmap_.isNull())
            return;
        reset();
    }
}

    
bool ImageWidget::isScalingEnabled() const
{
    return isScalingEnabled_;
}


void ImageWidget::setPixmap(const QPixmap& pixmap)
{
    std::lock_guard<std::mutex> lock(mtx);
    pixmap_ = pixmap;
    fitCenter();
    update();
}


void ImageWidget::setImage(const QImage& image)
{
    std::lock_guard<std::mutex> lock(mtx);
    pixmap_ = QPixmap::fromImage(image);
    fitCenter();
    update();
}


void ImageWidget::setImage(const Image& image)
{
    if(image.width() * image.height()==0)
        return;

    std::lock_guard<std::mutex> lock(mtx);
    static QImage::Format componentSizeToFormat[] = {
        QImage::Format_Invalid,
        QImage::Format_Invalid, //! \todo convert a gray scale image to RGB888
        QImage::Format_Invalid,
        QImage::Format_RGB888,
        QImage::Format_RGBA8888
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
    std::lock_guard<std::mutex> lock(mtx);
    if(pixmap_.isNull())
        return;

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
    std::lock_guard<std::mutex> lock(mtx);
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


void ImageWidget::rotate(double angle)
{
    std::lock_guard<std::mutex> lock(mtx);
	QSize r = rect().size();
	QTransform invT = transform_.inverted();
	double x,y;
	invT.map(r.width()/2,r.height()/2,&x,&y);

	transform_.translate(x,y);
	transform_.rotate(angle);
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
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    painter.setWorldTransform(transform_);
    pixmap_.setDevicePixelRatio(devicePixelRatio());
    painter.drawPixmap(0, 0, pixmap_);
}


void ImageWidget::fitCenter()
{
    if(fitted)
        return;

    if(settedT){
        oldSize = pixmap_.size();
        oldScale = 1.0;
        resize(rect().size());
        fitted = true;
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

    oldScale = scale;
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
    std::lock_guard<std::mutex> lock(mtx);
    if(pixmap_.isNull())
            return;

    resize(event->size());
}


void ImageWidget::resize(const QSize& size)
{
    if(size.width() <= 0 || size.height() <= 0){
        return;
    }

    if(isScalingEnabled_ ){
        QSize s = pixmap_.size();
        s.scale(size, Qt::KeepAspectRatio);
        double newScale = (double)s.width() / (double)pixmap_.size().width();
        double scale = newScale / oldScale;
        oldScale = newScale;

        QTransform invT = transform_.inverted();
        double cx = (double)oldSize.width()/2.0;
        double cy = (double)oldSize.height()/2.0;
        double x,y;
        invT.map(cx, cy, &x, &y);
        transform_.translate(x,y);
        transform_.scale(scale, scale);
        transform_.translate(-x,-y);
    }

    QTransform T(transform_.m11(), transform_.m12(),transform_.m21(), transform_.m22(), 0,0);
    QTransform invT = T.inverted();
    double dx = ((double)size.width()-(double)oldSize.width())/2.0;
    double dy = ((double)size.height()-(double)oldSize.height())/2.0;
    double x,y;
    invT.map(dx, dy,&x,&y);
    transform_.translate(x, y);

    oldSize = size;
}


void ImageWidget::setTransform(const QTransform& transform)
{
    transform_ = transform;
    settedT = true;
    initialTransform_ = transform_;
}


bool ImageWidget::getTransform(QTransform& transform)
{
    if(pixmap_.isNull())
        return false;

    notScaledTransform_ = transform_;
    if(isScalingEnabled_ && !pixmap_.isNull()){
        QSize size = pixmap_.size();
        double scale = 1.0 / oldScale;

        QTransform invT = transform_.inverted();
        double cx = (double)oldSize.width()/2.0;
        double cy = (double)oldSize.height()/2.0;
        double x,y;
        invT.map(cx, cy, &x, &y);
        notScaledTransform_.translate(x,y);
        notScaledTransform_.scale(scale, scale);
        notScaledTransform_.translate(-x,-y);

        QTransform T(notScaledTransform_.m11(), notScaledTransform_.m12(),notScaledTransform_.m21(), notScaledTransform_.m22(), 0,0);
        invT = T.inverted();
        double dx = ((double)size.width()-(double)oldSize.width())/2.0;
        double dy = ((double)size.height()-(double)oldSize.height())/2.0;
        invT.map(dx, dy,&x,&y);
        notScaledTransform_.translate(x, y);
    }
    transform = notScaledTransform_;
    return true;
}

double ImageWidget::getAngle(){
    double scale;
    if(pixmap_.isNull())
        scale = 1.0;
    else
        scale = oldScale;

    notScaledTransform_ = transform_;
    notScaledTransform_ *= scale;

    return atan2(notScaledTransform_.m12(), notScaledTransform_.m11());

}


void ImageWidget::setAngle(double angle)
{
    transform_.reset();
    rotate(angle);
    initialTransform_ = transform_;
}


void ImageWidget::reset()
{
    std::lock_guard<std::mutex> lock(mtx);
    transform_ = initialTransform_;
    fitted = false;
    settedT = true;

    fitCenter();
    update();
}


Image& ImageWidget::getImage()
{
    std::lock_guard<std::mutex> lock(mtx);

    if(pixmap_.isNull()){
        transformedImage.setSize(0,0,1);
        return transformedImage;
    }

    QImage image(rect().size(), QImage::Format_RGB888);
    image.fill(QColor(0,0,0));
    QPainter painter(&image);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    painter.setWorldTransform(transform_);
    painter.drawPixmap(0, 0, pixmap_);

    transformedImage.setSize(image.width(), image.height(), 3);
    unsigned char* p = transformedImage.pixels();
    for(int i=0; i<image.height(); i++){
        for(int j=0; j<image.width(); j++){
            QRgb rgb = image.pixel(j, i);
            *p++ = qRed(rgb);
            *p++ = qGreen(rgb);
            *p++ = qBlue(rgb);
        }
    }

    return transformedImage;
}

void ImageWidget::clear()
{
    std::lock_guard<std::mutex> lock(mtx);
    pixmap_.fill(Qt::black);
    update();
}

