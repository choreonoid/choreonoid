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
    transform_.reset();
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


void ImageWidget::setTransform(QTransform& transform)
{
	transform_ = transform;
	update();
}


void ImageWidget::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);

    if(pixmap_.isNull()){
        return;
    }

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
