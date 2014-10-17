/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_IMAGE_WIDGET_H_INCLUDED
#define CNOID_BASE_IMAGE_WIDGET_H_INCLUDED

#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class Image;

class CNOID_EXPORT ImageWidget : public QWidget
{
    Q_OBJECT

        public:
    explicit ImageWidget(QWidget* parent = 0);

public Q_SLOTS:
    void setPixmap(const QPixmap& pixmap);
    void setImage(const Image& image);
    void setImage(const QImage& image);

    void setScalingEnabled(bool on);
    bool isScalingEnabled() const;

    void setTransform(QTransform& transform);

protected:
    virtual void paintEvent(QPaintEvent* event);
    virtual QSize sizeHint() const;
        
private:
    QPixmap pixmap_;
    QTransform transform_;
    bool isScalingEnabled_;

};
}

#endif
