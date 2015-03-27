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

//public Q_SLOTS:
    void setPixmap(const QPixmap& pixmap);
    void setImage(const Image& image);
    void setImage(const QImage& image);

    void setScalingEnabled(bool on);
    bool isScalingEnabled() const;

    void zoom(double scale);
    void translate(QPoint pos);
    void rotate(double rotation);
    const QTransform& transform() { return transform_; }
    void setTransform(const QTransform& transform);

protected:
    virtual void paintEvent(QPaintEvent* event);
    virtual QSize sizeHint() const;
    virtual void resizeEvent(QResizeEvent *event);
        
private:
    QPixmap pixmap_;
    bool isScalingEnabled_;
    QTransform transform_;

    void fitCenter();
    void resize(const QSize& size);
    bool fitted;
    QSize oldSize;
    bool settedT;

};
}

#endif
