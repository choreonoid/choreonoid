#ifndef CNOID_BASE_IMAGE_WIDGET_H
#define CNOID_BASE_IMAGE_WIDGET_H

#include <QWidget>
#include <cnoid/Image>
#include <mutex>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ImageWidget : public QWidget
{
public:
    explicit ImageWidget(QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());

    void setPixmap(const QPixmap& pixmap, bool doRest = false);
    void setImage(const Image& image, bool doRest = false);
    void setImage(const QImage& image, bool doRest = false);

    void setScalingEnabled(bool on);
    bool isScalingEnabled() const;

    void zoom(double scale);
    void translate(QPoint pos);

    bool getTransform(QTransform& transform);
    double getAngle();
    void setTransform(const QTransform& transform);
    void setAngle(double angle);
    void rotate(double angle);
    void reset();
    Image& getImage();
    void clear();

protected:
    virtual void paintEvent(QPaintEvent* event);
    virtual QSize sizeHint() const;
    virtual void resizeEvent(QResizeEvent *event);
        
private:
    void fitCenter();
    void resize(const QSize& size);
    
    QPixmap pixmap_;
    bool isScalingEnabled_;
    QTransform transform_;
    QTransform notScaledTransform_;
    QTransform initialTransform_;
    Image transformedImage;
    std::mutex mtx;
    bool fitted;
    QSize oldSize;
    double oldScale;
    bool settedT;
};

}

#endif
