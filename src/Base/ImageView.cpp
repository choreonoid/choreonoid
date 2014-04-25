/**
   @author Shin'ichiro Nakaoka
*/

#include "ImageView.h"
#include "ImageWidget.h"
#include "ViewManager.h"
#include <cnoid/Image>
#include <QImage>
#include <QBoxLayout>
#include "gettext.h"

using namespace cnoid;

namespace cnoid {
    
class ImageViewImpl
{
public:
    ImageWidget* imageWidget;
};
}


void ImageView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<ImageView>(
        "ImageView", N_("Image"), ViewManager::MULTI_OPTIONAL);
}

    
ImageView::ImageView()
{
    impl = new ImageViewImpl;
    
    setDefaultLayoutArea(View::CENTER);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);
    impl->imageWidget = new ImageWidget;
    impl->imageWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    vbox->addWidget(impl->imageWidget, 1);
    setLayout(vbox);
}


ImageView::~ImageView()
{
    delete impl;
}


void ImageView::setPixmap(const QPixmap& pixmap)
{
    impl->imageWidget->setPixmap(pixmap);
}


void ImageView::setImage(const QImage& image)
{
    impl->imageWidget->setImage(image);
}


void ImageView::setImage(const Image& image)
{
    impl->imageWidget->setImage(image);
}


void ImageView::setScalingEnabled(bool on)
{
    impl->imageWidget->setScalingEnabled(on);
}


bool ImageView::isScalingEnabled() const
{
    return impl->imageWidget->isScalingEnabled();
}
