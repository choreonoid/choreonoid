/**
   @author Shin'ichiro Nakaoka
*/

#include "ImageView.h"
#include "ImageWidget.h"
#include "ViewManager.h"
#include "RootItem.h"
#include "ComboBox.h"
#include <cnoid/ConnectionSet>
#include <cnoid/Image>
#include <cnoid/ImageProvider>
#include <QImage>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

Q_DECLARE_METATYPE(ImageProvider *)

namespace {
vector<ImageView*> instances;
}

namespace cnoid {

class ImageViewImpl
{
public:
    ImageView* self;
    ImageWidget* imageWidget;

    Connection sigUpdatedConnection;
    ImageProvider* imageProvider;
    Item* imageProviderItem;

    ImageViewImpl(ImageView* self);
    ~ImageViewImpl();

    void updateImage();
    void setImageProvider(ImageProvider* imageProvider_, Item* item);
    void clear();

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

};

class ImageViewBarImpl
{
public:
    ImageViewBar* self;
    ComboBox* imageCombo;
    Connection sigItemAddedConnection;
    Connection sigViewActivatedConnection;
    typedef map<ImageProvider*, ConnectionSet> ImageProviderConnectionsMap;
    ImageProviderConnectionsMap imageProviderConnections;
    typedef map<ImageView*, ConnectionSet> ViewConnectionsMap;
    ViewConnectionsMap viewConnections;
    ImageView* targetView;
    typedef map<ImageProvider*, Item*> ImageProviderItemMap;
    ImageProviderItemMap imageProviderItems;

    ImageViewBarImpl(ImageViewBar* self);
    ~ImageViewBarImpl();
    ToolButton* adjustSizeToggle;
    void onItemAdded(Item* item);
    void onImageComboCurrentIndexChanged(int index);
    void onItemDetachedFromRoot(Item* item);
    void onItemNameChange(Item* item);
    void onViewActivated(View* view);
    void onForcusViewChanged(View* view);
    void onViewDeactivated(ImageView* imageView);
    void onAdjustSizeClicked(bool on);
    ImageProvider* getSelectedImageProvider();
    ImageProvider* getImageProvider(int index);
};

}


ImageViewImpl::ImageViewImpl(ImageView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::CENTER);

    QVBoxLayout* vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    imageWidget = new ImageWidget;
    imageWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    imageWidget->setScalingEnabled(true);
    vbox->addWidget(imageWidget, 1);
    self-> setLayout(vbox);
    self->setFocusPolicy( Qt::StrongFocus );

    ImageViewBar* imageViewBar = ImageViewBar::instance();
    imageProvider = 0;
    imageProviderItem = 0;
    ImageProvider* imageProvider_ = imageViewBar->getSelectedImageProvider();
    Item* item = imageViewBar->getImageProviderItem(imageProvider_);
    setImageProvider(imageProvider_, item);

}


ImageViewImpl::~ImageViewImpl()
{
    instances.erase(std::find(instances.begin(), instances.end(), self));

    sigUpdatedConnection.disconnect();
}


void ImageViewImpl::updateImage()
{
    if(imageProvider){
        auto image = imageProvider->getImage();
        if(image){
            imageWidget->setImage(*imageProvider->getImage());
            return;
        }
    }
    imageWidget->clear();
}


void ImageViewImpl::clear()
{
    imageWidget->clear();
}


void ImageView::initializeClass(ExtensionManager* ext)
{
    if(instances.empty()){
        ext->viewManager().registerClass<ImageView>(
                "ImageView", N_("Image"), ViewManager::MULTI_OPTIONAL);
    }
}


ImageView* ImageView::instance()
{
    if(instances.empty()){
        return 0;
    }
    return instances.front();
}


ImageView::ImageView()
{
    impl = new ImageViewImpl(this);
    instances.push_back(this);

    impl->updateImage();
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


ImageProvider* ImageView::getImageProvider()
{
    return impl->imageProvider;
}


void ImageView::setImageProvider(ImageProvider* imageProvider, Item* item)
{
    impl->setImageProvider(imageProvider, item);

}


void ImageViewImpl::setImageProvider(ImageProvider* imageProvider_, Item* item)
{
    if(imageProvider == imageProvider_)
        return;

    if(imageProvider){
        clear();
        sigUpdatedConnection.disconnect();
    }

    imageProvider = imageProvider_;

    if(imageProvider){
        sigUpdatedConnection =
            imageProvider->sigImageUpdated().connect( [&](){ updateImage(); } );
        updateImage();
    }

    imageProviderItem = item;
}


bool ImageView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool ImageViewImpl::storeState(Archive& archive)
{
    archive.writeItemId("ImageProviderItem", imageProviderItem);
    return true;
}


bool ImageView::restoreState(const Archive& archive)
{
    archive.addPostProcess([&](){ impl->restoreState(archive); });
    return true;
}


bool ImageViewImpl::restoreState(const Archive& archive)
{
    Item* item = archive.findItem<Item>("ImageProviderItem");
    ImageProvider* imageProvider = dynamic_cast<ImageProvider*>(item);
    if(item && imageProvider){
        setImageProvider(imageProvider, item);
    }

    return true;
}


void ImageViewBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(instance());

    }
}


ImageViewBar* ImageViewBar::instance()
{
    static ImageViewBar* imageViewBar = new ImageViewBar();
    return imageViewBar;
}


ImageViewBar::ImageViewBar()
    : ToolBar(N_("ImageViewBar"))
{
    impl = new ImageViewBarImpl(this);
}


ImageViewBar::~ImageViewBar()
{
    delete impl;
}


ImageViewBarImpl::ImageViewBarImpl(ImageViewBar* self)
    : self(self)
{
    targetView = 0;
    imageProviderItems[0] = 0;

    imageCombo = new ComboBox();
    imageCombo->setToolTip(_("Select a ImageProvider Item"));
    imageCombo->setMinimumContentsLength(6);
    imageCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    imageCombo->sigCurrentIndexChanged().connect(
            [&](int index){ onImageComboCurrentIndexChanged(index); });
    imageCombo->addItem("No selection", QVariant::fromValue(0));
    imageCombo->setCurrentIndex(0);
    self->addWidget(imageCombo);

    adjustSizeToggle = self->addToggleButton(QIcon(":/Base/icons/adjustSize.png"), _("Adjust image size according to view"));
    adjustSizeToggle->sigToggled().connect( [&](bool on){ onAdjustSizeClicked( on ); } );
    adjustSizeToggle->setChecked(true);

    RootItem* rootItem = RootItem::instance();
    sigItemAddedConnection =
        rootItem->sigItemAdded().connect( [&](Item* item){ onItemAdded(item); } );

    sigViewActivatedConnection =
            ViewManager::sigViewActivated().connect( [&](View* view){ onViewActivated(view); } );
}


ImageViewBarImpl::~ImageViewBarImpl()
{
    sigItemAddedConnection.disconnect();
    sigViewActivatedConnection.disconnect();
}

void ImageViewBarImpl::onItemAdded(Item* item)
{
    ImageProvider* imageProvider = dynamic_cast<ImageProvider*>(item);

    if(imageProvider && imageCombo->findData(QVariant::fromValue(imageProvider))==-1){

        ConnectionSet& connections = imageProviderConnections[imageProvider];
        connections.add(
            item->sigDisconnectedFromRoot().connect(
                [&, item](){ onItemDetachedFromRoot(item); }) );
        connections.add(
            item->sigNameChanged().connect(
                [&, item](const string& /* oldName */) { onItemNameChange(item); }) );

        imageCombo->addItem(item->name().c_str(), QVariant::fromValue(imageProvider));

        imageProviderItems[imageProvider] = item;
    }
}


void ImageViewBarImpl::onItemDetachedFromRoot(Item* item)
{
    ImageProvider* imageProvider = dynamic_cast<ImageProvider*>(item);
    if(!imageProvider)
        return;

    int removeIndex = imageCombo->findData(QVariant::fromValue(imageProvider));
    imageCombo->removeItem(removeIndex);

    ConnectionSet& connections = imageProviderConnections[imageProvider];
    connections.disconnect();

    imageProviderConnections.erase(imageProvider);

    imageProviderItems.erase(imageProvider);
}


void ImageViewBarImpl::onItemNameChange(Item* item)
{
    int index = imageCombo->findData(QVariant::fromValue(dynamic_cast<ImageProvider*>(item)));
    imageCombo->setItemText(index, QString(item->name().c_str()));
}


void ImageViewBarImpl::onImageComboCurrentIndexChanged(int index)
{
    if(index == -1)
        return;

    if(targetView){
        ImageProvider* imageProvider = getImageProvider(index);
        targetView->setImageProvider(imageProvider, imageProviderItems[imageProvider]);
    }

}


ImageProvider* ImageViewBar::getSelectedImageProvider()
{
    return impl->getSelectedImageProvider();
}


ImageProvider* ImageViewBarImpl::getSelectedImageProvider()
{
    int index = imageCombo->currentIndex();
    return getImageProvider(index);
}


ImageProvider* ImageViewBarImpl::getImageProvider(int index)
{
    const QVariant qv = imageCombo->itemData(index);
    if(qv.isValid()){
        return qv.value<ImageProvider*>();
    }else{
        return 0;
    }
}


Item* ImageViewBar::getImageProviderItem(ImageProvider* imageProvider)
{
    return impl->imageProviderItems[imageProvider];
}


void ImageViewBarImpl::onViewActivated(View* view)
{
    ImageView* imageView = dynamic_cast<ImageView*>(view);
    if(imageView){
        ConnectionSet& connections = viewConnections[imageView];
        connections.add( imageView->sigFocusChanged().connect(
            [&](View* view){ onForcusViewChanged(view); } ) );
        connections.add( imageView->sigDeactivated().connect(
            [&, imageView](){ onViewDeactivated(imageView); } ) );
    }
}


void ImageViewBarImpl::onForcusViewChanged(View* view)
{
    ImageView* imageView = dynamic_cast<ImageView*>(view);
    if(imageView){
        targetView = imageView;
        ImageProvider* imageProvider = targetView->getImageProvider();
        if(imageProvider){
            int index = imageCombo->findData(QVariant::fromValue(imageProvider));
            imageCombo->setCurrentIndex(index);
        }else{
            imageCombo->setCurrentIndex(0);
        }

        if( targetView->isScalingEnabled() ){
            adjustSizeToggle->setChecked(true);
        }else{
            adjustSizeToggle->setChecked(false);
        }
    }

}


void ImageViewBarImpl::onViewDeactivated(ImageView* imageView)
{
    if(targetView == imageView){
        targetView = 0;
    }

    ConnectionSet& connections = viewConnections[imageView];
    connections.disconnect();

    viewConnections.erase(imageView);
}


void ImageViewBarImpl::onAdjustSizeClicked(bool on)
{
    if(targetView){
        targetView->setScalingEnabled(on);
    }
}

