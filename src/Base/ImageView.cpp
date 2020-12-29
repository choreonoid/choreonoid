#include "ImageView.h"
#include "ImageWidget.h"
#include "ViewManager.h"
#include "RootItem.h"
#include "ImageableItem.h"
#include "ComboBox.h"
#include <cnoid/ConnectionSet>
#include <cnoid/Image>
#include <QImage>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

Q_DECLARE_METATYPE(ImageableItem *)

namespace {
vector<ImageView*> instances;
}

namespace cnoid {

class ImageView::Impl
{
public:
    ImageView* self;
    ImageWidget* imageWidget;
    ScopedConnectionSet itemConnections;
    ItemPtr currentItem;
    ImageableItem* currentImageable;

    Impl(ImageView* self);
    ~Impl();

    void updateImage();
    void setImageableItem(ImageableItem* imageable);

    bool storeState(Archive& archive);
    void restoreState(const Archive& archive);
};

class ImageViewBar::Impl
{
public:
    ImageViewBar* self;
    ComboBox* imageCombo;
    Connection sigItemAddedConnection;
    typedef map<ImageableItem*, ConnectionSet> ImageableItemConnectionsMap;
    ImageableItemConnectionsMap imageableItemConnections;
    ImageView* targetView;

    Impl(ImageViewBar* self);
    ~Impl();
    ToolButton* adjustSizeToggle;
    void onItemAdded(Item* item);
    void onImageComboCurrentIndexChanged(int index);
    void onItemDisconnectedFromRoot(Item* item);
    void onItemNameChange(Item* item);
    void onFocusViewChanged(ImageView* imageView);
    void onViewDeactivated(ImageView* imageView);
    void onAdjustSizeClicked(bool on);
    ImageableItem* getSelectedImageableItem();
    ImageableItem* getImageableItem(int index);
};

}


ImageView::Impl::Impl(ImageView* self)
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

    currentImageable = nullptr;

    setImageableItem(imageViewBar->getSelectedImageableItem());
}


ImageView::Impl::~Impl()
{
    instances.erase(std::find(instances.begin(), instances.end(), self));
}


void ImageView::Impl::updateImage()
{
    bool updated = false;
    if(currentImageable){
        if(auto image = currentImageable->getImage()){
            imageWidget->setImage(*image);
            updated = true;
        }
    }
    if(!updated){
        imageWidget->clear();
    }
}


void ImageView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<ImageView>(
        "ImageView", N_("Image"), ViewManager::MULTI_OPTIONAL);
}


ImageView* ImageView::instance()
{
    if(instances.empty()){
        return nullptr;
    }
    return instances.front();
}


ImageView::ImageView()
{
    impl = new Impl(this);
    instances.push_back(this);
    impl->updateImage();
}


ImageView::~ImageView()
{
    delete impl;
}


void ImageView::onDeactivated()
{
    ImageViewBar::instance()->impl->onViewDeactivated(this);
}


void ImageView::onFocusChanged(bool on)
{
    if(on){
        ImageViewBar::instance()->impl->onFocusViewChanged(this);
    }
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


ImageableItem* ImageView::getImageableItem()
{
    return impl->currentImageable;
}


void ImageView::setImageableItem(ImageableItem* imageable)
{
    impl->setImageableItem(imageable);
}


void ImageView::Impl::setImageableItem(ImageableItem* imageable)
{
    if(imageable != currentImageable){
        itemConnections.disconnect();
        imageWidget->clear();
        currentItem.reset();
        currentImageable = nullptr;
        if(imageable){
            currentItem = dynamic_cast<Item*>(imageable);
            if(currentItem){
                currentImageable = imageable;
                itemConnections.add(
                    currentItem->sigDisconnectedFromRoot().connect(
                        [&](){ setImageableItem(nullptr); }));
                itemConnections.add(
                    imageable->sigImageUpdated().connect(
                        [&](){ updateImage(); }));
                updateImage();
            }
        }
    }
}


bool ImageView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool ImageView::Impl::storeState(Archive& archive)
{
    if(currentItem){
        archive.writeItemId("imageable_item", currentItem);
    }
    return true;
}


bool ImageView::restoreState(const Archive& archive)
{
    archive.addPostProcess([&](){ impl->restoreState(archive); });
    return true;
}


void ImageView::Impl::restoreState(const Archive& archive)
{
    auto item = dynamic_cast<ImageableItem*>(archive.findItem<Item>("imageable_item"));
    if(!item){
        item = dynamic_cast<ImageableItem*>(archive.findItem<Item>("ImageableItem"));
    }
    if(item){
        setImageableItem(item);
    }
}


void ImageViewBar::initialize(ExtensionManager* ext)
{
    ext->addToolBar(instance());
}


ImageViewBar* ImageViewBar::instance()
{
    static ImageViewBar* imageViewBar = new ImageViewBar;
    return imageViewBar;
}


ImageViewBar::ImageViewBar()
    : ToolBar(N_("ImageViewBar"))
{
    impl = new Impl(this);
}


ImageViewBar::~ImageViewBar()
{
    delete impl;
}


ImageViewBar::Impl::Impl(ImageViewBar* self)
    : self(self)
{
    targetView = nullptr;

    imageCombo = new ComboBox();
    imageCombo->setToolTip(_("Select an imageable item"));
    imageCombo->setMinimumContentsLength(6);
    imageCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    imageCombo->sigCurrentIndexChanged().connect(
            [&](int index){ onImageComboCurrentIndexChanged(index); });
    imageCombo->addItem("No selection", QVariant::fromValue(0));
    imageCombo->setCurrentIndex(0);
    self->addWidget(imageCombo);

    adjustSizeToggle = self->addToggleButton(QIcon(":/Base/icon/adjustsize.svg"), _("Adjust image size according to view"));
    adjustSizeToggle->sigToggled().connect( [&](bool on){ onAdjustSizeClicked( on ); } );
    adjustSizeToggle->setChecked(true);

    RootItem* rootItem = RootItem::instance();
    sigItemAddedConnection =
        rootItem->sigItemAdded().connect( [&](Item* item){ onItemAdded(item); } );
}


ImageViewBar::Impl::~Impl()
{
    sigItemAddedConnection.disconnect();
}


void ImageViewBar::Impl::onItemAdded(Item* item)
{
    auto imageable = dynamic_cast<ImageableItem*>(item);

    if(imageable && imageCombo->findData(QVariant::fromValue(imageable))==-1){

        ConnectionSet& connections = imageableItemConnections[imageable];
        connections.add(
            item->sigDisconnectedFromRoot().connect(
                [&, item](){ onItemDisconnectedFromRoot(item); }));
        connections.add(
            item->sigNameChanged().connect(
                [&, item](const string& /* oldName */) { onItemNameChange(item); }));

        imageCombo->addItem(item->displayName().c_str(), QVariant::fromValue(imageable));
    }
}


void ImageViewBar::Impl::onItemDisconnectedFromRoot(Item* item)
{
    if(auto imageable = dynamic_cast<ImageableItem*>(item)){
        int removeIndex = imageCombo->findData(QVariant::fromValue(imageable));
        imageCombo->removeItem(removeIndex);
        ConnectionSet& connections = imageableItemConnections[imageable];
        connections.disconnect();
        imageableItemConnections.erase(imageable);
    }
}


void ImageViewBar::Impl::onItemNameChange(Item* item)
{
    int index = imageCombo->findData(QVariant::fromValue(dynamic_cast<ImageableItem*>(item)));
    imageCombo->setItemText(index, QString(item->displayName().c_str()));
}


void ImageViewBar::Impl::onImageComboCurrentIndexChanged(int index)
{
    if(index == -1){
        return;
    }
    if(targetView){
        auto item = getImageableItem(index);
        targetView->setImageableItem(item);
    }
}


ImageableItem* ImageViewBar::getSelectedImageableItem()
{
    return impl->getSelectedImageableItem();
}


ImageableItem* ImageViewBar::Impl::getSelectedImageableItem()
{
    int index = imageCombo->currentIndex();
    return getImageableItem(index);
}


ImageableItem* ImageViewBar::Impl::getImageableItem(int index)
{
    const QVariant qv = imageCombo->itemData(index);
    if(qv.isValid()){
        return qv.value<ImageableItem*>();
    }else{
        return nullptr;
    }
}


void ImageViewBar::Impl::onFocusViewChanged(ImageView* imageView)
{
    targetView = imageView;
    
    if(targetView){
        if(auto imageable = targetView->getImageableItem()){
            int index = imageCombo->findData(QVariant::fromValue(imageable));
            imageCombo->setCurrentIndex(index);
        }else{
            imageCombo->setCurrentIndex(0);
        }
        if(targetView->isScalingEnabled()){
            adjustSizeToggle->setChecked(true);
        }else{
            adjustSizeToggle->setChecked(false);
        }
    }
}


void ImageViewBar::Impl::onViewDeactivated(ImageView* imageView)
{
    if(targetView == imageView){
        targetView = nullptr;
    }
}


void ImageViewBar::Impl::onAdjustSizeClicked(bool on)
{
    if(targetView){
        targetView->setScalingEnabled(on);
    }
}
