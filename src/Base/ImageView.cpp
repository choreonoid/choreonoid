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

class ImageViewImpl
{
public:
    ImageView* self;
    ImageWidget* imageWidget;
    Connection sigUpdatedConnection;
    ImageableItem* imageable;

    ImageViewImpl(ImageView* self);
    ~ImageViewImpl();

    void updateImage();
    void setImageableItem(ImageableItem* imageable);

    bool storeState(Archive& archive);
    void restoreState(const Archive& archive);
};

class ImageViewBarImpl
{
public:
    ImageViewBar* self;
    ComboBox* imageCombo;
    Connection sigItemAddedConnection;
    Connection sigViewActivatedConnection;
    typedef map<ImageableItem*, ConnectionSet> ImageableItemConnectionsMap;
    ImageableItemConnectionsMap imageableItemConnections;
    typedef map<ImageView*, ConnectionSet> ViewConnectionsMap;
    ViewConnectionsMap viewConnections;
    ImageView* targetView;

    ImageViewBarImpl(ImageViewBar* self);
    ~ImageViewBarImpl();
    ToolButton* adjustSizeToggle;
    void onItemAdded(Item* item);
    void onImageComboCurrentIndexChanged(int index);
    void onItemDisconnectedFromRoot(Item* item);
    void onItemNameChange(Item* item);
    void onViewActivated(View* view);
    void onForcusViewChanged(View* view);
    void onViewDeactivated(ImageView* imageView);
    void onAdjustSizeClicked(bool on);
    ImageableItem* getSelectedImageableItem();
    ImageableItem* getImageableItem(int index);
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

    imageable = nullptr;

    setImageableItem(imageViewBar->getSelectedImageableItem());
}


ImageViewImpl::~ImageViewImpl()
{
    instances.erase(std::find(instances.begin(), instances.end(), self));

    sigUpdatedConnection.disconnect();
}


void ImageViewImpl::updateImage()
{
    bool updated = false;
    if(imageable){
        if(auto image = imageable->getImage()){
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


ImageableItem* ImageView::getImageableItem()
{
    return impl->imageable;
}


void ImageView::setImageableItem(ImageableItem* imageable)
{
    impl->setImageableItem(imageable);
}


void ImageViewImpl::setImageableItem(ImageableItem* imageable)
{
    if(imageable != this->imageable){
        this->imageable = imageable;
        imageWidget->clear();
        sigUpdatedConnection.disconnect();
        sigUpdatedConnection = imageable->sigImageUpdated().connect([&](){ updateImage(); });
        updateImage();
    }
}


bool ImageView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool ImageViewImpl::storeState(Archive& archive)
{
    if(auto item = dynamic_cast<Item*>(imageable)){
        archive.writeItemId("ImageableItem", item);
    }
    return true;
}


bool ImageView::restoreState(const Archive& archive)
{
    archive.addPostProcess([&](){ impl->restoreState(archive); });
    return true;
}


void ImageViewImpl::restoreState(const Archive& archive)
{
    setImageableItem(
        dynamic_cast<ImageableItem*>(archive.findItem<Item>("ImageableItem")));
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
    impl = new ImageViewBarImpl(this);
}


ImageViewBar::~ImageViewBar()
{
    delete impl;
}


ImageViewBarImpl::ImageViewBarImpl(ImageViewBar* self)
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
    auto imageable = dynamic_cast<ImageableItem*>(item);

    if(imageable && imageCombo->findData(QVariant::fromValue(imageable))==-1){

        ConnectionSet& connections = imageableItemConnections[imageable];
        connections.add(
            item->sigDisconnectedFromRoot().connect(
                [&, item](){ onItemDisconnectedFromRoot(item); }));
        connections.add(
            item->sigNameChanged().connect(
                [&, item](const string& /* oldName */) { onItemNameChange(item); }));

        imageCombo->addItem(item->name().c_str(), QVariant::fromValue(imageable));
    }
}


void ImageViewBarImpl::onItemDisconnectedFromRoot(Item* item)
{
    if(auto imageable = dynamic_cast<ImageableItem*>(item)){
        int removeIndex = imageCombo->findData(QVariant::fromValue(imageable));
        imageCombo->removeItem(removeIndex);
        ConnectionSet& connections = imageableItemConnections[imageable];
        connections.disconnect();
        imageableItemConnections.erase(imageable);
    }
}


void ImageViewBarImpl::onItemNameChange(Item* item)
{
    int index = imageCombo->findData(QVariant::fromValue(dynamic_cast<ImageableItem*>(item)));
    imageCombo->setItemText(index, QString(item->name().c_str()));
}


void ImageViewBarImpl::onImageComboCurrentIndexChanged(int index)
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


ImageableItem* ImageViewBarImpl::getSelectedImageableItem()
{
    int index = imageCombo->currentIndex();
    return getImageableItem(index);
}


ImageableItem* ImageViewBarImpl::getImageableItem(int index)
{
    const QVariant qv = imageCombo->itemData(index);
    if(qv.isValid()){
        return qv.value<ImageableItem*>();
    }else{
        return nullptr;
    }
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
    auto imageView = dynamic_cast<ImageView*>(view);
    if(imageView){
        targetView = imageView;
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


void ImageViewBarImpl::onViewDeactivated(ImageView* imageView)
{
    if(targetView == imageView){
        targetView = nullptr;
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
