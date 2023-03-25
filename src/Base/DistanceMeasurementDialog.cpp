#include "DistanceMeasurementDialog.h"
#include "Item.h"
#include "RootItem.h"
#include "RenderableItem.h"
#include "GeometryMeasurementTracker.h"
#include "SceneView.h"
#include "SceneWidget.h"
#include "ScenePointSelectionMode.h"
#include "DisplayValueFormat.h"
#include "Buttons.h"
#include "ButtonGroup.h"
#include "ComboBox.h"
#include "CheckBox.h"
#include "SpinBox.h"
#include "Separator.h"
#include "LazyCaller.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneNodeClassRegistry>
#include <cnoid/SceneRenderer>
#include <cnoid/MathUtil>
#include <cnoid/CollisionDetector>
#include <cnoid/ThreadPool>
#include <cnoid/IdPair>
#include <cnoid/stdx/optional>
#include <QLabel>
#include <QBoxLayout>
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QColorDialog>
#include <vector>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class ItemInfo : public Referenced
{
public:
    Item* item;
    shared_ptr<ScopedConnection> itemConnection;
    GeometryMeasurementTrackerPtr tracker;
    ScopedConnection trackerConnection;
    vector<stdx::optional<CollisionDetector::GeometryHandle>> geometryHandles;
    bool isShortestDistanceFixedPoint;

    GeometryMeasurementTracker* getOrCreateTracker(){
        if(!tracker){
            tracker = GeometryMeasurementTracker::createTracker(item);
        }
        return tracker;
    }
};

typedef ref_ptr<ItemInfo> ItemInfoPtr;

class ViewportText : public SgViewportOverlay
{
public:
    ViewportText(DisplayValueFormat* displayValueFormat);
    void setMeasurementData(const Vector3& p1, const Vector3& p2, double distance);
    void setColor(const Vector3f& color);
    void render(SceneRenderer* renderer);
    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume) override;

    SgPosTransformPtr distanceTextTransform;
    SgTextPtr distanceText;
    DisplayValueFormat* displayValueFormat;
    string distanceDisplayFormat;
    double textHeight;
    Vector3 p1;
    Vector3 p2;
};

typedef ref_ptr<ViewportText> ViewportTextPtr;

class ObjectPointPickMode : public ScenePointSelectionMode
{
public:
    DistanceMeasurementDialog::Impl* dialogImpl;
    weak_ref_ptr<Item> pickableItemRef;
    
    ObjectPointPickMode(DistanceMeasurementDialog::Impl* dialogImpl);
    ~ObjectPointPickMode();
    void limitPickableItem(Item* item);
    void unlimitPickableItems();
    virtual std::vector<SgNode*> getTargetSceneNodes(SceneWidget* sceneWidget) override;
    void addTargetItem(std::vector<SgNode*>& targets, Item* item);
};

}

namespace cnoid {

class DistanceMeasurementDialog::Impl
{
public:
    DistanceMeasurementDialog* self;

    std::vector<ItemInfoPtr> candidateItemInfoLists[2];
    ItemInfoPtr targetItemInfos[2];
    bool isMeasurementActive;
    ObjectPointPickMode* objectPointPickMode;
    int whichObjectToPickPoint;
    bool isEditModeOriginally;
    CollisionDetectorPtr collisionDetector;
    CollisionDetectorDistanceAPI* collisionDetectorDistanceAPI;
    typedef CollisionDetector::GeometryHandle GeometryHandle;
    vector<std::pair<GeometryHandle, GeometryHandle>> handlePairs;
    unique_ptr<ThreadPool> threadPool;
    LazyCaller updateDistanceLater;

    DisplayValueFormat* displayValueFormat;
    string distanceDisplayFormat;
    string pointDisplayFormat;
    
    ComboBox itemCombos[2];
    ComboBox subEntryCombos[2];
    QLabel pointLabels[2];
    ToggleButton pointPickButtons[2];
    ButtonGroup pickRadioGroup[2];
    RadioButton surfaceRadios[2];
    RadioButton vertexRadios[2];
    CheckBox circleCenterCheck[2];
    CheckBox shortestDistanceCheck;
    CheckBox shortestDistanceFixOneSideCheck;
    ButtonGroup fixOneSideRadioGroup;
    RadioButton fixOneSideRadios[2];
    QLabel distanceLabel;
    QLabel distanceElementLabels[3];
    CheckBox distanceMarkerCheck;
    SpinBox distanceLineWidthSpin;
    PushButton distanceMarkerColorButton;
    QColorDialog* colorDialog;
    CheckBox distanceLineOverlayCheck;

    SceneView* sceneView;
    SceneWidget* sceneWidget;
    SgGroupPtr distanceMarker;
    SgOverlayPtr distanceLineOverlay;
    SgLineSetPtr distanceLine;
    ViewportTextPtr viewportDistanceText;
    SgUpdate sgUpdate;

    Impl(DistanceMeasurementDialog* self);
    ~Impl();
    void clearCandidateItems();
    void updateItemCandidates(Item* topItem);
    void updateItemCombos();
    void onItemDisconnectedFromRoot(Item* item);
    void setTargetItem(int which, int itemIndex, bool doUpdateItemComb);
    void setTargetSubEntry(int which, int entryIndex, bool doUpdateCombo);
    void onObjectPointPickSubModeRadioToggled(int which, int subMode);
    void onCircleCenterCheckToggled(int which);
    void activateObjectPointPickMode(int which, bool on);
    void onObjectPointPicked(const std::vector<ScenePointSelectionMode::PointInfoPtr>& additionalPoints);
    int findPickedItemIndex(ScenePointSelectionMode::PointInfo* pointInfo);
    bool calcCircleCenter(const Vector3& p1, const Vector3& p2, const Vector3& p3, Vector3& out_circleCenter);
    bool startMeasurement();
    void stopMeasurement();
    void onGeometryChanged(int which);
    void onShortestDistanceFixOneSideConfigurationChanged();
    void initializeCollisionDetector();
    void updateCollisionDetectionPositions(int which);
    void updateShortestDistance();
    void updateDistance();
    void updatePointDisplay(int which, Vector3 p);
    void updateDistanceDisplay(double distance, const Vector3& p1, const Vector3& p2);
    void invalidateDistanceDisplay();
    void showDistanceMarker(bool on);
    void setDistanceLineOverlayEnabled(bool on);
    void setDistanceMarkerColor(const QColor& color);
    void setDistanceMarkerColor(const Vector3f& color);
    void inputDistanceMarkerColorWithColorDialog();
};

}

struct ViewportTextRegistration {
    ViewportTextRegistration(){
        SceneNodeClassRegistry::instance().registerClass<ViewportText, SgViewportOverlay>();
        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                renderer->renderingFunctions()->setFunction<ViewportText>(
                    [renderer](SgNode* node){
                        static_cast<ViewportText*>(node)->render(renderer);
                    });
            });
    }
};


DistanceMeasurementDialog* DistanceMeasurementDialog::instance()
{
    static ViewportTextRegistration registration;
    static DistanceMeasurementDialog* dialog = new DistanceMeasurementDialog;
    return dialog;
}


DistanceMeasurementDialog::DistanceMeasurementDialog()
{
    impl = new Impl(this);
}


DistanceMeasurementDialog::Impl::Impl(DistanceMeasurementDialog* self_)
    : self(self_),
      updateDistanceLater([this](){ updateDistance(); }, LazyCaller::LowPriority)
{
    self->setWindowTitle(_("Distance Measurement"));

    isMeasurementActive = false;

    objectPointPickMode = new ObjectPointPickMode(this);
    objectPointPickMode->sigPointSelectionAdded().connect(
        [this](const std::vector<ScenePointSelectionMode::PointInfoPtr>& additionalPoints){
            onObjectPointPicked(additionalPoints);
        });

    int collisionDetectorIndex = CollisionDetector::factoryIndex("AISTCollisionDetector");
    collisionDetector = CollisionDetector::create(collisionDetectorIndex);
    collisionDetectorDistanceAPI = dynamic_cast<CollisionDetectorDistanceAPI*>(collisionDetector.get());
    if(!collisionDetectorDistanceAPI){
        collisionDetector.reset();
    }

    displayValueFormat = DisplayValueFormat::instance();
    distanceDisplayFormat = format("{{0:.{0}f}}", displayValueFormat->lengthDecimals());
    pointDisplayFormat = format("( {{0:.{0}f}}, {{1:.{0}f}}, {{2:.{0}f}} )", displayValueFormat->lengthDecimals());

    auto vbox = new QVBoxLayout;
    self->setLayout(vbox);

    for(int i=0; i < 2; ++i){
        auto objectLabel = new QLabel(QString(_("Object %1")).arg(i + 1));
        objectLabel->setAlignment(Qt::AlignCenter);
        vbox->addWidget(objectLabel);
        
        auto hbox = new QHBoxLayout;
        hbox->addWidget(&itemCombos[i], 1);
        hbox->addWidget(new QLabel("-"));
        hbox->addWidget(&subEntryCombos[i], 1);
        vbox->addLayout(hbox);

        hbox = new QHBoxLayout;
        hbox->addWidget(new QLabel(_("Position:")));
        hbox->addWidget(&pointLabels[i]);
        hbox->addStretch();
        vbox->addLayout(hbox);

        hbox = new QHBoxLayout;
        pointPickButtons[i].setText(_("Pick"));
        hbox->addWidget(&pointPickButtons[i]);
        
        auto& radioGroup = pickRadioGroup[i];
        auto radio = &surfaceRadios[i];
        radio->setText(_("Surface"));
        radioGroup.addButton(radio, ScenePointSelectionMode::SurfacePointMode);
        radio->setChecked(true);
        hbox->addWidget(radio);
        radio = &vertexRadios[i];
        radio->setText(_("Vertex"));
        radioGroup.addButton(radio, ScenePointSelectionMode::MeshVertexMode);
        hbox->addWidget(radio);

        hbox->addWidget(new QLabel("("));
        circleCenterCheck[i].setText(_("Circle Center"));
        hbox->addWidget(&circleCenterCheck[i]);
        hbox->addWidget(new QLabel(")"));
        
        hbox->addStretch();
        vbox->addLayout(hbox);

        itemCombos[i].sigCurrentIndexChanged().connect(
            [this, i](int index){ setTargetItem(i, index, false); });
        subEntryCombos[i].sigActivated().connect(
            [this, i](int index){ setTargetSubEntry(i, index, false); });
        pointPickButtons[i].sigToggled().connect(
            [this, i](bool on){ activateObjectPointPickMode(i, on); });
        radioGroup.sigButtonToggled().connect(
            [this, i](int id, bool on){
                if(on){ onObjectPointPickSubModeRadioToggled(i, id); }
            });
        circleCenterCheck[i].sigToggled().connect(
            [this, i](bool){ onCircleCenterCheckToggled(i); });
    }

    auto hbox = new QHBoxLayout;
    shortestDistanceCheck.setText(_("Shortest Distance"));
    if(collisionDetector){
        shortestDistanceCheck.sigToggled().connect(
            [this](bool){
                if(isMeasurementActive){
                    startMeasurement();
                }
            });
    } else {
        shortestDistanceCheck.setEnabled(false);
    }
    hbox->addWidget(&shortestDistanceCheck);

    shortestDistanceFixOneSideCheck.setText(_("Fix one side"));
    shortestDistanceFixOneSideCheck.sigToggled().connect(
        [this](bool){ onShortestDistanceFixOneSideConfigurationChanged(); });
    hbox->addWidget(&shortestDistanceFixOneSideCheck);
    
    for(int i=0; i < 2; ++i){
        auto radio = &fixOneSideRadios[i];
        radio->setText(QString(_("Point %1")).arg(i + 1));
        fixOneSideRadioGroup.addButton(radio, i);
        hbox->addWidget(radio);
    }
    fixOneSideRadios[0].setChecked(true);
    fixOneSideRadioGroup.sigButtonToggled().connect(
        [this](int, bool on){
            if(on){ onShortestDistanceFixOneSideConfigurationChanged(); }
        });
                
    hbox->addStretch();
    vbox->addLayout(hbox);

    int vspace = self->layoutVerticalSpacing();
    vbox->addSpacing(vspace);
    vbox->addWidget(new HSeparator);
    vbox->addSpacing(vspace);
    
    auto grid = new QGridLayout;
    grid->setColumnStretch(2, 1);
    grid->setColumnStretch(3, 1);
    grid->setColumnStretch(4, 1);
    int row = 0;
    
    grid->addWidget(new QLabel(_("Distance")), row, 0, Qt::AlignCenter);
    grid->addWidget(new QLabel(":"), row, 1, Qt::AlignCenter);
    grid->addWidget(&distanceLabel, row, 2, Qt::AlignCenter);
    ++row;

    static const char* xyzLabels[] = { "X", "Y", "Z" };
    for(int i=0; i < 3; ++i){
        grid->addWidget(new QLabel(xyzLabels[i]), row, 0, Qt::AlignCenter);
        grid->addWidget(new QLabel(":"), row, 1, Qt::AlignCenter);
        grid->addWidget(&distanceElementLabels[i], row, 2, Qt::AlignCenter);
        ++row;
    }

    vbox->addLayout(grid);

    vbox->addSpacing(vspace);
    vbox->addWidget(new HSeparator);
    vbox->addSpacing(vspace);

    hbox = new QHBoxLayout;

    distanceMarkerCheck.setText(_("Marker"));
    distanceMarkerCheck.setChecked(true);
    distanceMarkerCheck.sigToggled().connect([this](bool on){ showDistanceMarker(on); });
    hbox->addWidget(&distanceMarkerCheck);

    distanceMarkerColorButton.setText(_("Color"));
    distanceMarkerColorButton.sigClicked().connect(
        [this](){ inputDistanceMarkerColorWithColorDialog(); });
    hbox->addWidget(&distanceMarkerColorButton);

    hbox->addWidget(new QLabel(_("Line width")));
    distanceLineWidthSpin.setRange(1, 10);
    distanceLineWidthSpin.setValue(2);
    distanceLineWidthSpin.sigValueChanged().connect(
        [this](int width){
            distanceLine->setLineWidth(width);
            distanceLine->notifyUpdate(sgUpdate.withAction(SgUpdate::AppearanceModified));
        });
    hbox->addWidget(&distanceLineWidthSpin);


    colorDialog = nullptr;

    distanceLineOverlayCheck.setText(_("Overlay"));
    distanceLineOverlayCheck.setChecked(true);
    distanceLineOverlayCheck.sigToggled().connect(
        [this](bool on){ setDistanceLineOverlayEnabled(on); });
    hbox->addWidget(&distanceLineOverlayCheck);
    hbox->addStretch();

    vbox->addLayout(hbox);

    vbox->addSpacing(vspace);
    vbox->addWidget(new HSeparator);
    vbox->addSpacing(vspace);

    auto buttonBox = new QDialogButtonBox(self);

    PushButton* measureButton = new PushButton(_("&Measure"));
    measureButton->setDefault(true);
    measureButton->sigClicked().connect([this](){ startMeasurement(); });
    buttonBox->addButton(measureButton, QDialogButtonBox::ActionRole);
    
    QPushButton* cancelButton = new QPushButton(_("&Cancel"));
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    connect(buttonBox, &QDialogButtonBox::rejected,
            [this](){
                stopMeasurement();
                self->reject();
            });

    vbox->addWidget(buttonBox);

    sceneView = SceneView::instance();
    sceneWidget = sceneView->sceneWidget();

    distanceLine = new SgLineSet;
    distanceLine->getOrCreateVertices(2);
    distanceLine->addLine(0, 1);
    distanceLine->setLineWidth(distanceLineWidthSpin.value());

    viewportDistanceText = new ViewportText(displayValueFormat);

    distanceMarker = new SgGroup;
    distanceMarker->setAttribute(SgObject::MetaScene);
    distanceMarker->addChild(viewportDistanceText);
    distanceLineOverlay = new SgOverlay;
    distanceLineOverlay->addChild(distanceLine);
    distanceMarker->addChild(distanceLineOverlay);

    setDistanceMarkerColor(Vector3f(1.0f, 0.0f, 0.0f)); // Red
}


DistanceMeasurementDialog::~DistanceMeasurementDialog()
{
    delete impl;
}


DistanceMeasurementDialog::Impl::~Impl()
{
    delete objectPointPickMode;
}


void DistanceMeasurementDialog::show()
{
    impl->updateItemCandidates(RootItem::instance());
    Dialog::show();
}


void DistanceMeasurementDialog::Impl::clearCandidateItems()
{
    for(int i=0; i < 2; ++i){
        candidateItemInfoLists[i].clear();
        auto& combo = itemCombos[i];
        combo.blockSignals(true);
        combo.clear();
        combo.blockSignals(false);
    }
}


void DistanceMeasurementDialog::Impl::updateItemCandidates(Item* topItem)
{
    clearCandidateItems();

    topItem->traverse(
        [this](Item* item) -> bool {
            if(GeometryMeasurementTracker::checkIfMeasureable(item)){
                auto connection = make_shared<ScopedConnection>();
                *connection = item->sigDisconnectedFromRoot().connect(
                            [this, item](){ onItemDisconnectedFromRoot(item); });
                for(int i=0; i < 2; ++i){
                    ItemInfoPtr info = new ItemInfo;
                    info->item = item;
                    info->itemConnection =connection;
                    candidateItemInfoLists[i].push_back(info);
                }
            }
            return true;
        });

    updateItemCombos();
}


void DistanceMeasurementDialog::Impl::updateItemCombos()
{
    for(int i=0; i < 2; ++i){
        auto& combo = itemCombos[i];
        combo.blockSignals(true);
        Item* currentItem = nullptr;
        if(targetItemInfos[i]){
            currentItem = targetItemInfos[i]->item;
        }
        combo.clear();
        for(auto& info : candidateItemInfoLists[i]){
            combo.addItem(info->item->displayName().c_str());
            if(info->item == currentItem){
                combo.setCurrentIndex(combo.count() - 1);
            } else if(info->tracker){
                info->tracker.reset();
            }
        }
        combo.blockSignals(false);
    }

    for(int i=0; i < 2; ++i){
        auto& info = targetItemInfos[i];
        if(!info && !candidateItemInfoLists[i].empty()){
            setTargetItem(i, itemCombos[i].currentIndex(), false);
        }
    }
}


void DistanceMeasurementDialog::Impl::onItemDisconnectedFromRoot(Item* item)
{
    for(int i=0; i < 2; ++i){
        auto& infos = candidateItemInfoLists[i];
        auto it = infos.begin();
        while(it != infos.end()){
            if((*it)->item == item){
                it = infos.erase(it);
            } else {
                ++it;
            }
        }
    }

    bool isTargetItemRemoved = false;
    for(int i=0; i < 2; ++i){
        if(auto info = targetItemInfos[i]){
            if(info->item == item){
                targetItemInfos[i].reset();
                isTargetItemRemoved = true;
            }
        }
    }

    if(isTargetItemRemoved){
        stopMeasurement();
    }
        
    updateItemCombos();
}


void DistanceMeasurementDialog::Impl::setTargetItem(int which, int itemIndex, bool doUpdateItemComb)
{
    auto info = candidateItemInfoLists[which][itemIndex];
    targetItemInfos[which] = info;

    if(doUpdateItemComb){
        auto& itemCombo = itemCombos[which];
        if(itemIndex < itemCombo.count()){
            itemCombo.blockSignals(true);
            itemCombo.setCurrentIndex(itemIndex);
            itemCombo.blockSignals(false);
        }
    }

    if(auto tracker = info->getOrCreateTracker()){
        auto& subCombo = subEntryCombos[which];
        subCombo.blockSignals(true);
        subCombo.clear();
        int n = tracker->getNumSubEntries();
        int current = tracker->getCurrentSubEntryIndex();
        for(int i=0; i < n; ++i){
            subCombo.addItem(tracker->getSubEntryName(i).c_str());
            if(i == current){
                subCombo.setCurrentIndex(i);
            }
        }
        subCombo.blockSignals(false);
    }

    if(shortestDistanceFixOneSideCheck.isChecked() || !shortestDistanceCheck.isChecked()){
        if(auto tracker = info->tracker){
            updatePointDisplay(which, tracker->getMeasurementPoint());
        }
    }
    if(isMeasurementActive){
        startMeasurement();
    }
}


void DistanceMeasurementDialog::Impl::setTargetSubEntry(int which, int entryIndex, bool doUpdateCombo)
{
    auto info = targetItemInfos[which];

    bool updated = false;
    if(auto tracker = info->getOrCreateTracker()){
        updated = tracker->setCurrentSubEntry(entryIndex);
    }
    
    if(updated){
        if(doUpdateCombo){
            auto& combo = subEntryCombos[which];
            if(entryIndex < combo.count()){
                combo.blockSignals(true);
                combo.setCurrentIndex(entryIndex);
                combo.blockSignals(false);
            }
        }
    }

    if(isMeasurementActive){
        startMeasurement();
    }
}


void DistanceMeasurementDialog::Impl::onObjectPointPickSubModeRadioToggled(int which, int subMode)
{
    if(sceneView->customMode() == objectPointPickMode->customModeId()){
        if(which == whichObjectToPickPoint){
            objectPointPickMode->clearSelection();
            objectPointPickMode->setSubMode(subMode);
        }
    }
}


void DistanceMeasurementDialog::Impl::onCircleCenterCheckToggled(int which)
{
    if(sceneView->customMode() == objectPointPickMode->customModeId()){
        if(which == whichObjectToPickPoint){
            objectPointPickMode->clearSelection();
        }
    }
}


void DistanceMeasurementDialog::Impl::activateObjectPointPickMode(int which, bool on)
{
    if(on){
        whichObjectToPickPoint = which;
        objectPointPickMode->setSubMode(pickRadioGroup[which].checkedId());
        if(sceneView->customMode() != objectPointPickMode->customModeId()){
            sceneView->setCustomMode(objectPointPickMode->customModeId());
        }
        isEditModeOriginally = sceneWidget->isEditMode();
        sceneWidget->setEditMode(true);
        pointPickButtons[1 - which].setEnabled(false);

    } else {
        if(sceneView->customMode() == objectPointPickMode->customModeId()){
            sceneView->setCustomMode(0);
            sceneWidget->setEditMode(isEditModeOriginally);
        }
        objectPointPickMode->clearSelection();
        objectPointPickMode->unlimitPickableItems();
        
        auto& button = pointPickButtons[which];
        button.blockSignals(true);
        button.setChecked(false);
        button.blockSignals(false);
        pointPickButtons[1 - which].setEnabled(true);
    }
}


void DistanceMeasurementDialog::Impl::onObjectPointPicked
(const std::vector<ScenePointSelectionMode::PointInfoPtr>& additionalPoints)
{
    auto& points = objectPointPickMode->selectedPoints();
    bool doStopPicking = true;

    if(!points.empty()){
        auto pointInfo = points.back();
        auto pickedItemIndex = findPickedItemIndex(pointInfo);
        auto itemInfo = candidateItemInfoLists[whichObjectToPickPoint][pickedItemIndex];
        if(pickedItemIndex >= 0){
            Vector3 measurementPoint;
            bool doSetMeasurementPoint = false;
            if(!circleCenterCheck[whichObjectToPickPoint].isChecked()){
                measurementPoint = pointInfo->position();
                doSetMeasurementPoint = true;
            } else {
                if(points.size() <= 2){
                    if(points.size() == 1){
                        objectPointPickMode->limitPickableItem(itemInfo->item);
                    }
                    doStopPicking = false;
                } else {
                    Vector3 p1 = points[0]->position();
                    Vector3 p2 = points[1]->position();
                    Vector3 p3 = points[2]->position();
                    if(calcCircleCenter(p1, p2, p3, measurementPoint)){
                        doSetMeasurementPoint = true;
                    }
                }
            }
            if(doSetMeasurementPoint){
                if(itemInfo->tracker->setMeasurementPoint(pointInfo->path(), measurementPoint)){
                    setTargetItem(whichObjectToPickPoint, pickedItemIndex, true);
                }
            }
        }
    }
    
    if(doStopPicking){
        activateObjectPointPickMode(whichObjectToPickPoint, false);
    }
}


int DistanceMeasurementDialog::Impl::findPickedItemIndex(ScenePointSelectionMode::PointInfo* pointInfo)
{
    int itemIndex = -1;
    auto& itemInfos = candidateItemInfoLists[whichObjectToPickPoint];
    for(size_t i=0; i < itemInfos.size(); ++i){
        auto itemInfo = itemInfos[i];
        if(auto tracker = itemInfo->getOrCreateTracker()){
            auto p = pointInfo->position().cast<Vector3::Scalar>();
            if(tracker->checkTrackable(pointInfo->path())){
                itemIndex = i;
                break;
            }
        }
    }
    return itemIndex;
}


bool DistanceMeasurementDialog::Impl::calcCircleCenter
(const Vector3& p1, const Vector3& p2, const Vector3& p3, Vector3& out_circleCenter)
{
    // triangle edges
    const Vector3 t = p2 - p1;
    const Vector3 u = p3 - p1;
    const Vector3 v = p3 - p2;

    // triangle normal
    const Vector3 w = t.cross(u);
    const double wsl = w.squaredNorm();
    if(wsl < 10e-14){
        // area of the triangle is too small
        return false;
    }

    const double iwsl2 = 1.0 / (2.0 * wsl);
    const double tt = t.dot(t);
    const double uu = u.dot(u);

    out_circleCenter = p1 + (u * tt * u.dot(v) - t * uu * t.dot(v)) * iwsl2;

    return true;
}


bool DistanceMeasurementDialog::Impl::startMeasurement()
{
    if(isMeasurementActive){
        stopMeasurement();
    }

    int numValidItems = 0;
    for(int i=0; i < 2; ++i){
        auto& info = targetItemInfos[i];
        if(info && info->item){
            if(auto tracker = info->getOrCreateTracker()){
                ++numValidItems;
            }
        }
    }

    if(numValidItems >= 2){
        isMeasurementActive = true;
        if(shortestDistanceCheck.isChecked()){
            initializeCollisionDetector();
        }
        for(int i=0; i < 2; ++i){
            auto info = targetItemInfos[i];
            info->trackerConnection =
                info->tracker->sigGeometryChanged().connect(
                    [this, i](){ onGeometryChanged(i); });
        }
        updateDistance();
        if(distanceMarkerCheck.isChecked()){
            showDistanceMarker(true);
        }
    }
    
    return isMeasurementActive;
}


void DistanceMeasurementDialog::Impl::stopMeasurement()
{
    if(isMeasurementActive){
        if(distanceMarkerCheck.isChecked()){
            showDistanceMarker(false);
        }
        for(int i=0; i < 2; ++i){
            auto info = targetItemInfos[i];
            info->trackerConnection.disconnect();
            info->geometryHandles.clear();
        }
        if(collisionDetector){
            collisionDetector->clearGeometries();
        }
        isMeasurementActive = false;
    }
}


void DistanceMeasurementDialog::Impl::onGeometryChanged(int which)
{
    if(shortestDistanceCheck.isChecked()){
        updateCollisionDetectionPositions(which);
    }
    updateDistanceLater();
}


void DistanceMeasurementDialog::Impl::onShortestDistanceFixOneSideConfigurationChanged()
{
    if(isMeasurementActive && shortestDistanceCheck.isChecked()){
        startMeasurement();
    }
}


void DistanceMeasurementDialog::Impl::initializeCollisionDetector()
{
    for(int i=0; i < 2; ++i){
        auto& info = targetItemInfos[i];
        info->geometryHandles.clear();
        auto tracker = info->tracker;
        if(shortestDistanceFixOneSideCheck.isChecked() && fixOneSideRadioGroup.checkedId() == i){
            auto shape = new SgShape;
            auto mesh = shape->getOrCreateMesh();
            mesh->getOrCreateVertices()->push_back(Vector3f::Zero());
            mesh->addTriangle(0, 0, 0);
            auto handle = collisionDetector->addGeometry(shape);
            info->geometryHandles.push_back(handle);
            info->isShortestDistanceFixedPoint = true;
        } else {
            int n = tracker->getNumShapes();
            for(int i=0; i < n; ++i){
                auto handle = collisionDetector->addGeometry(tracker->getShape(i));
                info->geometryHandles.push_back(handle);
            }
            info->isShortestDistanceFixedPoint = false;
        }
        updateCollisionDetectionPositions(i);
    }

    handlePairs.clear();
    for(auto& handle1 : targetItemInfos[0]->geometryHandles){
        if(handle1){
            for(auto& handle2 : targetItemInfos[1]->geometryHandles){
                if(handle2){
                    handlePairs.emplace_back(*handle1, *handle2);
                }
            }
        }
    }
    int numThreads = std::min(static_cast<unsigned int>(handlePairs.size()), thread::hardware_concurrency());
    threadPool = make_unique<ThreadPool>(numThreads);
    
    collisionDetector->makeReady();
}


void DistanceMeasurementDialog::Impl::updateCollisionDetectionPositions(int which)
{
    auto info = targetItemInfos[which];
    auto tracker = info->tracker;
    if(info->isShortestDistanceFixedPoint){
        Isometry3 T = Isometry3::Identity();
        T.translation() = tracker->getMeasurementPoint();
        collisionDetector->updatePosition(*info->geometryHandles.front(), T);
    } else {
        int n = tracker->getNumShapes();
        for(int i=0; i < n; ++i){
            if(auto handle = info->geometryHandles[i]){
                collisionDetector->updatePosition(*handle, tracker->getShapePosition(i));
            }
        }
    }
}


/**
   \todo Make the main process to find the shortest distance a background process
   to improve the response of dragging a target object.
*/
void DistanceMeasurementDialog::Impl::updateShortestDistance()
{
    double shortestDistance = std::numeric_limits<double>::max();
    Vector3 p1s, p2s;
    std::mutex distanceMutex;
    bool detected = false;
    
    for(auto& handlePair : handlePairs){
        threadPool->start([&](){
            Vector3 p1, p2;
            auto distance = collisionDetectorDistanceAPI->detectDistance(
                handlePair.first, handlePair.second, p1, p2);
            {
                std::lock_guard<std::mutex> guard(distanceMutex);
                if(distance < shortestDistance){
                    shortestDistance = distance;
                    p1s = p1;
                    p2s = p2;
                    detected = true;
                }
            }
        });
    }
    threadPool->wait();

    if(detected){
        updateDistanceDisplay(shortestDistance, p1s, p2s);
    } else {
        invalidateDistanceDisplay();
    }
}


void DistanceMeasurementDialog::Impl::updateDistance()
{
    if(!isMeasurementActive){
        return;
    }

    if(shortestDistanceCheck.isChecked()){
        updateShortestDistance();
    } else {
        Vector3 p[2];
        for(int i=0; i < 2; ++i){
            p[i] = targetItemInfos[i]->tracker->getMeasurementPoint();
        }
        double distance = Vector3(p[1] - p[0]).norm();
        updateDistanceDisplay(distance, p[0], p[1]);
    }
}


void DistanceMeasurementDialog::Impl::updatePointDisplay(int which, Vector3 p)
{
    p *= displayValueFormat->ratioToDisplayLength();
    pointLabels[which].setText(format(pointDisplayFormat, p.x(), p.y(), p.z()).c_str());
}
    

void DistanceMeasurementDialog::Impl::updateDistanceDisplay(double distance, const Vector3& p1, const Vector3& p2)
{
    if(!isMeasurementActive){
        return;
    }

    auto& vertices = *distanceLine->vertices();
    for(int i=0; i < 2; ++i){
        const Vector3& p = (i == 0) ? p1 : p2;
        vertices[i] = p.cast<Vector3f::Scalar>();
        updatePointDisplay(i, p);
    }
    distanceLabel.setText(format(distanceDisplayFormat, displayValueFormat->toDisplayLength(distance)).c_str());

    Vector3 dp = displayValueFormat->ratioToDisplayLength() * (p2 - p1);
    for(int i=0; i < 3; ++i){
        distanceElementLabels[i].setText(format(distanceDisplayFormat, dp(i)).c_str());
    }

    viewportDistanceText->setMeasurementData(p1, p2, distance);
    if(distanceMarkerCheck.isChecked()){
        sgUpdate.setAction(SgUpdate::GeometryModified);
        vertices.notifyUpdate(sgUpdate);
    }
}


void DistanceMeasurementDialog::Impl::invalidateDistanceDisplay()
{
    for(int i=0; i < 2; ++i){
        pointLabels[i].setText("---");
    }
    distanceLabel.setText("---");
    for(int i=0; i < 3; ++i){
        distanceElementLabels[i].setText("---");
    }
    showDistanceMarker(false);
}


void DistanceMeasurementDialog::Impl::showDistanceMarker(bool on)
{
    auto systemNodeGroup = sceneWidget->systemNodeGroup();
    if(on){
        systemNodeGroup->addChildOnce(distanceMarker, sgUpdate);
    } else {
        systemNodeGroup->removeChild(distanceMarker, sgUpdate);
    }
}


void DistanceMeasurementDialog::Impl::inputDistanceMarkerColorWithColorDialog()
{
    if(!colorDialog){
        colorDialog = new QColorDialog(self);
    }

    colorDialog->setOption(QColorDialog::DontUseNativeDialog);
    colorDialog->setWindowTitle(_("Distance Marker Color"));

    auto material = distanceLine->material();
    Vector3f c = material->diffuseColor();
    QColor color;
    color.setRgbF(c[0], c[1], c[2], 1.0f);
    colorDialog->setCurrentColor(color);

    QObject::connect(
        colorDialog, &QColorDialog::currentColorChanged,
        [this](const QColor &color){ setDistanceMarkerColor(color); });

    if(colorDialog->exec() == QDialog::Accepted){
        setDistanceMarkerColor(colorDialog->selectedColor());
    } else {
        setDistanceMarkerColor(color);
    }
}


void DistanceMeasurementDialog::Impl::setDistanceLineOverlayEnabled(bool on)
{
    if(on){
        distanceMarker->removeChild(distanceLine);
        distanceMarker->addChildOnce(distanceLineOverlay, sgUpdate);
    } else {
        distanceMarker->removeChild(distanceLineOverlay);
        distanceMarker->addChildOnce(distanceLine, sgUpdate);
    }
}


void DistanceMeasurementDialog::Impl::setDistanceMarkerColor(const QColor& color)
{
    setDistanceMarkerColor(Vector3f(color.redF(), color.greenF(), color.blueF()));
}


void DistanceMeasurementDialog::Impl::setDistanceMarkerColor(const Vector3f& color)
{
    viewportDistanceText->setColor(color);

    auto material = distanceLine->getOrCreateMaterial();
    material->setDiffuseColor(color);
    sgUpdate.setAction(SgUpdate::AppearanceModified);
    material->notifyUpdate(sgUpdate);

    objectPointPickMode->setHighlightedPointColor(color);
    objectPointPickMode->setSelectedPointColor(color);
}


void DistanceMeasurementDialog::onFinished(int result)
{
    impl->stopMeasurement();
    impl->clearCandidateItems();

    for(int i=0; i < 2; ++i){
        impl->activateObjectPointPickMode(i, false);
    }
}


ViewportText::ViewportText(DisplayValueFormat* displayValueFormat)
    : SgViewportOverlay(findClassId<ViewportText>()),
      displayValueFormat(displayValueFormat)
{
    distanceDisplayFormat = format("{{0:.{0}f}}mm", displayValueFormat->lengthDecimals());
    
    textHeight = 20.0;
    distanceText = new SgText;
    distanceText->setTextHeight(textHeight);
    distanceTextTransform = new SgPosTransform;
    distanceTextTransform->addChild(distanceText);
    addChild(distanceTextTransform);
}


void ViewportText::setMeasurementData(const Vector3& p1, const Vector3& p2, double distance)
{
    this->p1 = p1;
    this->p2 = p2;
    double d = displayValueFormat->toDisplayLength(distance);
    distanceText->setText(format(distanceDisplayFormat, d).c_str());
    distanceText->notifyUpdate();
}


void ViewportText::setColor(const Vector3f& color)
{
    distanceText->setColor(color);
}


void ViewportText::render(SceneRenderer* renderer)
{
    Vector3 q1 = renderer->project(p1);
    Vector3 q2 = renderer->project(p2);
    Vector3 p = (q1 + q2) / 2.0;
    double x = q2.x() - q1.x();
    double y = q2.y() - q1.y();
    double theta = degree(atan2(y, x));
    if((theta > 0 && theta < 90.0) || theta < -90.0){
        p.y() -= textHeight;
    }
    distanceTextTransform->setTranslation(p);
    renderer->renderingFunctions()->dispatchAs<SgViewportOverlay>(this);    
}


void ViewportText::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    io_volume.left = 0;
    io_volume.right = viewportWidth;
    io_volume.bottom = 0;
    io_volume.top = viewportHeight;
    io_volume.zNear = 1.0;
    io_volume.zFar = -1.0;
}


ObjectPointPickMode::ObjectPointPickMode(DistanceMeasurementDialog::Impl* dialogImpl)
    : dialogImpl(dialogImpl)
{
    //! \todo Try to use the mode without the registration
    setCustomModeId(SceneView::registerCustomMode(this));

    setControlModifierEnabled(false);
}


ObjectPointPickMode::~ObjectPointPickMode()
{
    SceneView::unregisterCustomMode(customModeId());
}


void ObjectPointPickMode::limitPickableItem(Item* item)
{
    pickableItemRef = item;
    updateTargetSceneNodes();
}


void ObjectPointPickMode::unlimitPickableItems()
{
    pickableItemRef.reset();
    updateTargetSceneNodes();
}


std::vector<SgNode*> ObjectPointPickMode::getTargetSceneNodes(SceneWidget* /* sceneWidget */)
{
    std::vector<SgNode*> targets;
    if(pickableItemRef){
        if(auto pickableItem = pickableItemRef.lock()){
            addTargetItem(targets, pickableItem);
        }
    } else {
        for(auto& info : dialogImpl->candidateItemInfoLists[dialogImpl->whichObjectToPickPoint]){
            addTargetItem(targets, info->item);
        }
    }
    return targets;
}


void ObjectPointPickMode::addTargetItem(std::vector<SgNode*>& targets, Item* item)
{
    if(auto renderableItem = dynamic_cast<RenderableItem*>(item)){
        if(auto node = renderableItem->getScene()){
            targets.push_back(node);
        }
    }
}
