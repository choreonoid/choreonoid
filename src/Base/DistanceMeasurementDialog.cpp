#include "DistanceMeasurementDialog.h"
#include "DistanceMeasurementItem.h"
#include "Item.h"
#include "RootItem.h"
#include "GeometryMeasurementTracker.h"
#include "SceneView.h"
#include "SceneWidget.h"
#include "ScenePointSelectionMode.h"
#include "DisplayValueFormat.h"
#include "MessageView.h"
#include "Buttons.h"
#include "ButtonGroup.h"
#include "ComboBox.h"
#include "CheckBox.h"
#include "SpinBox.h"
#include "LineEdit.h"
#include "Separator.h"
#include <cnoid/ConnectionSet>
#include <QLabel>
#include <QBoxLayout>
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QColorDialog>
#include <fmt/format.h>
#include <vector>
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

    GeometryMeasurementTracker* getOrCreateTracker(){
        if(!tracker){
            tracker = GeometryMeasurementTracker::createTracker(item);
        }
        return tracker;
    }
};

typedef ref_ptr<ItemInfo> ItemInfoPtr;

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

    DistanceMeasurementItemPtr measurementItem;
    ScopedConnectionSet measurementItemConnections;
    DistanceMeasurementItemPtr builtinMeasurementItem;
    weak_ref_ptr<Item> defaultItemToAddNewMeasurementItem;
    ItemPtr itemToAddNewMeasurementItem;
    bool isExistingMeasurementItem;
    
    ObjectPointPickMode* objectPointPickMode;
    int whichObjectToPickPoint;
    bool isEditModeOriginally;

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
    CheckBox createMeasurementItemCheck;
    LineEdit measurementItemNameEdit;

    SceneView* sceneView;
    SceneWidget* sceneWidget;

    Impl(DistanceMeasurementDialog* self);
    ~Impl();
    void setMeasurementItem(DistanceMeasurementItem* item, bool isNewSession);
    void updateMeasurementConfigurationWidgets();
    void finalizeDistanceMeasurement();
    void onMeasurementItemDisconnectedFromRoot();
    void onCreateMeasurementItemCheckToggled(bool on);
    void onMeasurementItemNameEditingFinishied();
    void clearCandidateItems();
    void updateItemCandidates(Item* topItem);
    void onItemComboAboutToShowPopup(int which);
    void clearItemComboBox(int which);
    void updateItemCombo(int which);
    void onItemDisconnectedFromRoot(Item* item);
    void setTargetItem(int which, int itemIndex, bool doUpdateItemComb);
    void setTargetSubEntry(int which, int entryIndex, bool doUpdateCombo);
    void onObjectPointPickSubModeRadioToggled(int which, int subMode);
    void onCircleCenterCheckToggled(int which);
    void activateObjectPointPickMode(int which, bool on);
    void onObjectPointPicked(const std::vector<ScenePointSelectionMode::PointInfoPtr>& additionalPoints);
    int findPickedItemIndex(ScenePointSelectionMode::PointInfo* pointInfo);
    bool calcCircleCenter(const Vector3& p1, const Vector3& p2, const Vector3& p3, Vector3& out_circleCenter);
    void onMeasureButtonClicked();
    bool startMeasurement();
    void onShortestDistanceCheckToggled(bool on);
    void onShortestDistanceFixOneSideCheckToggled(bool on);
    void onFixOneSideRadioToggled(int which);
    void updatePointDisplay(int which, Vector3 p);
    void updateDistanceDisplay();
    void invalidateDistanceDisplay();
    void showDistanceMarker(bool on);
    void showDistanceMarkerOfBuiltinMeasurementItem(bool on);
    void onDistanceLineOverlayCheckToggled(bool on);
    void setDistanceMarkerColor(const QColor& color);
    void onDistanceLineWidthChanged(double width);
    void inputDistanceMarkerColorWithColorDialog();
};

}

DistanceMeasurementDialog* DistanceMeasurementDialog::instance()
{
    static DistanceMeasurementDialog* dialog = new DistanceMeasurementDialog;
    return dialog;
}


DistanceMeasurementDialog::DistanceMeasurementDialog()
{
    impl = new Impl(this);
}


DistanceMeasurementDialog::Impl::Impl(DistanceMeasurementDialog* self_)
    : self(self_)
{
    self->setWindowTitle(_("Distance Measurement"));

    defaultItemToAddNewMeasurementItem = RootItem::instance();
    isExistingMeasurementItem = false;

    objectPointPickMode = new ObjectPointPickMode(this);
    objectPointPickMode->sigPointSelectionAdded().connect(
        [this](const std::vector<ScenePointSelectionMode::PointInfoPtr>& additionalPoints){
            onObjectPointPicked(additionalPoints);
        });

    displayValueFormat = DisplayValueFormat::instance();
    distanceDisplayFormat = format("{{0:.{0}f}}", displayValueFormat->lengthDecimals());
    pointDisplayFormat = format("( {{0:.{0}f}}, {{1:.{0}f}}, {{2:.{0}f}} )", displayValueFormat->lengthDecimals());

    auto vbox = new QVBoxLayout;
    self->setLayout(vbox);

    QHBoxLayout* hbox;
    QGridLayout* grid;
    int row;

    grid = new QGridLayout;
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(2, 1);
    row = 0;
    for(int i=0; i < 2; ++i){
        auto objectLabel = new QLabel(QString(_("Object %1")).arg(i + 1));
        objectLabel->setAlignment(Qt::AlignCenter);
        grid->addWidget(objectLabel, row++, 0, 1, 3);

        auto& itemCombo = itemCombos[i];
        itemCombo.setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        itemCombo.sigAboutToShowPopup().connect(
            [this, i]{ onItemComboAboutToShowPopup(i); });
        grid->addWidget(&itemCombo, row, 0);
        grid->addWidget(new QLabel("-"), row, 1);
        subEntryCombos[i].setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        grid->addWidget(&subEntryCombos[i], row++, 2);

        hbox = new QHBoxLayout;
        hbox->addWidget(new QLabel(_("Position:")));
        hbox->addWidget(&pointLabels[i]);
        hbox->addStretch();
        grid->addLayout(hbox, row++, 0, 1, 3);

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
        grid->addLayout(hbox, row++, 0, 1, 3);

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

    vbox->addLayout(grid);

    hbox = new QHBoxLayout;
    shortestDistanceCheck.setText(_("Shortest Distance"));
    shortestDistanceCheck.sigToggled().connect(
        [this](bool on){ onShortestDistanceCheckToggled(on); });
    hbox->addWidget(&shortestDistanceCheck);

    shortestDistanceFixOneSideCheck.setText(_("Fix one side"));
    shortestDistanceFixOneSideCheck.sigToggled().connect(
        [this](bool on){ onShortestDistanceFixOneSideCheckToggled(on); });
    hbox->addWidget(&shortestDistanceFixOneSideCheck);
    
    for(int i=0; i < 2; ++i){
        auto radio = &fixOneSideRadios[i];
        radio->setText(QString(_("Point %1")).arg(i + 1));
        fixOneSideRadioGroup.addButton(radio, i);
        hbox->addWidget(radio);
    }
    fixOneSideRadios[0].setChecked(true);
    fixOneSideRadioGroup.sigButtonToggled().connect(
        [this](int id, bool on){
            if(on){ onFixOneSideRadioToggled(id); }
        });
                
    hbox->addStretch();
    vbox->addLayout(hbox);

    int vspace = self->layoutVerticalSpacing();
    vbox->addSpacing(vspace);
    vbox->addWidget(new HSeparator);
    vbox->addSpacing(vspace);
    
    grid = new QGridLayout;
    grid->setColumnStretch(2, 1);
    grid->setColumnStretch(3, 1);
    grid->setColumnStretch(4, 1);
    row = 0;
    
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
        [this](int width){ onDistanceLineWidthChanged(width); });
    hbox->addWidget(&distanceLineWidthSpin);

    colorDialog = nullptr;

    distanceLineOverlayCheck.setText(_("Overlay"));
    distanceLineOverlayCheck.setChecked(true);
    distanceLineOverlayCheck.sigToggled().connect(
        [this](bool on){ onDistanceLineOverlayCheckToggled(on); });
    hbox->addWidget(&distanceLineOverlayCheck);
    hbox->addStretch();

    vbox->addLayout(hbox);

    vbox->addSpacing(vspace);
    vbox->addWidget(new HSeparator);
    vbox->addSpacing(vspace);

    hbox = new QHBoxLayout;
    createMeasurementItemCheck.setText(_("Create measurement item"));
    createMeasurementItemCheck.sigToggled().connect(
        [this](bool on){ onCreateMeasurementItemCheckToggled(on); });
    hbox->addWidget(&createMeasurementItemCheck);
    hbox->addWidget(new QLabel(_("Name:")));
    measurementItemNameEdit.sigEditingFinished().connect(
        [this]{ onMeasurementItemNameEditingFinishied(); });
    hbox->addWidget(&measurementItemNameEdit);
    vbox->addLayout(hbox);
    
    vbox->addSpacing(vspace);
    vbox->addWidget(new HSeparator);
    vbox->addSpacing(vspace);

    auto buttonBox = new QDialogButtonBox(self);

    PushButton* measureButton = new PushButton(_("&Measure"));
    measureButton->setDefault(true);
    measureButton->sigClicked().connect([this](){ onMeasureButtonClicked(); });
    buttonBox->addButton(measureButton, QDialogButtonBox::ActionRole);
    
    buttonBox->addButton(QDialogButtonBox::Close);
    connect(buttonBox, SIGNAL(rejected()), self, SLOT(reject()));

    vbox->addWidget(buttonBox);

    sceneView = SceneView::instance();
    sceneWidget = sceneView->sceneWidget();
}


DistanceMeasurementDialog::~DistanceMeasurementDialog()
{
    delete impl;
}


DistanceMeasurementDialog::Impl::~Impl()
{
    delete objectPointPickMode;
}


void DistanceMeasurementDialog::show(DistanceMeasurementItem* item)
{
    impl->setMeasurementItem(item, true);
    impl->updateItemCandidates(RootItem::instance());
    Dialog::show();
}


void DistanceMeasurementDialog::Impl::setMeasurementItem(DistanceMeasurementItem* item, bool isNewSession)
{
    measurementItemConnections.disconnect();

    bool isBuiltinItem = false;
    
    if(item){
        measurementItem = item;

        measurementItemConnections.add(
            item->sigDisconnectedFromRoot().connect(
                [this]{ onMeasurementItemDisconnectedFromRoot(); }));
        
        measurementItemConnections.add(
            item->sigMeasurementConfigurationChanged().connect(
                [this]{ updateMeasurementConfigurationWidgets(); }));
        
        measurementItemConnections.add(
            item->sigCheckToggled().connect(
                [this](bool){ updateMeasurementConfigurationWidgets(); }));

        measurementItemConnections.add(
            item->sigNameChanged().connect(
                [this](const std::string&){ updateMeasurementConfigurationWidgets(); }));
        
        itemToAddNewMeasurementItem = item->parentItem();
        if(isNewSession){
            isExistingMeasurementItem = true;
        }
    } else {
        if(!builtinMeasurementItem){
            builtinMeasurementItem = new DistanceMeasurementItem;
            builtinMeasurementItem->setName(_("Distance Measurement"));
            builtinMeasurementItem->setChecked(true);
        }
        measurementItem = builtinMeasurementItem;
        if(isNewSession){
            itemToAddNewMeasurementItem = defaultItemToAddNewMeasurementItem.lock();
            if(!itemToAddNewMeasurementItem){
                itemToAddNewMeasurementItem = RootItem::instance();
            }
        }
        isExistingMeasurementItem = false;
        isBuiltinItem = true;
    }

    measurementItemConnections.add(
        measurementItem->sigDistanceUpdated().connect(
            [this](bool isValid){
                if(isValid){
                    updateDistanceDisplay();
                } else {
                    invalidateDistanceDisplay();
                }
            }));

    updateMeasurementConfigurationWidgets();

    createMeasurementItemCheck.blockSignals(true);
    createMeasurementItemCheck.setChecked(!isBuiltinItem);
    createMeasurementItemCheck.blockSignals(false);

    if(isBuiltinItem && measurementItem->isChecked() && measurementItem->isMeasurementActive()){
        showDistanceMarkerOfBuiltinMeasurementItem(true);
    }
}


void DistanceMeasurementDialog::Impl::updateMeasurementConfigurationWidgets()
{
    shortestDistanceCheck.blockSignals(true);
    shortestDistanceCheck.setChecked(measurementItem->isShortestDistanceMode());
    shortestDistanceCheck.blockSignals(false);

    shortestDistanceFixOneSideCheck.blockSignals(true);
    shortestDistanceFixOneSideCheck.setChecked(measurementItem->isShortestDistanceFixOneSideMode());
    shortestDistanceFixOneSideCheck.blockSignals(false);

    fixOneSideRadioGroup.blockSignals(true);
    fixOneSideRadios[measurementItem->shortestDistanceFixedSide()].setChecked(true);
    fixOneSideRadioGroup.blockSignals(false);

    distanceMarkerCheck.blockSignals(true);
    distanceMarkerCheck.setChecked(measurementItem->isChecked());
    distanceMarkerCheck.blockSignals(false);

    distanceLineWidthSpin.blockSignals(true);
    distanceLineWidthSpin.setValue(measurementItem->distanceLineWidth());
    distanceLineWidthSpin.blockSignals(false);

    distanceLineOverlayCheck.blockSignals(true);
    distanceLineOverlayCheck.setChecked(measurementItem->isDistanceLineOverlayEnabled());
    distanceLineOverlayCheck.blockSignals(false);

    measurementItemNameEdit.blockSignals(true);
    measurementItemNameEdit.setText(measurementItem->name());
    measurementItemNameEdit.blockSignals(false);
}


void DistanceMeasurementDialog::Impl::finalizeDistanceMeasurement()
{
    if(measurementItem == builtinMeasurementItem){
        measurementItem->stopMeasurement();
    }
    showDistanceMarkerOfBuiltinMeasurementItem(false);
    clearCandidateItems();

    for(int i=0; i < 2; ++i){
        activateObjectPointPickMode(i, false);
    }
    
    measurementItemConnections.disconnect();
    measurementItem.reset();
    itemToAddNewMeasurementItem.reset();
}
                 

void DistanceMeasurementDialog::onFinished(int result)
{
    impl->finalizeDistanceMeasurement();
}


void DistanceMeasurementDialog::Impl::onMeasurementItemDisconnectedFromRoot()
{
    showErrorDialog(
        format(_("The Distance measurement item \"{0}\" has been removed from the project and "
                 "close the distance measurement dialog."),
               measurementItem->name()));

    // The finalizeDistanceMeasurement function is called when the dialog is closed
    self->close();
}


void DistanceMeasurementDialog::setDefaultItemToAddNewMeasurementItem(Item* item)
{
    impl->defaultItemToAddNewMeasurementItem = item;
}


void DistanceMeasurementDialog::Impl::onCreateMeasurementItemCheckToggled(bool on)
{
    bool doCancel = false;
    
    if(on){
        if(!itemToAddNewMeasurementItem->isConnectedToRoot()){
            doCancel = true;
        } else {
            itemToAddNewMeasurementItem->addChildItem(measurementItem);
            showDistanceMarkerOfBuiltinMeasurementItem(false);
            builtinMeasurementItem = static_cast<DistanceMeasurementItem*>(measurementItem->clone());
            setMeasurementItem(measurementItem, false);
        }
    } else {
        bool doRemove = true;
        if(isExistingMeasurementItem){
            bool confirmed = showConfirmDialog(
                _("Removing Distance Measurement Item"),
                format(_("The distance measurement item \"{0}\" will be removed from the project. "
                         "Do you want to continue this operation?"),
                       measurementItem->name()));
            if(!confirmed){
                doRemove = false;
                doCancel = true;
            }
        }
        if(doRemove){
            bool isMeasurementActive = measurementItem->isMeasurementActive();
            itemToAddNewMeasurementItem = measurementItem->parentItem();
            measurementItemConnections.disconnect();
            measurementItem->removeFromParentItem(); // Measurement is stopped here
            builtinMeasurementItem = measurementItem;
            isExistingMeasurementItem = false;
            setMeasurementItem(nullptr, false);
            if(isMeasurementActive){
                startMeasurement();
            }
        }
    }

    if(doCancel){
        createMeasurementItemCheck.blockSignals(true);
        createMeasurementItemCheck.setChecked(!on);
        createMeasurementItemCheck.blockSignals(false);
    }
}


void DistanceMeasurementDialog::Impl::onMeasurementItemNameEditingFinishied()
{
    auto name = measurementItemNameEdit.string();
    if(name.empty()){
        measurementItemNameEdit.blockSignals(true);
        measurementItemNameEdit.setText(measurementItem->name());
        measurementItemNameEdit.blockSignals(false);
    } else {
        measurementItem->setName(name);
    }
}


void DistanceMeasurementDialog::Impl::clearCandidateItems()
{
    for(int i=0; i < 2; ++i){
        candidateItemInfoLists[i].clear();
        updateItemCombo(i);
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
                    info->itemConnection = connection;
                    if(item == measurementItem->targetItem(i)){
                        info->tracker = measurementItem->targetTracker(i);
                    }
                    candidateItemInfoLists[i].push_back(info);
                }
            }
            return true;
        });

    for(int i=0; i < 2; ++i){
        if(auto targetItem = measurementItem->targetItem(i)){
            updateItemCombo(i);
        }
    }
}


void DistanceMeasurementDialog::Impl::onItemComboAboutToShowPopup(int which)
{
    auto& combo = itemCombos[which];
    if(combo.count() == 0){
        updateItemCombo(which);
    }
}


void DistanceMeasurementDialog::Impl::clearItemComboBox(int which)
{
    auto& itemCombo = itemCombos[which];
    itemCombo.blockSignals(true);
    itemCombo.clear();
    itemCombo.blockSignals(false);
    
    auto& subEntryCombo = subEntryCombos[which];
    subEntryCombo.blockSignals(true);
    subEntryCombo.clear();
    subEntryCombo.blockSignals(false);
}


void DistanceMeasurementDialog::Impl::updateItemCombo(int which)
{
    clearItemComboBox(which);
    
    auto& combo = itemCombos[which];
    combo.blockSignals(true);
    Item* currentItem = measurementItem->targetItem(which);
    for(auto& info : candidateItemInfoLists[which]){
        combo.addItem(info->item->displayName().c_str());
        if(info->item == currentItem){
            combo.setCurrentIndex(combo.count() - 1);
        }
    }
    combo.blockSignals(false);

    if(!candidateItemInfoLists[which].empty()){
        setTargetItem(which, itemCombos[which].currentIndex(), false);
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
        auto currentItem = measurementItem->targetItem(i);
        if(!currentItem || item == currentItem){
            clearItemComboBox(i);
        } else {
            updateItemCombo(i);
        }
    }
}


void DistanceMeasurementDialog::Impl::setTargetItem(int which, int itemIndex, bool doUpdateItemComb)
{
    auto info = candidateItemInfoLists[which][itemIndex];
    auto tracker = info->getOrCreateTracker();

    measurementItem->setTargetItem(which, info->item, tracker);
    
    if(doUpdateItemComb){
        auto& itemCombo = itemCombos[which];
        if(itemIndex < itemCombo.count()){
            itemCombo.blockSignals(true);
            itemCombo.setCurrentIndex(itemIndex);
            itemCombo.blockSignals(false);
        }
    }

    if(tracker){
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

        if(shortestDistanceFixOneSideCheck.isChecked() || !shortestDistanceCheck.isChecked()){
            updatePointDisplay(which, tracker->getMeasurementPoint());
        }
    }
    
    if(measurementItem->isMeasurementActive()){
        startMeasurement();
    }
}


void DistanceMeasurementDialog::Impl::setTargetSubEntry(int which, int entryIndex, bool doUpdateCombo)
{
    bool updated = false;
    if(auto tracker = measurementItem->targetTracker(which)){
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

    if(measurementItem->isMeasurementActive()){
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
        if(pickedItemIndex >= 0){
            auto itemInfo = candidateItemInfoLists[whichObjectToPickPoint][pickedItemIndex];
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
                    if(itemCombos[whichObjectToPickPoint].count() == 0){
                        updateItemCombo(whichObjectToPickPoint);
                    }
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


void DistanceMeasurementDialog::Impl::onMeasureButtonClicked()
{
    if(measurementItem->hasValidTargetItems()){
        startMeasurement();
    } else {
        showErrorDialog(
            _("The distance measurement cannot be started since the target items are not specified."));
    }
}


bool DistanceMeasurementDialog::Impl::startMeasurement()
{
    bool isActive = measurementItem->startMeasurement();
    showDistanceMarker(isActive && distanceMarkerCheck.isChecked());
    return isActive;
}


void DistanceMeasurementDialog::Impl::onShortestDistanceCheckToggled(bool on)
{
    measurementItem->setShortestDistanceMode(on);
    if(measurementItem->isMeasurementActive()){
        measurementItem->startMeasurement();
    }
    measurementItem->notifyMeasurmentConfigurationChange();
}


void DistanceMeasurementDialog::Impl::onShortestDistanceFixOneSideCheckToggled(bool on)
{
    measurementItem->setShortestDistanceFixOneSideMode(on);
    if(measurementItem->isMeasurementActive()){
        measurementItem->startMeasurement();
    }
    measurementItem->notifyMeasurmentConfigurationChange();
}


void DistanceMeasurementDialog::Impl::onFixOneSideRadioToggled(int which)
{
    measurementItem->setShortestDistanceFixedSide(which);
    if(measurementItem->isMeasurementActive()){
        measurementItem->startMeasurement();
    }
    measurementItem->notifyMeasurmentConfigurationChange();
}


void DistanceMeasurementDialog::Impl::updatePointDisplay(int which, Vector3 p)
{
    p *= displayValueFormat->ratioToDisplayLength();
    pointLabels[which].setText(format(pointDisplayFormat, p.x(), p.y(), p.z()).c_str());
}
    

void DistanceMeasurementDialog::Impl::updateDistanceDisplay()
{
    double distance = measurementItem->distance();
    const Vector3& p0 = measurementItem->measurementPoint(0);
    const Vector3& p1 = measurementItem->measurementPoint(1);

    distanceLabel.setText(format(distanceDisplayFormat, displayValueFormat->toDisplayLength(distance)).c_str());
    updatePointDisplay(0, p0);
    updatePointDisplay(1, p1);

    Vector3 dp = displayValueFormat->ratioToDisplayLength() * (p1 - p0);
    for(int i=0; i < 3; ++i){
        distanceElementLabels[i].setText(format(distanceDisplayFormat, dp(i)).c_str());
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
    measurementItem->setChecked(on);
    
    if(measurementItem == builtinMeasurementItem){
        showDistanceMarkerOfBuiltinMeasurementItem(on);
    }
}


void DistanceMeasurementDialog::Impl::showDistanceMarkerOfBuiltinMeasurementItem(bool on)
{
    if(builtinMeasurementItem){
        auto systemNodeGroup = sceneWidget->systemNodeGroup();
        auto marker = builtinMeasurementItem->getScene();
        if(on){
            systemNodeGroup->addChildOnce(marker, true);
        } else {
            systemNodeGroup->removeChild(marker, true);
        }
    }
}


void DistanceMeasurementDialog::Impl::inputDistanceMarkerColorWithColorDialog()
{
    if(!colorDialog){
        colorDialog = new QColorDialog(self);
    }

    colorDialog->setOption(QColorDialog::DontUseNativeDialog);
    colorDialog->setWindowTitle(_("Distance Marker Color"));

    const Vector3f& c = measurementItem->distanceMarkerColor();
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


void DistanceMeasurementDialog::Impl::onDistanceLineOverlayCheckToggled(bool on)
{
    measurementItem->setDistanceLineOverlayEnabled(on);
    measurementItem->notifyMeasurmentConfigurationChange();
}


void DistanceMeasurementDialog::Impl::setDistanceMarkerColor(const QColor& color)
{
    Vector3f c(color.redF(), color.greenF(), color.blueF());
    measurementItem->setDistanceMarkerColor(c);
    measurementItem->notifyMeasurmentConfigurationChange();
    objectPointPickMode->setHighlightedPointColor(c);
    objectPointPickMode->setSelectedPointColor(c);
}


void DistanceMeasurementDialog::Impl::onDistanceLineWidthChanged(double width)
{
    measurementItem->setDistanceLineWidth(width);
    measurementItem->notifyMeasurmentConfigurationChange();
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
