#include "PointSetToMeshConversionDialog.h"
#include "PointCloudUtil.h"
#include <cnoid/ExtensionManager>
#include <cnoid/RootItem>
#include <cnoid/SceneItem>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/DisplayValueFormat>
#include <cnoid/Archive>
#include <cnoid/MathUtil>
#include <cnoid/Buttons>
#include <cnoid/ButtonGroup>
#include <cnoid/Separator>
#include <cnoid/PointSetItem>
#include <cnoid/SpinBox>
#include <cnoid/DoubleSpinBox>
#include <cnoid/CheckBox>
#include <cnoid/Format>
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <QStyle>
#include <pcl/console/print.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

bool doCloseDialogAfterConversion = false;

}

namespace cnoid {

class PointSetToMeshConversionDialog::Impl
{
public:
    PointSetToMeshConversionDialog* self;
    PointSetItemPtr targetPointSetItem;

    double normalEstimationSearchRadius;
    double mu;
    double searchRadius;

    ButtonGroup normalEstimationRadioGroup;
    RadioButton normalEstimationKSearchRadio;
    SpinBox normalEstimationKSearchSpin;
    RadioButton normalEstimationSearchRadiusRadio;
    DoubleSpinBox normalEstimationSearchRadiusSpin;
    QLabel normalEstimationSearchRadiusUnitLabel;
    SpinBox maxNearestNeighborSpin;
    DoubleSpinBox muSpin;
    QLabel muUnitLabel;
    DoubleSpinBox searchRadiusSpin;
    QLabel searchRadiusUnitLabel;
    DoubleSpinBox minAngleSpin;
    DoubleSpinBox maxAngleSpin;
    DoubleSpinBox maxSurfaceAngleSpin;
    CheckBox normalConsistencyCheck;
    ButtonGroup outputItemRadioGroup;
    RadioButton outputSceneItemRadio;
    RadioButton outputBodyItemRadio;
    QDialogButtonBox buttonBox;

    Impl(PointSetToMeshConversionDialog* self);
    void updateInterfaces();
    void convertTargetItems();
    void convertPointSetItemToMeshItem(PointSetItem* pointSetItem);
    bool store(Archive& archive);
    void restore(const Archive& archive);
};

}


void PointSetToMeshConversionDialog::initializeClass(ExtensionManager* ext)
{
    ext->setProjectArchiver(
        "PointSetToMeshConversionDialog",
        [](Archive& archive){ return PointSetToMeshConversionDialog::instance()->impl->store(archive); },
        [](const Archive& archive){ PointSetToMeshConversionDialog::instance()->impl->restore(archive); });
}


PointSetToMeshConversionDialog* PointSetToMeshConversionDialog::instance()
{
    static PointSetToMeshConversionDialog* dialog = new PointSetToMeshConversionDialog;;
    return dialog;
}


void PointSetToMeshConversionDialog::setAutoCloseAfterConversion(bool on)
{
    doCloseDialogAfterConversion = on;
}


PointSetToMeshConversionDialog::PointSetToMeshConversionDialog()
{
    impl = new Impl(this);
}


PointSetToMeshConversionDialog::~PointSetToMeshConversionDialog()
{
    delete impl;
}


PointSetToMeshConversionDialog::Impl::Impl(PointSetToMeshConversionDialog* self)
    : self(self),
      buttonBox(self)
{
    self->setWindowTitle(_("Point Set to Mesh Conversion"));

    auto vbox = new QVBoxLayout;
    self->setLayout(vbox);

    int hs = self->style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Normal estimation nearest neighbors:")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addSpacing(hs);
    normalEstimationKSearchRadio.setText("K");
    normalEstimationKSearchRadio.setChecked(true);
    normalEstimationRadioGroup.addButton(&normalEstimationKSearchRadio);
    hbox->addWidget(&normalEstimationKSearchRadio);
    normalEstimationKSearchSpin.setRange(1, 1000);
    normalEstimationKSearchSpin.setValue(20);
    hbox->addWidget(&normalEstimationKSearchSpin);
    hbox->addSpacing(hs);
    
    normalEstimationSearchRadiusRadio.setText(_("Radius"));
    normalEstimationRadioGroup.addButton(&normalEstimationSearchRadiusRadio);
    hbox->addWidget(&normalEstimationSearchRadiusRadio);
    normalEstimationSearchRadius = 0.03;
    normalEstimationSearchRadiusSpin.sigValueChanged().connect(
        [this](double value){
            normalEstimationSearchRadius = DisplayValueFormat::instance()->toMeter(value);
        });
    hbox->addWidget(&normalEstimationSearchRadiusSpin);
    hbox->addWidget(&normalEstimationSearchRadiusUnitLabel);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Max nearest neighbors")));
    maxNearestNeighborSpin.setRange(1, 10000);
    maxNearestNeighborSpin.setValue(50);
    hbox->addWidget(&maxNearestNeighborSpin);

    hbox->addSpacing(hs);
    hbox->addWidget(new QLabel(_("Mu")));
    mu = 2.5;
    muSpin.sigValueChanged().connect(
        [this](double value){
            mu = DisplayValueFormat::instance()->toMeter(value);
        });
    hbox->addWidget(&muSpin);
    hbox->addWidget(&muUnitLabel);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Search radius")));
    searchRadius = 0.05;
    searchRadiusSpin.sigValueChanged().connect(
        [this](double value){
            searchRadius = DisplayValueFormat::instance()->toMeter(value);
        });
    hbox->addWidget(&searchRadiusSpin);
    hbox->addWidget(&searchRadiusUnitLabel);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Triangle angle range")));
    minAngleSpin.setDecimals(1);
    minAngleSpin.setRange(0.1, 30.0);
    minAngleSpin.setValue(5.0);
    hbox->addWidget(&minAngleSpin);
    hbox->addWidget(new QLabel("-"));
    maxAngleSpin.setDecimals(1);
    maxAngleSpin.setRange(60.0, 179.9);
    maxAngleSpin.setValue(150.0);
    hbox->addWidget(&maxAngleSpin);
    hbox->addWidget(new QLabel("[deg]"));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Max surface angle")));
    maxSurfaceAngleSpin.setDecimals(0);
    maxSurfaceAngleSpin.setRange(1.0, 180.0);
    maxSurfaceAngleSpin.setValue(45.0);
    hbox->addWidget(&maxSurfaceAngleSpin);
    hbox->addWidget(new QLabel("[deg]"));

    normalConsistencyCheck.setText(_("Normal consistency"));
    hbox->addWidget(&normalConsistencyCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Output item type:")));
    outputSceneItemRadio.setText(_("Scene item"));
    outputSceneItemRadio.setChecked(true);
    outputItemRadioGroup.addButton(&outputSceneItemRadio, 0);
    hbox->addWidget(&outputSceneItemRadio);
    outputBodyItemRadio.setText(_("Body item"));
    outputItemRadioGroup.addButton(&outputBodyItemRadio, 1);
    hbox->addWidget(&outputBodyItemRadio);
    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator);

    auto convertButton = new PushButton(_("&Convert"));
    convertButton->setDefault(true);
    convertButton->sigClicked().connect([this](){ convertTargetItems(); });
    buttonBox.addButton(convertButton, QDialogButtonBox::ActionRole);

    buttonBox.addButton(QDialogButtonBox::Close);
    connect(&buttonBox, SIGNAL(rejected()), self, SLOT(reject()));
    
    vbox->addWidget(&buttonBox);
}


void PointSetToMeshConversionDialog::show(PointSetItem* targetItem)
{
    impl->targetPointSetItem = targetItem;
    
    if(targetItem){
        setWindowTitle(QString(_("Convert \"%1\" to Mesh")).arg(targetItem->displayName().c_str()));
    } else {
        setWindowTitle(_("Point Set to Mesh Conversion"));
    }

    impl->updateInterfaces();

    Dialog::show();
}


void PointSetToMeshConversionDialog::Impl::updateInterfaces()
{
    auto dvf = DisplayValueFormat::instance();

    normalEstimationSearchRadiusSpin.blockSignals(true);
    muSpin.blockSignals(true);
    searchRadiusSpin.blockSignals(true);

    if(dvf->isMillimeter()){
        normalEstimationSearchRadiusUnitLabel.setText("[mm]");
        muSpin.setDecimals(0);
        muSpin.setSingleStep(1.0);
        muUnitLabel.setText("[mm]");
        searchRadiusUnitLabel.setText("[mm]");
    } else {
        normalEstimationSearchRadiusUnitLabel.setText("[m]");
        muSpin.setDecimals(1);
        muSpin.setSingleStep(0.1);
        muUnitLabel.setText("[m]");
        searchRadiusUnitLabel.setText("[m]");
    }

    normalEstimationSearchRadiusSpin.setDecimals(dvf->lengthDecimals());
    normalEstimationSearchRadiusSpin.setSingleStep(dvf->lengthStep());
    normalEstimationSearchRadiusSpin.setRange(dvf->toDisplayLength(0.001), dvf->toDisplayLength(10.0));
    normalEstimationSearchRadiusSpin.setValue(dvf->toDisplayLength(normalEstimationSearchRadius));

    muSpin.setRange(dvf->toDisplayLength(0.1), dvf->toDisplayLength(100.0));
    muSpin.setValue(dvf->toDisplayLength(mu));

    searchRadiusSpin.setDecimals(dvf->lengthDecimals());
    searchRadiusSpin.setSingleStep(dvf->lengthStep());
    searchRadiusSpin.setRange(dvf->toDisplayLength(0.001), dvf->toDisplayLength(1000.0));
    searchRadiusSpin.setValue(dvf->toDisplayLength(searchRadius));
    
    normalEstimationSearchRadiusSpin.blockSignals(false);
    muSpin.blockSignals(false);
    searchRadiusSpin.blockSignals(false);
}    


void PointSetToMeshConversionDialog::onFinished(int result)
{
    impl->targetPointSetItem.reset();
}


void PointSetToMeshConversionDialog::Impl::convertTargetItems()
{
    for(auto& button : buttonBox.buttons()){
        button->setEnabled(false);
    }
    
    if(targetPointSetItem){
        convertPointSetItemToMeshItem(targetPointSetItem);
    } else {
        for(auto& item : RootItem::instance()->selectedItems<PointSetItem>()){
            convertPointSetItemToMeshItem(item);
        }
    }
    for(auto& button : buttonBox.buttons()){
        button->setEnabled(true);
    }

    if(doCloseDialogAfterConversion){
        self->accept();
    }
}


void PointSetToMeshConversionDialog::Impl::convertPointSetItemToMeshItem(PointSetItem* pointSetItem)
{
    auto mv = MessageView::instance();
    mv->put(formatR(_("Convert \"{0}\" to mesh data ..."), pointSetItem->displayName()));
    mv->flush();

    auto pclVerbosityLevel = pcl::console::getVerbosityLevel();
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    
    SgMeshPtr mesh =
        applyPCLGreedyProjectionTriangulation(
            pointSetItem->getScaledPointSet(),
            normalEstimationKSearchRadio.isChecked() ? normalEstimationKSearchSpin.value() : 0,
            normalEstimationSearchRadiusRadio.isChecked() ? normalEstimationSearchRadius : 0.0,
            maxNearestNeighborSpin.value(), mu,
            searchRadius,
            radian(minAngleSpin.value()), radian(maxAngleSpin.value()),
            radian(maxSurfaceAngleSpin.value()), normalConsistencyCheck.isChecked());

    pcl::console::setVerbosityLevel(pclVerbosityLevel);

    ItemPtr meshItem;

    if(mesh){
        SgShapePtr shape = new SgShape;
        shape->setMesh(mesh);

        if(outputSceneItemRadio.isChecked()){
            auto sceneItem = new SceneItem;
            sceneItem->topNode()->addChild(shape);
            sceneItem->topNode()->setPosition(pointSetItem->offsetPosition());
            meshItem = sceneItem;

        } else if(outputBodyItemRadio.isChecked()){
            auto bodyItem = new BodyItem;
            auto rootLink = bodyItem->body()->rootLink();
            rootLink->addShapeNode(shape);
            rootLink->setPosition(pointSetItem->offsetPosition());
            bodyItem->setLocationLocked(true);
            meshItem = bodyItem;
        }

        if(meshItem){
            meshItem->setName(formatR(_("{0}_Mesh"), pointSetItem->name()));
            meshItem->setTemporary();
            meshItem->unsetAttribute(Item::FileImmutable);
            meshItem->setChecked();
            pointSetItem->addChildItem(meshItem);
            mv->putln(_(" Done."));
        }
    }

    if(meshItem){
        mv->flush();
    } else {
        mv->putln(_("Failed."));
        mv->flush();
        showErrorDialog(_("Conversion to mesh data failed."));
    }
}


bool PointSetToMeshConversionDialog::Impl::store(Archive& archive)
{
    archive.write(
        "normal_estimation_nearest_neighbors_type",
        normalEstimationKSearchRadio.isChecked() ? "k" : "radius");
    archive.write("normal_estimation_k_search", normalEstimationKSearchSpin.value());
    archive.write("normal_estimation_search_radius", normalEstimationSearchRadius);
    archive.write("max_nearest_neighbor", maxNearestNeighborSpin.value());
    archive.write("mu", mu);
    archive.write("search_radius", searchRadius);
    archive.write("min_angle", minAngleSpin.value());
    archive.write("max_angle", maxAngleSpin.value());
    archive.write("max_surface_angle", maxSurfaceAngleSpin.value());
    archive.write("normal_consistency", normalConsistencyCheck.isChecked());
    archive.write("output_item_type", outputBodyItemRadio.isChecked() ? "body" : "scene");
    return true;
}


void PointSetToMeshConversionDialog::Impl::restore(const Archive& archive)
{
    if(archive.get("normal_estimation_nearest_neighbors_type", "") == "radius"){
        normalEstimationSearchRadiusRadio.setChecked(true);
    } else {
        normalEstimationKSearchRadio.setChecked(true);
    }
    normalEstimationKSearchSpin.setValue(
        archive.get("normal_estimation_k_search", normalEstimationKSearchSpin.value()));
    archive.read("normal_estimation_search_radius", normalEstimationSearchRadius);
    maxNearestNeighborSpin.setValue(archive.get("max_nearest_neighbor", maxNearestNeighborSpin.value()));
    archive.read("mu", mu);
    archive.read("search_radius", searchRadius);
    minAngleSpin.setValue(archive.get("min_angle", minAngleSpin.value()));
    maxAngleSpin.setValue(archive.get("max_angle", maxAngleSpin.value()));
    maxSurfaceAngleSpin.setValue(archive.get("max_surface_angle", maxSurfaceAngleSpin.value()));
    normalConsistencyCheck.setChecked(archive.get("normal_consistency", normalConsistencyCheck.isChecked()));

    if(archive.get("output_item_type", "") == "body"){
        outputBodyItemRadio.setChecked(true);
    } else {
        outputSceneItemRadio.setChecked(true);
    }
}
