#include "YawMomentCompensationDialog.h"
#include "YawMomentCompensationFilter.h"
#include <cnoid/Plugin>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/MultiValueSeqItem>
#include <cnoid/MessageOut>
#include <cnoid/Archive>
#include <cnoid/CheckBox>
#include <cnoid/SpinBox>
#include <cnoid/DoubleSpinBox>
#include <cnoid/ButtonGroup>
#include <cnoid/RadioButton>
#include <cnoid/PushButton>
#include <cnoid/Separator>
#include <cnoid/TreeWidget>
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

Plugin* plugin = nullptr;

}

namespace cnoid {

class YawMomentCompensationDialog::Impl
{
public:
    YawMomentCompensationDialog* self;
    unique_ptr<YawMomentCompensationFilter> filter;
    MessageOut* mout;
    TreeWidget jointWeightTree;
    DoubleSpinBox kp_recoverySpin;
    DoubleSpinBox kd_recoverySpin;
    CheckBox jointRangeConstraintCheck;
    RadioButton inputRangeModeRadio;
    RadioButton modelRangeModeRadio;
    CheckBox opposingAdjustmentJointCheck;
    SpinBox opposingAdjustmentJointIdSpin;
    SpinBox opposingAdjustmentReferenceJointIdSpin;
    DoubleSpinBox frictionCoefficientSpin;
    DoubleSpinBox soleRadiusSpin;
    RadioButton dynamicNormalForceModeRadio;
    RadioButton constantNormalForceModeRadio;
    DoubleSpinBox constantNormalForceFactorSpin;
    CheckBox bothFeetSupportFrictionCheck;
    CheckBox yawMomentSeqOutputCheck;
    CheckBox copSeqOutputCheck;

    Impl(YawMomentCompensationDialog* self);
    void addTargetJoint(int jointId, double weight, double velLimitRatio);
    void removeSelectedTargetJoints();
    bool applyFilter(BodyItem* bodyItem, BodyMotionItem* motionItem);
    bool store(Archive& archive);
    void restore(const Archive& archive);
};

}

void YawMomentCompensationDialog::initialize(Plugin* plugin)
{
    if(!::plugin){
        ::plugin = plugin;
        plugin->setProjectArchiver(
            "YawMomentCompensationDialog",
            [](Archive& archive){ return instance()->impl->store(archive); },
            [](const Archive& archive){ return instance()->impl->restore(archive); });
    }
}


YawMomentCompensationDialog* YawMomentCompensationDialog::instance()
{
    static YawMomentCompensationDialog* instance_ = plugin->manage(new YawMomentCompensationDialog);
    return instance_;
}


YawMomentCompensationDialog::YawMomentCompensationDialog()
{
    impl = new Impl(this);
}


YawMomentCompensationDialog::Impl::Impl(YawMomentCompensationDialog* self_)
    : self(self_)
{
    self->setWindowTitle(_("Yaw moment compensation filter"));

    mout = MessageOut::master();

    auto vbox = new QVBoxLayout;
    self->setLayout(vbox);

    QHBoxLayout* hbox;

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Joint set used for compensation:")));
    hbox->addStretch();

    auto addTargetJointButton = new PushButton(_("Add"));
    addTargetJointButton->sigClicked().connect([this]{ addTargetJoint(0, 1.0, 1.0); });
    hbox->addWidget(addTargetJointButton);

    auto removeTargetJointButton = new PushButton(_("Remove"));
    removeTargetJointButton->sigClicked().connect([this]{ removeSelectedTargetJoints(); });
    hbox->addWidget(removeTargetJointButton);
    
    vbox->addLayout(hbox);

    jointWeightTree.setIndentation(0);
    jointWeightTree.setAlternatingRowColors(true);
    jointWeightTree.setVerticalGridLineShown(true);
    jointWeightTree.setSelectionMode(QAbstractItemView::ExtendedSelection);
    jointWeightTree.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    jointWeightTree.setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    jointWeightTree.setDragDropMode(QAbstractItemView::InternalMove);

    jointWeightTree.setColumnCount(3);

#ifdef Q_OS_WIN32
    auto header = jointWeightTree.header();
    header->setSectionResizeMode(0, QHeaderView::Stretch);
    header->setSectionResizeMode(1, QHeaderView::Stretch);
    header->setSectionResizeMode(2, QHeaderView::Stretch);
    header->setStretchLastSection(false);
#endif
    
    auto headerItem = new QTreeWidgetItem;
    headerItem->setText(0, _("Joint ID"));
    headerItem->setTextAlignment(0, Qt::AlignCenter);
    headerItem->setText(1, _("Weight"));
    headerItem->setTextAlignment(1, Qt::AlignCenter);
    headerItem->setText(2, _("Vel. limit ratio"));
    headerItem->setTextAlignment(2, Qt::AlignCenter);
    jointWeightTree.setHeaderItem(headerItem);

    vbox->addWidget(&jointWeightTree, 1);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Coefficients of joint recovery force:")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addSpacing(20);
    hbox->addWidget(new QLabel(_("P:")));
    kp_recoverySpin.setDecimals(2);
    kp_recoverySpin.setRange(0.0, 999.0);
    kp_recoverySpin.setSingleStep(0.01);
    kp_recoverySpin.setValue(YawMomentCompensationFilter::defaultRecoveryForcePGain());
    hbox->addWidget(&kp_recoverySpin);

    hbox->addWidget(new QLabel("D:"));
    kd_recoverySpin.setDecimals(2);
    kd_recoverySpin.setRange(0.0, 999.0);
    kd_recoverySpin.setSingleStep(0.01);
    kd_recoverySpin.setValue(YawMomentCompensationFilter::defaultRecoveryForceDGain());
    hbox->addWidget(&kd_recoverySpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    jointRangeConstraintCheck.setText(_("Joint displacement range constraints"));
    jointRangeConstraintCheck.setChecked(true);
    hbox->addWidget(&jointRangeConstraintCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addSpacing(20);
    auto rangeModeRadioGroup = new ButtonGroup(self);
    inputRangeModeRadio.setText(_("Input Motion"));
    inputRangeModeRadio.setChecked(true);
    rangeModeRadioGroup->addButton(&inputRangeModeRadio);
    hbox->addWidget(&inputRangeModeRadio);
    modelRangeModeRadio.setText(_("Model"));
    rangeModeRadioGroup->addButton(&modelRangeModeRadio);
    hbox->addWidget(&modelRangeModeRadio);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    opposingAdjustmentJointCheck.setText(_("Opposing adjustment joint:"));
    hbox->addWidget(&opposingAdjustmentJointCheck);
    opposingAdjustmentJointIdSpin.setRange(0, 99);
    hbox->addWidget(&opposingAdjustmentJointIdSpin);
    hbox->addWidget(new QLabel(_("Reference joint:")));
    opposingAdjustmentReferenceJointIdSpin.setRange(0, 99);
    hbox->addWidget(&opposingAdjustmentReferenceJointIdSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Coefficient of friction:")));
    frictionCoefficientSpin.setDecimals(2);
    frictionCoefficientSpin.setRange(0.0, 9.99);
    frictionCoefficientSpin.setSingleStep(0.01);
    frictionCoefficientSpin.setValue(YawMomentCompensationFilter::defaultFrictionCoefficient());
    hbox->addWidget(&frictionCoefficientSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Sole radius:")));
    soleRadiusSpin.setDecimals(1);
    soleRadiusSpin.setRange(0.1, 100.0);
    soleRadiusSpin.setSingleStep(0.1);
    soleRadiusSpin.setValue(YawMomentCompensationFilter::defaultSoleRadius() * 100.0 /* cm */);
    hbox->addWidget(&soleRadiusSpin);
    hbox->addWidget(new QLabel(_("[cm]")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    bothFeetSupportFrictionCheck.setText(_("Enable friction moment of both feet support"));
    bothFeetSupportFrictionCheck.setChecked(true);
    hbox->addWidget(&bothFeetSupportFrictionCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Floor normal force:")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addSpacing(20);
    auto normalForceRadioGroup = new ButtonGroup(self);
    dynamicNormalForceModeRadio.setText(_("Inverse dynamics"));
    dynamicNormalForceModeRadio.setChecked(true);
    normalForceRadioGroup->addButton(&dynamicNormalForceModeRadio);
    hbox->addWidget(&dynamicNormalForceModeRadio);
    constantNormalForceModeRadio.setText(_("Constant"));
    normalForceRadioGroup->addButton(&constantNormalForceModeRadio);
    hbox->addWidget(&constantNormalForceModeRadio);

    constantNormalForceFactorSpin.setDecimals(2);
    constantNormalForceFactorSpin.setRange(0.0, 10.);
    constantNormalForceFactorSpin.setSingleStep(0.01);
    constantNormalForceFactorSpin.setValue(
        YawMomentCompensationFilter::defaultConstantNormalForceFactor());
    hbox->addWidget(&constantNormalForceFactorSpin);
    hbox->addWidget(new QLabel("* mg"));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    yawMomentSeqOutputCheck.setText(_("Output yaw moment sequence"));
    hbox->addWidget(&yawMomentSeqOutputCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    copSeqOutputCheck.setText(_("Output center of pressure sequence"));
    hbox->addWidget(&copSeqOutputCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    vbox->addStretch();

    vbox->addWidget(new HSeparator);
    auto buttonBox = new QDialogButtonBox(self);
    
    auto applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    applyButton->sigClicked().connect([this](){ self->accept(); });
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);

    auto cancelButton = new PushButton(_("&Cancel"));
    cancelButton->sigClicked().connect([this]{ self->reject(); });
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    
    vbox->addWidget(buttonBox);

    addTargetJoint(0, 1.0, 1.0);
}


YawMomentCompensationDialog::~YawMomentCompensationDialog()
{
    delete impl;
}


void YawMomentCompensationDialog::Impl::addTargetJoint(int jointId, double weight, double velLimitRatio)
{
    auto item = new QTreeWidgetItem;
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable | Qt::ItemIsDragEnabled);

    // Joint ID
    item->setData(0, Qt::DisplayRole, jointId);
    item->setTextAlignment(0, Qt::AlignCenter);

     // Weight
    item->setData(1, Qt::DisplayRole, weight);
    item->setTextAlignment(1, Qt::AlignCenter);
    
    item->setData(2, Qt::DisplayRole, velLimitRatio); // Vel. limit ratio
    item->setTextAlignment(2, Qt::AlignCenter);
    
    jointWeightTree.addTopLevelItem(item);
}


void YawMomentCompensationDialog::Impl::removeSelectedTargetJoints()
{
    for(auto& item : jointWeightTree.selectedItems()){
        delete item;
    }
}


void YawMomentCompensationDialog::onAccepted()
{
    for(auto& motionItem : RootItem::instance()->selectedItems<BodyMotionItem>()){
        if(auto bodyItem = motionItem->findOwnerItem<BodyItem>()){
            BodyMotionItemPtr outputMotionItem = static_cast<BodyMotionItem*>(motionItem->clone());
            outputMotionItem->setName(motionItem->name() + "'");
            outputMotionItem->setTemporary(true);

            if(impl->applyFilter(bodyItem, outputMotionItem)){
                motionItem->parentItem()->insertChild(motionItem->nextItem(), outputMotionItem);
                impl->mout->putln(
                    format(_("Body motion \"{0}\" has been converted to \"{1}\" by the yaw moment compensation filter."),
                           motionItem->name(), outputMotionItem->name()));
            } else {
                impl->mout->putln(
                    format(_("Failed to apply the yaw moment compensation filter to \"{0}\"."),
                           motionItem->name()));
            }
        }
    }
}


bool YawMomentCompensationDialog::applyFilter(BodyItem* bodyItem, BodyMotionItem* motionItem)
{
    bool result = impl->applyFilter(bodyItem, motionItem);

    if(result){
        impl->mout->putln(
            format(_("Body motion \"{0}\" has been updated by the yaw moment compensation filter."),
                   motionItem->name()));
    } else {
        impl->mout->putln(
            format(_("Failed to apply the yaw moment compensation filter to \"{0}\"."),
                   motionItem->name()));
    }

    return result;
}


bool YawMomentCompensationDialog::Impl::applyFilter(BodyItem* bodyItem, BodyMotionItem* motionItem)
{
    if(!filter){
        filter = make_unique<YawMomentCompensationFilter>();
    } else {
        filter->clearJointWeights();
    }

    Body* body = bodyItem->body();
    int index = 0;
    int n = jointWeightTree.topLevelItemCount();
    bool isValid = true;
    
    while(index < n){
        auto item = jointWeightTree.topLevelItem(index);
        int jointId = item->data(0, Qt::DisplayRole).toInt();
        if(jointId < 0 || jointId >= body->numJoints()){
            isValid = false;
            break;
        }
        double weight = item->data(1, Qt::DisplayRole).toDouble();
        if(weight <= 0.0){
            isValid = false;
            break;
        }
        double ratio = item->data(2, Qt::DisplayRole).toDouble();
        if(ratio <= 0.0){
            isValid = false;
            break;
        }
        filter->setJointWeight(jointId, true, weight, ratio);
        ++index;
    }

    if(!isValid){
        mout->putErrorln(
            format(_("The filter cannot apply to {0} because the parameter values of {1}-th target joint is invalid."),
                   bodyItem->name(), index + 1));
        return false;
    }
            
    filter->setRecoveryForceGains(kp_recoverySpin.value(), kd_recoverySpin.value());

    if(inputRangeModeRadio.isChecked()){
        filter->setJointDisplacementRangeMode(YawMomentCompensationFilter::InputMotionJointDisplacementRange);
    } else {
        filter->setJointDisplacementRangeMode(YawMomentCompensationFilter::ModelJointDisplacementRange);
    }
    filter->setJointDisplacementRangeConstraintEnabled(jointRangeConstraintCheck.isChecked());

    if(opposingAdjustmentJointCheck.isChecked()){
        filter->setOpposingAdjustmentJoint(
            opposingAdjustmentJointIdSpin.value(), opposingAdjustmentReferenceJointIdSpin.value());
    } else {
        filter->resetOpposingAdjustmentJoint();
    }
    
    filter->setFrictionCoefficient(frictionCoefficientSpin.value());
    filter->setSoleRadius(soleRadiusSpin.value() / 100.0); // From cm to meter
    filter->setDynamicNormalForceMode(dynamicNormalForceModeRadio.isChecked());
    filter->setConstantNormalForceFactor(constantNormalForceFactorSpin.value());
    filter->enableMomentOfBothFeetSupporting(bothFeetSupportFrictionCheck.isChecked());

    if(!yawMomentSeqOutputCheck.isChecked()){
        filter->setYawMomentSeqOutput(nullptr);
    } else {
        auto yawMomentSeq = make_shared<MultiValueSeq>();
        yawMomentSeq->setSeqContentName("YawMoment");
        motionItem->motion()->setExtraSeq(yawMomentSeq);
        filter->setYawMomentSeqOutput(yawMomentSeq);
    }

    if(!copSeqOutputCheck.isChecked()){
        filter->setCopSeqOutput(nullptr);
    } else {
        auto copSeq = make_shared<Vector3Seq>();
        copSeq->setSeqContentName("COP");
        motionItem->motion()->setExtraSeq(copSeq);
        filter->setCopSeqOutput(copSeq);
    }
    
    return filter->apply(body, *motionItem->motion());
}


bool YawMomentCompensationDialog::Impl::store(Archive& archive)
{
    int n = jointWeightTree.topLevelItemCount();
    if(n > 0){
        auto jointList = archive.createListing("target_joints");
        for(int i=0; i < n; ++i){
            auto item = jointWeightTree.topLevelItem(i);
            MappingPtr info = new Mapping;
            info->setFlowStyle();
            info->write("joint_id", item->data(0, Qt::DisplayRole).toInt());
            info->write("weight", item->data(1, Qt::DisplayRole).toDouble());
            info->write("velocity_limit_ratio", item->data(2, Qt::DisplayRole).toDouble());
            jointList->append(info);
        }
    }

    archive.write("kp_recovery", kp_recoverySpin.value());
    archive.write("kd_recovery", kd_recoverySpin.value());

    archive.write(
        "joint_displacement_range_mode",
        inputRangeModeRadio.isChecked() ? "input_motion" : "model");
    archive.write("enable_joint_displacement_range_constraints", jointRangeConstraintCheck.isChecked());

    archive.write("enable_opposing_adjustment_joint", opposingAdjustmentJointCheck.isChecked());
    archive.write("opposing_adjustment_joint_id", opposingAdjustmentJointIdSpin.value());
    archive.write("opposing_adjustment_reference_joint_id", opposingAdjustmentReferenceJointIdSpin.value());
    
    archive.write("friction_coefficient", frictionCoefficientSpin.value());
    archive.write("sole_radius", soleRadiusSpin.value() / 100.0); // from cm to meter
    archive.write("floor_normal_force_mode",
                  dynamicNormalForceModeRadio.isChecked() ? "inverse_dynamics" : "constant");
    archive.write("constant_normal_force", constantNormalForceFactorSpin.value());
    archive.write("enable_both_feet_support_friction", bothFeetSupportFrictionCheck.isChecked());
    archive.write("output_yaw_moment_seq", yawMomentSeqOutputCheck.isChecked());
    archive.write("output_cop_seq", copSeqOutputCheck.isChecked());
    return true;
}


void YawMomentCompensationDialog::Impl::restore(const Archive& archive)
{
    jointWeightTree.clear();
    auto jointList = archive.findListing("target_joints");
    if(jointList->isValid()){
        for(auto& node : *jointList){
            auto info = node->toMapping();
            int id;
            if(info->read("joint_id", id)){
                addTargetJoint(id, info->get("weight", 1.0), info->get("velocity_limit_ratio", 1.0));
            }
        }
    }

    kp_recoverySpin.setValue(archive.get("kp_recovery", kp_recoverySpin.value()));
    kd_recoverySpin.setValue(archive.get("kd_recovery", kd_recoverySpin.value()));

    string mode;
    if(archive.read("joint_displacement_range_mode", mode)){
        if(mode == "input_motion"){
            inputRangeModeRadio.setChecked(true);
        } else if(mode == "model"){
            modelRangeModeRadio.setChecked(true);
        }
    }

    opposingAdjustmentJointCheck.setChecked(
        archive.get("enable_opposing_adjustment_joint", opposingAdjustmentJointCheck.isChecked()));
    opposingAdjustmentJointIdSpin.setValue(
        archive.get("opposing_adjustment_joint_id", opposingAdjustmentJointIdSpin.value()));
    opposingAdjustmentReferenceJointIdSpin.setValue(
        archive.get("opposing_adjustment_reference_joint_id", opposingAdjustmentReferenceJointIdSpin.value()));

    jointRangeConstraintCheck.setChecked(
        archive.get("enable_joint_displacement_range_constraints", jointRangeConstraintCheck.isChecked()));
    
    frictionCoefficientSpin.setValue(archive.get("friction_coefficient", frictionCoefficientSpin.value()));
    soleRadiusSpin.setValue(
        archive.get("sole_radius", soleRadiusSpin.value() / 100.0) * 100.0);

    if(archive.read("floor_normal_force_mode", mode)){
        if(mode == "inverse_dynamics"){
            dynamicNormalForceModeRadio.setChecked(true);
        } else if(mode == "constant"){
            constantNormalForceModeRadio.setChecked(true);
        }
    }
    constantNormalForceFactorSpin.setValue(archive.get("constant_normal_force", constantNormalForceFactorSpin.value()));
    bothFeetSupportFrictionCheck.setChecked(
        archive.get("enable_both_feet_support_friction", bothFeetSupportFrictionCheck.isChecked()));

    yawMomentSeqOutputCheck.setChecked(archive.get("output_yaw_moment_seq", yawMomentSeqOutputCheck.isChecked()));
    copSeqOutputCheck.setChecked(archive.get("output_cop_seq", copSeqOutputCheck.isChecked()));
}
