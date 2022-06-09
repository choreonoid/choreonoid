#include "MultiBodyJointDisplacementWidget.h"
#include "JointDisplacementWidgetSet.h"
#include "BodyItem.h"
#include "KinematicBodyItemSet.h"
#include <cnoid/Archive>
#include <cnoid/MenuManager>
#include <QBoxLayout>
#include <QGridLayout>
#include <QStyle>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class MultiBodyJointDisplacementWidget::Impl
{
public:
    MultiBodyJointDisplacementWidget* self;
    KinematicBodyItemSetPtr currentBodyItemSet;
    ScopedConnection bodyItemSetConnection;
    QGridLayout* grid;
    int rowCounter;
    vector<JointDisplacementWidgetSet*> displacementWidgetSets;
    int labelOptions;
    
    Impl(MultiBodyJointDisplacementWidget* self);
    ~Impl();
    void setOptionMenuTo(MenuManager& menu);
    void setKinematicBodyItemSet(KinematicBodyItemSet* bodyItemSet);
    void updateDisplacementWidgets();
    bool storeState(Archive* archive);
    bool restoreState(const Archive* archive);
};

}


MultiBodyJointDisplacementWidget::MultiBodyJointDisplacementWidget(QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this);
}


MultiBodyJointDisplacementWidget::Impl::Impl(MultiBodyJointDisplacementWidget* self)
    : self(self)
{
    auto vbox = new QVBoxLayout;
    auto style = self->style();
    int lmargin = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int rmargin = style->pixelMetric(QStyle::PM_LayoutRightMargin);
    int hspacing = style->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
    int vspacing = style->pixelMetric(QStyle::PM_LayoutVerticalSpacing);
    vbox->setContentsMargins(lmargin, 0, rmargin, 0);
    self->setLayout(vbox);

    grid = new QGridLayout;
    grid->setHorizontalSpacing(hspacing / 2);
    grid->setVerticalSpacing(vspacing / 2);
    vbox->addLayout(grid);
    vbox->addStretch();

    labelOptions = JointDisplacementWidgetSet::BoldLabel;
}


MultiBodyJointDisplacementWidget::~MultiBodyJointDisplacementWidget()
{
    delete impl;
}


MultiBodyJointDisplacementWidget::Impl::~Impl()
{
    for(auto& widgetSet : displacementWidgetSets){
        delete widgetSet;
    }
}


void MultiBodyJointDisplacementWidget::setTargetBodyLabelOptions(int options)
{
    impl->labelOptions = options;
}


void MultiBodyJointDisplacementWidget::setKinematicBodyItemSet(KinematicBodyItemSet* bodyItemSet)
{
    impl->setKinematicBodyItemSet(bodyItemSet);
}


void MultiBodyJointDisplacementWidget::Impl::setKinematicBodyItemSet(KinematicBodyItemSet* bodyItemSet)
{
    if(bodyItemSet == currentBodyItemSet){
        return;
    }
    bodyItemSetConnection.disconnect();
    currentBodyItemSet = bodyItemSet;

    if(bodyItemSet){
        updateDisplacementWidgets();
        bodyItemSetConnection =
            bodyItemSet->sigBodySetChanged().connect(
                [this](){ updateDisplacementWidgets(); });
    }

}


void MultiBodyJointDisplacementWidget::Impl::updateDisplacementWidgets()
{
    auto bodyPartIndices = currentBodyItemSet->validBodyPartIndices();
    int numBodyParts = bodyPartIndices.size();

    if(numBodyParts > static_cast<int>(displacementWidgetSets.size())){
        displacementWidgetSets.resize(bodyPartIndices.size());
    }

    rowCounter = 0;
    int widgetSetIndex = 0;
    while(widgetSetIndex < numBodyParts){
        int bodyPartIndex = bodyPartIndices[widgetSetIndex];
        auto part = currentBodyItemSet->bodyItemPart(bodyPartIndex);
        auto& widgetSet = displacementWidgetSets[widgetSetIndex];
        if(!widgetSet){
            widgetSet = new JointDisplacementWidgetSet(self, grid, &rowCounter);
        }
        widgetSet->setTargetBodyLabelEnabled(numBodyParts >= 2, labelOptions);
        widgetSet->setBodyItem(part->bodyItem());
        widgetSet->setVisible(true);
        ++widgetSetIndex;
    }
    
    // Deactivate the remaining displacement widgets
    while(widgetSetIndex < static_cast<int>(displacementWidgetSets.size())){
        auto widgetSet = displacementWidgetSets[widgetSetIndex];
        widgetSet->setBodyItem(nullptr);
        widgetSet->setVisible(false);
        ++widgetSetIndex;
    }
}


KinematicBodyItemSet* MultiBodyJointDisplacementWidget::kinematicBodyItemSet()
{
    return impl->currentBodyItemSet;
}
