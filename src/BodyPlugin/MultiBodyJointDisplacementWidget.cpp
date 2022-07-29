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
    BodyItemPtr persistentBodyItem;
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


void MultiBodyJointDisplacementWidget::setPersistentBodyItem(BodyItem* bodyItem)
{
    impl->persistentBodyItem = bodyItem;
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

    updateDisplacementWidgets();

    if(bodyItemSet){
        bodyItemSetConnection =
            bodyItemSet->sigBodySetChanged().connect(
                [this](){ updateDisplacementWidgets(); });
    }

}


void MultiBodyJointDisplacementWidget::Impl::updateDisplacementWidgets()
{
    vector<BodyItem*> bodyItems;
    bool isPersistentBodyItemFound = false;
    
    if(currentBodyItemSet){
        for(auto& index : currentBodyItemSet->validBodyPartIndices()){
            auto bodyItem = currentBodyItemSet->bodyItem(index);
            if(bodyItem->body()->numJoints() > 0){
                bodyItems.push_back(bodyItem);
            }
            if(bodyItem == persistentBodyItem){
                isPersistentBodyItemFound = true;
            }
        }
    }
    if(persistentBodyItem && !isPersistentBodyItemFound){
        if(persistentBodyItem->body()->numJoints() > 0){
            bodyItems.insert(bodyItems.begin(), persistentBodyItem);
        }
    }
    int numBodyItems = bodyItems.size();

    if(numBodyItems > static_cast<int>(displacementWidgetSets.size())){
        displacementWidgetSets.resize(numBodyItems);
    }

    rowCounter = 0;
    int widgetSetIndex = 0;
    while(widgetSetIndex < numBodyItems){
        auto& widgetSet = displacementWidgetSets[widgetSetIndex];
        if(!widgetSet){
            widgetSet = new JointDisplacementWidgetSet(self, grid, &rowCounter);
        }
        widgetSet->setTargetBodyLabelEnabled(numBodyItems >= 2, labelOptions);
        widgetSet->setBodyItem(bodyItems[widgetSetIndex]);
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
