#include "ManipulatorStatementViewBase.h"
#include "ManipulatorProgramViewBase.h"
#include "ManipulatorProgramItemBase.h"
#include "ManipulatorStatement.h"
#include <cnoid/Archive>
#include <QBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <map>
#include <typeindex>

using namespace std;
using namespace cnoid;

namespace cnoid {

class ManipulatorStatementPanel::Impl
{
public:
    ManipulatorStatementPanel* self;
    ManipulatorProgramItemBasePtr programItem;
    ManipulatorStatementPtr statement;

    Impl(ManipulatorStatementPanel* self);
    void activate(ManipulatorProgramItemBase* programItem, ManipulatorStatement* statement);
    void deactivate();
};

class ManipulatorStatementViewBase::Impl
{
public:
    ManipulatorStatementViewBase* self;

    ManipulatorProgramItemBasePtr currentProgramItem;
    ManipulatorStatementPtr currentStatement;

    ScopedConnection programViewConnection;
    QScrollArea scrollArea;
    QLabel statementLabel;

    map<std::type_index, PanelFactoryFunction> panelFactoryMap;
    map<std::type_index, ManipulatorStatementPanel*> panelMap;
    
    Impl(ManipulatorStatementViewBase* self);
    ~Impl();
    void onActivated();
    ManipulatorStatementPanel* getOrCreateStatementPanel(ManipulatorStatement* statement);
    void setStatement(ManipulatorProgramItemBase* programItem, ManipulatorStatement* statement);
};

}


ManipulatorStatementPanel::ManipulatorStatementPanel()
{
    impl = new Impl(this);
}


ManipulatorStatementPanel::~ManipulatorStatementPanel()
{
    delete impl;
}


ManipulatorStatementPanel::Impl::Impl(ManipulatorStatementPanel* self)
    : self(self)
{

}


void ManipulatorStatementPanel::Impl::activate
(ManipulatorProgramItemBase* programItem, ManipulatorStatement* statement)
{
    this->programItem = programItem;
    this->statement = statement;

    self->onActivated();
}


void ManipulatorStatementPanel::Impl::deactivate()
{
    self->onDeactivated();
    
    programItem.reset();
    statement.reset();
}


void ManipulatorStatementPanel::onDeactivated()
{

}


ManipulatorProgramItemBase* ManipulatorStatementPanel::currentProgramItem()
{
    return impl->programItem;
}


ManipulatorStatement* ManipulatorStatementPanel::getCurrentStatement()
{
    return impl->statement;
}


ManipulatorStatementViewBase::ManipulatorStatementViewBase()
{
    impl = new Impl(this);
}


ManipulatorStatementViewBase::Impl::Impl(ManipulatorStatementViewBase* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    vbox->addWidget(&statementLabel);

    scrollArea.setFrameShape(QFrame::NoFrame);
    scrollArea.setWidgetResizable(true);
    scrollArea.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea.setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    vbox->addWidget(&scrollArea);

    self->setLayout(vbox);

    currentProgramItem = nullptr;
}


ManipulatorStatementViewBase::~ManipulatorStatementViewBase()
{
    delete impl;
}


ManipulatorStatementViewBase::Impl::~Impl()
{
    for(auto& kv : panelMap){
        auto& panel = kv.second;
        delete panel;
    }
}


void ManipulatorStatementViewBase::onActivated()
{
    impl->onActivated();
}


void ManipulatorStatementViewBase::Impl::onActivated()
{
    auto programView = self->getProgramView();
    
    programViewConnection =
        programView->sigCurrentStatementChanged().connect(
            [=](ManipulatorStatement* statement){
                setStatement(programView->currentProgramItem(), statement); });

    setStatement(programView->currentProgramItem(), programView->currentStatement());
}


void ManipulatorStatementViewBase::onDeactivated()
{
    impl->programViewConnection.disconnect();
    impl->currentProgramItem = nullptr;
    impl->currentStatement = nullptr;
}


void ManipulatorStatementViewBase::registerPanelFactory(const std::type_info& statementType, PanelFactoryFunction factory)
{
    impl->panelFactoryMap[statementType] = factory;
}


ManipulatorStatementPanel* ManipulatorStatementViewBase::Impl::getOrCreateStatementPanel(ManipulatorStatement* statement)
{
    ManipulatorStatementPanel* panel = nullptr;

    auto& ptype = typeid(*statement);
    auto panelIter = panelMap.find(ptype);

    if(panelIter != panelMap.end()){
        panel = panelIter->second;

    } else {
        auto factoryIter = panelFactoryMap.find(ptype);
        if(factoryIter != panelFactoryMap.end()){
            auto& factory = factoryIter->second;
            panel = factory();
            panelMap[ptype] = panel;
        }
    }

    return panel;
}


void ManipulatorStatementViewBase::Impl::setStatement(ManipulatorProgramItemBase* programItem, ManipulatorStatement* statement)
{
    if(programItem == currentProgramItem && statement == currentStatement){
        return;
    }

    auto prevPanel = dynamic_cast<ManipulatorStatementPanel*>(scrollArea.widget());
    if(prevPanel){
        prevPanel->impl->deactivate();
    }

    ManipulatorStatementPanel* panel = nullptr;

    if(statement){
        statementLabel.setText(statement->label(0).c_str());
        panel = getOrCreateStatementPanel(statement);
        if(panel){
            panel->impl->activate(programItem, statement);
        }
    } else {
        statementLabel.setText("---");
    }

    if(panel != prevPanel){
        scrollArea.takeWidget();
        if(panel){
            scrollArea.setWidget(panel);
        }
    }

    currentProgramItem = programItem;
    currentStatement = statement;
}


bool ManipulatorStatementViewBase::storeState(Archive& archive)
{
    return true;
}


bool ManipulatorStatementViewBase::restoreState(const Archive& archive)
{
    return true;
}
