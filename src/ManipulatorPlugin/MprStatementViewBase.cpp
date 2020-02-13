#include "MprStatementViewBase.h"
#include "MprProgramViewBase.h"
#include "MprProgramItemBase.h"
#include "MprStatement.h"
#include <cnoid/Archive>
#include <QBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <map>
#include <typeindex>

using namespace std;
using namespace cnoid;

namespace cnoid {

class MprStatementPanel::Impl
{
public:
    MprStatementPanel* self;
    MprProgramItemBasePtr programItem;
    MprStatementPtr statement;
    ScopedConnection statementUpdateConnection;

    Impl(MprStatementPanel* self);
    void activate(MprProgramItemBase* programItem, MprStatement* statement);
    void deactivate();
};

class MprStatementViewBase::Impl
{
public:
    MprStatementViewBase* self;

    MprProgramItemBasePtr currentProgramItem;
    MprStatementPtr currentStatement;

    ScopedConnection programViewConnection;
    QScrollArea scrollArea;
    QLabel statementLabel;

    map<std::type_index, PanelFactoryFunction> panelFactoryMap;
    map<std::type_index, MprStatementPanel*> panelMap;
    
    Impl(MprStatementViewBase* self);
    ~Impl();
    void onActivated();
    MprStatementPanel* getOrCreateStatementPanel(MprStatement* statement);
    void setStatement(MprProgramItemBase* programItem, MprStatement* statement);
};

}


MprStatementPanel::MprStatementPanel()
{
    impl = new Impl(this);
}


MprStatementPanel::~MprStatementPanel()
{
    delete impl;
}


MprStatementPanel::Impl::Impl(MprStatementPanel* self)
    : self(self)
{

}


void MprStatementPanel::Impl::activate
(MprProgramItemBase* programItem_, MprStatement* statement_)
{
    programItem = programItem_;
    statement = statement_;

    statementUpdateConnection =
        programItem->program()->sigStatementUpdated().connect(
            [this](MprStatement* updated){
                if(updated == statement){
                    self->onStatementUpdated();
                }
            });

    self->onActivated();
}


void MprStatementPanel::Impl::deactivate()
{
    self->onDeactivated();
    
    programItem.reset();
    statement.reset();
    statementUpdateConnection.disconnect();
}


void MprStatementPanel::onDeactivated()
{

}


void MprStatementPanel::onStatementUpdated()
{

}


MprProgramItemBase* MprStatementPanel::currentProgramItem()
{
    return impl->programItem;
}


MprStatement* MprStatementPanel::getCurrentStatement()
{
    return impl->statement;
}


MprStatementViewBase::MprStatementViewBase()
{
    impl = new Impl(this);
}


MprStatementViewBase::Impl::Impl(MprStatementViewBase* self)
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


MprStatementViewBase::~MprStatementViewBase()
{
    delete impl;
}


MprStatementViewBase::Impl::~Impl()
{
    for(auto& kv : panelMap){
        auto& panel = kv.second;
        delete panel;
    }
}


void MprStatementViewBase::onActivated()
{
    impl->onActivated();
}


void MprStatementViewBase::Impl::onActivated()
{
    auto programView = self->getProgramView();
    
    programViewConnection =
        programView->sigCurrentStatementChanged().connect(
            [=](MprStatement* statement){
                setStatement(programView->currentProgramItem(), statement); });

    setStatement(programView->currentProgramItem(), programView->currentStatement());
}


void MprStatementViewBase::onDeactivated()
{
    impl->programViewConnection.disconnect();
    impl->currentProgramItem = nullptr;
    impl->currentStatement = nullptr;
}


void MprStatementViewBase::registerPanelFactory(const std::type_info& statementType, PanelFactoryFunction factory)
{
    impl->panelFactoryMap[statementType] = factory;
}


MprStatementPanel* MprStatementViewBase::Impl::getOrCreateStatementPanel(MprStatement* statement)
{
    MprStatementPanel* panel = nullptr;

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


void MprStatementViewBase::Impl::setStatement(MprProgramItemBase* programItem, MprStatement* statement)
{
    if(programItem == currentProgramItem && statement == currentStatement){
        return;
    }

    auto prevPanel = dynamic_cast<MprStatementPanel*>(scrollArea.widget());
    if(prevPanel){
        prevPanel->impl->deactivate();
    }

    MprStatementPanel* panel = nullptr;

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


bool MprStatementViewBase::storeState(Archive& archive)
{
    return true;
}


bool MprStatementViewBase::restoreState(const Archive& archive)
{
    return true;
}
