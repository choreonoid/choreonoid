#include "MprStatementViewBase.h"
#include "MprStatementPanel.h"
#include "MprProgramViewBase.h"
#include "MprProgramItemBase.h"
#include "MprStatement.h"
#include <cnoid/Archive>
#include <fmt/format.h>
#include <QBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <map>
#include <typeindex>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class MprStatementViewBase::Impl
{
public:
    MprStatementViewBase* self;

    MprProgramItemBasePtr currentProgramItem;
    MprStatementPtr currentStatement;
    vector<MprStatementPtr> additionalStatements;
    MprStatementPanel* currentPanel;
    ScopedConnection programViewConnection;
    QScrollArea scrollArea;
    QLabel captionLabel;
    bool needToUpdateCaption;

    map<std::type_index, PanelFactoryFunction> panelFactoryMap;
    map<std::type_index, MprStatementPanel*> panelMap;
    
    Impl(MprStatementViewBase* self);
    ~Impl();
    void onActivated();
    MprStatementPanel* getOrCreateStatementPanel(MprStatement* statement);
    void onSelectedStatementsChanged(
        MprProgramViewBase* programView, const std::vector<MprStatementPtr>& statements);
    void updateAdditionalStatements(const std::vector<MprStatementPtr>& statements);
    void setCaption(const std::string& caption);
};

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

    captionLabel.setFrameStyle(QFrame::Box | QFrame::Sunken);
    vbox->addWidget(&captionLabel);

    scrollArea.setFrameShape(QFrame::NoFrame);
    scrollArea.setStyleSheet("QScrollArea {background: transparent;}");
    scrollArea.setWidgetResizable(true);
    scrollArea.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea.setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    vbox->addWidget(&scrollArea);

    self->setLayout(vbox);

    currentPanel = nullptr;
    needToUpdateCaption = true;
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
        programView->sigSelectedStatementsChanged().connect(
            [=](std::vector<MprStatementPtr>& statements){
                onSelectedStatementsChanged(programView, statements);
            });

    onSelectedStatementsChanged(programView, programView->selectedStatements());
}


void MprStatementViewBase::onDeactivated()
{
    impl->currentProgramItem.reset();
    impl->currentStatement.reset();
    impl->additionalStatements.clear();
    impl->programViewConnection.disconnect();
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


void MprStatementViewBase::Impl::onSelectedStatementsChanged
(MprProgramViewBase* programView, const std::vector<MprStatementPtr>& statements)
{
    auto programItem = programView->currentProgramItem();
    auto statement = programView->currentStatement();
    needToUpdateCaption = true;

    if(currentPanel && programItem == currentProgramItem && statement == currentStatement){
        updateAdditionalStatements(statements);

    } else {
        if(currentPanel){
            currentPanel->deactivate();
        }
        auto prevPanel = currentPanel;
        currentPanel = nullptr;

        if(statement){
            currentPanel = getOrCreateStatementPanel(statement);
            if(currentPanel){
                currentPanel->activate(
                    programItem, statement,
                    [&](const std::string& caption) { setCaption(caption); });
                updateAdditionalStatements(statements);
            }
        }

        if(currentPanel != prevPanel){
            scrollArea.takeWidget();
            if(currentPanel){
                scrollArea.setWidget(currentPanel);
                currentPanel->setAutoFillBackground(false);
            }
        }

        currentProgramItem = programItem;
        currentStatement = statement;
    }

    if(needToUpdateCaption){
        if(statement){
            setCaption(format("<b>{0}</b>", statement->label(0)));
        } else {
            setCaption("---");
        }            
    }
}


void MprStatementViewBase::Impl::updateAdditionalStatements(const std::vector<MprStatementPtr>& statements)
{
    additionalStatements.clear();
    if(statements.size() >= 2){
        for(auto& statement : statements){
            if(statement != currentStatement){
                additionalStatements.push_back(statement);
            }
        }
    }
    currentPanel->onAdditionalStatementsUpdated(additionalStatements);
}


void MprStatementViewBase::Impl::setCaption(const std::string& caption)
{
    captionLabel.setText(caption.c_str());
    needToUpdateCaption = false;
}


bool MprStatementViewBase::storeState(Archive& archive)
{
    return true;
}


bool MprStatementViewBase::restoreState(const Archive& archive)
{
    return true;
}
