#include "MprStatementPanel.h"
#include "MprProgramItemBase.h"
#include "MprStatement.h"

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
};

}


MprStatementPanel::MprStatementPanel()
{
    impl = new Impl(this);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
}


MprStatementPanel::~MprStatementPanel()
{
    delete impl;
}


MprStatementPanel::Impl::Impl(MprStatementPanel* self)
    : self(self)
{

}


void MprStatementPanel::setEditingEnabled(bool on)
{
    setEnabled(on);
}


void MprStatementPanel::activate
(MprProgramItemBase* programItem, MprStatement* statement)
{
    impl->programItem = programItem;
    impl->statement = statement;

    impl->statementUpdateConnection =
        programItem->program()->sigStatementUpdated().connect(
            [this](MprStatement* updated){
                if(updated == impl->statement){
                    onStatementUpdated();
                }
            });

    onActivated();
}


void MprStatementPanel::deactivate()
{
    onDeactivated();
    
    impl->programItem.reset();
    impl->statement.reset();
    impl->statementUpdateConnection.disconnect();
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
