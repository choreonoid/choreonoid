#include "MprStatementPanel.h"
#include "MprProgramItemBase.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class MprStatementPanel::Impl
{
public:
    MprStatementPanel* self;
    MprProgramItemBasePtr programItem;
    MprStatementPtr statement;
    function<void(const std::string& caption)> setCaption;
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
(MprProgramItemBase* programItem, MprStatement* statement,
 std::function<void(const std::string& caption)> setCaption)
{
    impl->programItem = programItem;
    impl->statement = statement;
    impl->setCaption = setCaption;

    impl->statementUpdateConnection =
        programItem->program()->sigStatementUpdated().connect(
            [this](MprStatement* updated){
                if(updated == impl->statement){
                    onStatementUpdated();
                }
            });

    setEditingEnabled(statement->holderProgram()->isEditingEnabled());

    onActivated();
}


void MprStatementPanel::deactivate()
{
    onDeactivated();

    impl->programItem.reset();
    impl->statement.reset();
    impl->setCaption = nullptr;
    impl->statementUpdateConnection.disconnect();
}


void MprStatementPanel::onStatementUpdated()
{

}


void MprStatementPanel::onAdditionalStatementsUpdated
(const std::vector<MprStatementPtr>& /* additionalStatements */)
{

}


void MprStatementPanel::onDeactivated()
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


void MprStatementPanel::setCaption(const std::string& caption)
{
    if(impl->setCaption){
        impl->setCaption(caption);
    }
}

