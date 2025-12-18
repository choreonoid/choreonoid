#include "ButtonGroup.h"

using namespace cnoid;


ButtonGroup::ButtonGroup(QObject* parent)
    : QButtonGroup(parent)
{

}


SignalProxy<void(int id)> ButtonGroup::sigIdClicked()
{
    if(!sigIdClicked_){
        sigIdClicked_.emplace();
        connect(this, (void(QButtonGroup::*)(int))

#if (QT_VERSION >= QT_VERSION_CHECK(5, 15, 0))
                &QButtonGroup::idClicked,
#else
                &QButtonGroup::buttonClicked,
#endif

                [this](int id){ (*sigIdClicked_)(id); });
    }
    return *sigIdClicked_;
}
    

SignalProxy<void(int id, bool checked)> ButtonGroup::sigIdToggled()
{
    if(!sigIdToggled_){
        sigIdToggled_.emplace();
        connect(this, (void(QButtonGroup::*)(int,bool))

#if (QT_VERSION >= QT_VERSION_CHECK(5, 15, 0))
                &QButtonGroup::idToggled,
#else
                &QButtonGroup::buttonToggled,
#endif
                
                [this](int id, bool checked){ (*sigIdToggled_)(id, checked); });
    }
    return *sigIdToggled_;
}
