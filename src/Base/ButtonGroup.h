#ifndef CNOID_BASE_BUTTON_GROUP_H
#define CNOID_BASE_BUTTON_GROUP_H

#include <cnoid/Signal>
#include <QButtonGroup>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ButtonGroup : public QButtonGroup
{
public:
    ButtonGroup(QObject* parent = nullptr);

    SignalProxy<void(int id)> sigIdClicked();
    SignalProxy<void(int id, bool checked)> sigIdToggled();
    //[[deprecated]]
    SignalProxy<void(int id)> sigButtonClicked() { return sigIdClicked(); }
    //[[deprecated]]
    SignalProxy<void(int id, bool checked)> sigButtonToggled() { return sigIdToggled(); }

private:
    stdx::optional<Signal<void(int id)>> sigIdClicked_;
    stdx::optional<Signal<void(int id, bool checked)>> sigIdToggled_;
};

}

#endif
