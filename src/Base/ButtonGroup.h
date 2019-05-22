/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_BUTTON_GROUP_H
#define CNOID_BASE_BUTTON_GROUP_H

#include <cnoid/Signal>
#include <QButtonGroup>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ButtonGroup : public QButtonGroup
{
    Q_OBJECT

public:
    ButtonGroup(QObject* parent = 0);

    SignalProxy<void(int id)> sigButtonClicked();
    SignalProxy<void(int id, bool checked)> sigButtonToggled();

private Q_SLOTS:
    void onButtonClicked(int id);
    void onButtonToggled(int id, bool checked);

private:
    Signal<void(int id)> sigButtonClicked_;
    Signal<void(int id, bool checked)> sigButtonToggled_;
    bool sigButtonClickedConnected;
    bool sigButtonToggledConnected;
};

}

#endif
