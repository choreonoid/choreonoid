/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_BUTTON_GROUP_H_INCLUDED
#define CNOID_GUIBASE_BUTTON_GROUP_H_INCLUDED

#include <cnoid/SignalProxy>
#include <QButtonGroup>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ButtonGroup : public QButtonGroup
{
    Q_OBJECT

        public:
    ButtonGroup(QObject* parent = 0);

    inline SignalProxy< boost::signal<void(int id)> > sigButtonClicked() {
        return sigButtonClicked_;
    }

private Q_SLOTS:
    void onButtonClicked(int id);

private:
    boost::signal<void(int id)> sigButtonClicked_;
};
}

#endif
