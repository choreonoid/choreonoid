#ifndef CNOID_BASE_POSITION_WIDGET_H
#define CNOID_BASE_POSITION_WIDGET_H

#include <cnoid/EigenTypes>
#include <QWidget>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class MenuManager;
class Archive;

class CNOID_EXPORT PositionWidget : public QWidget
{
public:
    PositionWidget(QWidget* parent);
    ~PositionWidget();

    void setCaption(const std::string& caption);
    void setOptionMenu(MenuManager& menuManager);
    void setPositionCallback(std::function<bool(const Position& T)> callback);
    void clearPosition();
    void refreshPosition();
    void updatePosition(const Position& T);
    void storeState(Archive& archive);
    void restoreState(const Archive& archive);

private:
    class Impl;
    Impl* impl;
};

}

#endif
