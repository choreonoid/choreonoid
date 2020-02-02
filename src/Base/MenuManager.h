/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_MENU_MANAGER_H
#define CNOID_BASE_MENU_MANAGER_H

#include "Action.h"
#include "Menu.h"
#include <cnoid/ExtensionManager>
#include "exportdecl.h"

namespace cnoid {

class Menu;
class MenuManagerImpl;

class CNOID_EXPORT MenuManager
{
public:
    MenuManager();
    MenuManager(QWidget* topMenu);
    virtual ~MenuManager();

    void bindTextDomain(const std::string& domain);

    void setTopMenu(QWidget* topMenu);
    QWidget* topMenu();

    void setNewPopupMenu(QWidget* parent = 0);
    Menu* popupMenu();

    QWidget* current() const;
    MenuManager& setCurrent(QWidget* menu);

    int numItems() const;

    QAction* findItem(const QString& path);
    MenuManager& setPath(const QString& path);
        
    MenuManager& setBackwardMode();

    void addAction(QAction* action);

    Action* addItem(const QString& text);
    Action* addItem(const std::string& text);
    Action* addItem(const char* text);
    Action* addCheckItem(const QString& text);
    Action* addRadioItem(QActionGroup* group, const QString& text);

    MenuManager& addSeparator();

private:
    MenuManager(const MenuManager* org);
    QWidget* topMenu_;
    QWidget* currentMenu_;
    Menu* popupMenu_;
    bool isBackwardMode;
    std::string textDomain;

    std::pair<QAction*, QWidget*> findPath(const QString& path, bool createPath);
    void addItem(QWidget* menu, QAction* item);
};

}

#endif
