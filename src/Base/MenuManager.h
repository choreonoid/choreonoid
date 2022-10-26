/**
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BASE_MENU_MANAGER_H
#define CNOID_BASE_MENU_MANAGER_H

#include "Action.h"
#include "Menu.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MenuManager
{
public:
    MenuManager();
    MenuManager(QWidget* topMenu);
    MenuManager(const MenuManager* org) = delete;
    virtual ~MenuManager();

    void setTopMenu(QWidget* topMenu);
    void setMainMenuBarAsTopMenu();
    void setNewPopupMenu(QWidget* parent = nullptr);
    void bindTextDomain(const std::string& domain);

    QWidget* topMenu() { return topMenu_; }
    Menu* popupMenu() { return popupMenu_; }
    QWidget* current() const { return currentMenu_; }
    Menu* currentMenu() const { return dynamic_cast<Menu*>(currentMenu_); }

    MenuManager& setCurrent(QWidget* menu) {
        currentMenu_ = menu;
        return * this;
    }
    MenuManager& goBackToUpperMenu();

    int numItems() const;

    Action* findItem(const QString& path);
    Action* findItem(const std::string& path);
    Action* findItem(const char* path);
    MenuManager& setPath(const QString& path);
    MenuManager& setPath(const std::string& path);
    MenuManager& setPath(const char* path);
    MenuManager& setBackwardMode();

    Action* addItem(const QString& text);
    Action* addItem(const std::string& text);
    Action* addItem(const char* text);
    Action* addCheckItem(const QString& text);
    Action* addRadioItem(QActionGroup* group, const QString& text);
    void addAction(QAction* action);

    MenuManager& addSeparator();

private:
    QWidget* topMenu_;
    Menu* popupMenu_;
    QWidget* currentMenu_;
    QWidget* lastUpperMenu_;
    bool isBackwardMode;
    std::string textDomain;

    void findPath(
        const QString& path, bool createPath,
        QAction*& out_item, QWidget*& out_menu, QWidget*& out_upperMenu);
    void addItem(QWidget* menu, QAction* item);
};

}

#endif
