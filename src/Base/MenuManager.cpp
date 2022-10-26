/**
   @author Shin'ichiro Nakaoka
*/

#include "MenuManager.h"
#include "MainWindow.h"
#include "MessageView.h"
#include <QMenuBar>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;


MenuManager::MenuManager()
    : MenuManager(static_cast<QWidget*>(nullptr))
{

}


MenuManager::MenuManager(QWidget* topMenu)
{
    topMenu_ = topMenu;
    popupMenu_ = nullptr;
    currentMenu_ = topMenu;
    lastUpperMenu_ = nullptr;
    isBackwardMode = false;
}


MenuManager::~MenuManager()
{
    if(popupMenu_){
        delete popupMenu_;
    }
}


void MenuManager::setTopMenu(QWidget* topMenu)
{
    if(popupMenu_){
        delete popupMenu_;
        popupMenu_ = nullptr;
    }
    topMenu_ = topMenu;
    currentMenu_ = topMenu;
}


void MenuManager::setMainMenuBarAsTopMenu()
{
    setTopMenu(MainWindow::instance()->menuBar());
}


void MenuManager::setNewPopupMenu(QWidget* parent)
{
    if(popupMenu_){
        delete popupMenu_;
    }
    popupMenu_ = new Menu(parent);
    topMenu_ = popupMenu_;
    currentMenu_ = popupMenu_;
}


void MenuManager::bindTextDomain(const std::string& domain)
{
    textDomain = domain;
}


MenuManager& MenuManager::goBackToUpperMenu()
{
    if(lastUpperMenu_){
        currentMenu_ = lastUpperMenu_;
        lastUpperMenu_ = nullptr;

    } else if(currentMenu_){
        if(auto upper = currentMenu_->parentWidget()){
            if(auto upperMenu = dynamic_cast<QMenu*>(upper)){
                currentMenu_ = upperMenu;
            } else if(auto upperMenuBar = dynamic_cast<QMenuBar*>(upper)){
                currentMenu_ = upperMenuBar;
            }
        }
    }

    return *this;
}


int MenuManager::numItems() const
{
    if(topMenu_){
        return topMenu_->actions().size();
    }
    return 0;
}


Action* MenuManager::findItem(const QString& path)
{
    QAction* item;
    QWidget* menu;
    QWidget* upperMenu;
    findPath(path, false, item, menu, upperMenu);
    return dynamic_cast<Action*>(item);
}


Action* MenuManager::findItem(const std::string& path)
{
    return findItem(QString(path.c_str()));
}


Action* MenuManager::findItem(const char* path)
{
    return findItem(QString(path));
}


void MenuManager::findPath
(const QString& path, bool createPath, QAction*& out_item, QWidget*& out_menu, QWidget*& out_upperMenu)
{
    out_item = nullptr;
    out_upperMenu = nullptr;

    int pos = 0;
    int size = path.size();
    QAction* item = nullptr;
    QWidget* menu = currentMenu_;
        
    if(path[pos] == QChar('/')){
        ++pos;
        menu = topMenu_;
    }

    while(menu && (pos != size)){

        int next = path.indexOf(QChar('/'), pos);
        int length = (next >= 0) ? (next - pos) : next;
        QString targetName = path.mid(pos, length);
        QString name;
        item = nullptr;
        
        for(auto& action : menu->actions()){
            name = action->objectName();
            if(name.isEmpty()){
                name = action->text();
            }
            if(name == targetName){
                item = action;
                break;
            }
        }
        if(!item){
            if(!createPath){
                break;
            }
            if(textDomain.empty()){
                item = new Action(targetName, menu);
            } else {
                item = new Action(dgettext(textDomain.c_str(), targetName.toUtf8()), menu);
            }
            item->setObjectName(targetName);
            addItem(menu, item);
            item->setMenu(new Menu);
        }

        auto itemMenu = item->menu();
        if(itemMenu){
            out_upperMenu = menu;
        }
        menu = itemMenu;
        
        pos = (next >= 0) ? (next + 1) : size;
    }

    out_item = item;
    out_menu = menu;
}


/**
   This function specifies a menu to be subjected to item operation.
   The function must be called before adding a menu item.
   
   @param path The path to the target menu
   If the path begins with '/', it will be the path from the root.
   Otherwise, it will be a relative path from the currently specifed menu.
   A menu is newly created if the menu specified in the path does not exist.
*/
MenuManager& MenuManager::setPath(const QString& path)
{
    if(!path.isEmpty() && path[0] == QChar('/')){
        isBackwardMode = false;
    }
    
    QAction* item;
    findPath(path, true, item, currentMenu_, lastUpperMenu_);

    if(!currentMenu_){
        MessageView::instance()->putln(
            fmt::format(_("MenuManager failed to set the current menu path to {0}."), path.toStdString()),
            MessageView::Error);
    }

    isBackwardMode = false;

    return *this;
}


MenuManager& MenuManager::setPath(const std::string& path)
{
    return setPath(QString(path.c_str()));
}


MenuManager& MenuManager::setPath(const char* path)
{
    return setPath(QString(path));
}


MenuManager& MenuManager::setBackwardMode()
{
    isBackwardMode = true;
    return *this;
}


void MenuManager::addItem(QWidget* menu, QAction* item)
{
    QList<QAction*> items = menu->actions();
    
    int position;
    
    for(position = items.size() - 1; position >= 0; --position){
        QAction* sibling = items[position];
        if(!sibling->property("isBackward").toBool()){
            break;
        }
    }
    position++;
    
    if(position < items.size()){
        menu->insertAction(items[position], item);
    } else {
        menu->addAction(item);
    }

    if(isBackwardMode){
        item->setProperty("isBackward", true);
    }
}
    

Action* MenuManager::addItem(const QString& text)
{
    Action* item = new Action(text, currentMenu_);
    addItem(currentMenu_, item);
    return item;
}


Action* MenuManager::addItem(const std::string& text)
{
    return addItem(QString(text.c_str()));
}


Action* MenuManager::addItem(const char* text)
{
    return addItem(QString(text));
}


Action* MenuManager::addCheckItem(const QString& text)
{
    Action* item = addItem(text);
    if(item){
        item->setCheckable(true);
    }
    return item;
}


Action* MenuManager::addRadioItem(QActionGroup* group, const QString& text)
{
    Action* item = addItem(text);
    if(item){
        item->setCheckable(true);
        item->setActionGroup(group);
    }
    return item;
}


void MenuManager::addAction(QAction* action)
{
    addItem(currentMenu_, action);
}


MenuManager& MenuManager::addSeparator()
{
    Action* separator = new Action(currentMenu_);
    separator->setSeparator(true);
    addItem(currentMenu_, separator);
    return *this;
}
