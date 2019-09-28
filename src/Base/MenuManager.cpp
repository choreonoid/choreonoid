/**
   @author Shin'ichiro NAKAOKA
*/

#include "MenuManager.h"
#include "Menu.h"
#include <tuple>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;


MenuManager::MenuManager()
    : topMenu_(0)
{
    currentMenu_ = topMenu_;
    popupMenu_ = 0;
    isBackwardMode = false;
}


MenuManager::MenuManager(QWidget* topMenu)
    : topMenu_(topMenu)
{
    currentMenu_ = topMenu;
    popupMenu_ = 0;
    isBackwardMode = false;
}


void MenuManager::bindTextDomain(const std::string& domain)
{
    textDomain = domain;
}


void MenuManager::setTopMenu(QWidget* topMenu)
{
    if(popupMenu_){
        delete popupMenu_;
        popupMenu_ = 0;
    }
    topMenu_ = topMenu;
    currentMenu_ = topMenu;
}


QWidget* MenuManager::topMenu()
{
    return topMenu_;
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


Menu* MenuManager::popupMenu()
{
    return popupMenu_;
}


QWidget* MenuManager::current() const
{
    return currentMenu_;
}


MenuManager::~MenuManager()
{
    if(popupMenu_){
        delete popupMenu_;
    }
}


int MenuManager::numItems() const
{
    if(topMenu_){
        return topMenu_->actions().size();
    }
    return 0;
}


QAction* MenuManager::findItem(const QString& path)
{
    return findPath(path, false).first;
}


std::pair<QAction*, QWidget*> MenuManager::findPath(const QString& path, bool createPath)
{
    int pos = 0;
    int size = path.size();
    QAction* item = 0;
    QWidget* menu = currentMenu_;
    
    if(path[pos] == QChar('/')){
        pos++;
        menu = topMenu_;
    }

    while(menu && (pos != size)){

        int next = path.indexOf(QChar('/'), pos);
        int length = (next >= 0) ? (next - pos) : next;
        QString name = path.mid(pos, length);

        QList<QAction*> items = menu->actions();
        item = 0;
        for(int i=0; i < items.size(); ++i){
            if(name == items[i]->objectName()){
                item = items[i];
                break;
            }
        }
        if(!item){
            if(createPath){
                if(textDomain.empty()){
                    item = new QAction(name, menu);
                } else {
                    item = new QAction(dgettext(textDomain.c_str(), name.toUtf8()), menu);
                }
                item->setObjectName(name);
                addItem(menu, item);
                item->setMenu(new Menu());
            } else {
                break;
            }
        }

        menu = item->menu();
        pos = (next >= 0) ? (next + 1) : size;
    }

    return make_pair(item, menu);
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
    

/**
   @if jp

   This function specifies a menu to be subjected to item operation.
   The function must be called before adding a menu item.
   
   @param menuPath The path to the target menu
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
    QWidget* menu;

    std::tie(item, menu) = findPath(path, true);

    if(!menu){
        cerr << "cnoid::MenuManager warning: setting path to " << path.toLocal8Bit().data() << " failed." << endl;
    }

    currentMenu_ = menu;
    isBackwardMode = false;

    return *this;
}


MenuManager& MenuManager::setCurrent(QWidget* menu)
{
    currentMenu_ = menu;
    return *this;
}


MenuManager& MenuManager::setBackwardMode()
{
    isBackwardMode = true;
    return *this;
}


Action* MenuManager::addItem(const QString& text)
{
    Action* item = new Action(text, currentMenu_);
    addItem(currentMenu_, item);
    return item;
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
    QAction* separator = new QAction(currentMenu_);
    separator->setSeparator(true);
    addItem(currentMenu_, separator);
    return *this;
}
