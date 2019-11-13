/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MAIN_WINDOW_H
#define CNOID_BASE_MAIN_WINDOW_H

#include "Archive.h"
#include <QMainWindow>
#include "exportdecl.h"

namespace cnoid {

class ToolBarArea;
class ViewArea;
class ToolBar;
class MainWindowImpl;
class ExtensionManager;

class CNOID_EXPORT MainWindow : public QMainWindow
{
public:
    static MainWindow* initialize(const char* appName, ExtensionManager* ext);
    static MainWindow* instance();

    ~MainWindow();

    void show();
    void setProjectTitle(const std::string& title);
    ToolBarArea* toolBarArea();
    ViewArea* viewArea();
    void addToolBar(ToolBar* toolbar);
    void removeToolBar(ToolBar* toolbar);
    void getAllToolBars(std::vector<ToolBar*>& out_toolBars);
    void getVisibleToolBars(std::vector<ToolBar*>& out_toolBars);
    void restoreLayout(ArchivePtr archive);
    void storeLayout(ArchivePtr archive);
    void setInitialLayout(ArchivePtr archive);
    void storeWindowStateConfig();

protected:
    virtual void changeEvent(QEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
    virtual void keyPressEvent(QKeyEvent* event);
 
private:
    MainWindowImpl* impl;

    MainWindow(const char* appName, ExtensionManager* ext);

    friend class ExtensionManager;
};

}

#endif
