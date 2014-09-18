/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MAIN_WINDOW_H
#define CNOID_BASE_MAIN_WINDOW_H

#include "Archive.h"
#include <QMainWindow>
#include "exportdecl.h"

namespace cnoid {

class ViewArea;
class ToolBar;
class MainWindowImpl;
class ExtensionManager;

class CNOID_EXPORT MainWindow : public QMainWindow
{
public:
    static void initialize(const char* appName, ExtensionManager* ext);
    static MainWindow* instance();

    void show();
    void setProjectTitle(const std::string& title);
    ViewArea* viewArea();
    void addToolBar(ToolBar* toolbar);
    void getAllToolBars(std::vector<ToolBar*>& out_toolBars);
    void getVisibleToolBars(std::vector<ToolBar*>& out_toolBars);
    void storeLayout(ArchivePtr archive);
    void restoreLayout(ArchivePtr archive);
    void setInitialLayout(ArchivePtr archive);

protected:
    virtual void changeEvent(QEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
    virtual void keyPressEvent(QKeyEvent* event);
 
private:
    MainWindowImpl* impl;

    MainWindow(const char* appName, ExtensionManager* ext);
    virtual ~MainWindow();

    void storeWindowStateConfig();

    friend class AppImpl;
    friend class ExtensionManager;
};

}

#endif
