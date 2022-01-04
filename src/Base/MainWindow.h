/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MAIN_WINDOW_H
#define CNOID_BASE_MAIN_WINDOW_H

#include "Archive.h"
#include <cnoid/Signal>
#include <QMainWindow>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ToolBarArea;
class ViewArea;
class ToolBar;
class ExtensionManager;

class CNOID_EXPORT MainWindow : public QMainWindow
{
public:
    static MainWindow* initialize(const std::string& appName, ExtensionManager* ext);
    static MainWindow* instance();

    ~MainWindow();

    void show();

    bool isActivatedInWindowSystem() const;
    bool waitForWindowSystemToActivate();
    
    void setProjectTitle(const std::string& title);
    ToolBarArea* toolBarArea();
    ViewArea* viewArea();
    void addToolBar(ToolBar* toolbar);
    void removeToolBar(ToolBar* toolbar);
    std::vector<ToolBar*> toolBars() const;
    std::vector<ToolBar*> visibleToolBars() const;
    
    void restoreLayout(ArchivePtr archive);
    void storeLayout(ArchivePtr archive);
    void setInitialLayout(ArchivePtr archive);
    void resetLayout();
    void storeWindowStateConfig();

    void setFullScreen(bool on);
    void toggleFullScreen();
    SignalProxy<void(bool on)> sigFullScreenToggled();

protected:
    virtual void changeEvent(QEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
    virtual void keyPressEvent(QKeyEvent* event);
 
private:
    class Impl;
    Impl* impl;

    MainWindow(const std::string& appName, ExtensionManager* ext);

    friend class ExtensionManager;
};

}

#endif
