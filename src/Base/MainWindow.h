#ifndef CNOID_BASE_MAIN_WINDOW_H
#define CNOID_BASE_MAIN_WINDOW_H

#include <cnoid/Signal>
#include <QMainWindow>
#include <QKeyEvent>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ToolBarArea;
class ViewArea;
class ToolBar;
class ExtensionManager;
class Archive;

class CNOID_EXPORT MainWindow : public QMainWindow
{
public:
    //! \note This function can be called before the initialize function is called.
    static void setLayoutSwitcherAvailable(bool on);
    
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
    
    void restoreLayout(Archive* archive);
    void storeLayout(Archive* archive);
    void setInitialLayout(Archive* archive);
    void resetLayout();
    void storeWindowStateConfig();

    void setFullScreen(bool on);
    void toggleFullScreen();
    SignalProxy<void(bool on)> sigFullScreenToggled();

    SignalProxy<void(QKeyEvent* event)> sigKeyPressed();
    SignalProxy<void(QKeyEvent* event)> sigKeyReleased();

protected:
    virtual void changeEvent(QEvent* event) override;
    virtual void resizeEvent(QResizeEvent* event) override;
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void keyReleaseEvent(QKeyEvent* event) override;
 
private:
    class Impl;
    Impl* impl;

    MainWindow(const std::string& appName, ExtensionManager* ext);

    friend class ExtensionManager;
};

}

#endif
