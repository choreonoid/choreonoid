#ifndef CNOID_BASE_CAPTURE_BAR_H
#define CNOID_BASE_CAPTURE_BAR_H

#include "ToolBar.h"
#include "MenuManager.h"
#include <cnoid/ValueTree>
#include <optional>
#include <QMouseEvent>
#include <QTabWidget>
#include <functional>

namespace cnoid {

class View;

class CaptureBar : public ToolBar
{
public:
    static void initialize(ExtensionManager* ext);
    static CaptureBar* instance();
    virtual ~CaptureBar();

protected:
    virtual void mouseMoveEvent(QMouseEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual bool eventFilter(QObject* obj, QEvent* event) override;
    
private:
    CaptureBar();
    Mapping* getConfig();
    void onCaptureButtonRightClicked(QMouseEvent* event);    
    void setTabInclusionMode(bool on);
    void captureToolbar(ToolBar* bar);
    void captureView(View* view);
    bool saveWidgetImage(QWidget* widget, const QString& filename);
    bool saveTabViewImage(QTabWidget* tab, View* view, const QString& filename);
    void save(QWidget* widget, std::function<bool(const QString& filename)> saveImage);

    ToolButton* captureButton;
    MenuManager menuManager;
    QWidget* lastCaptureWidget;
    QString lastCaptureFile;
    MappingPtr config;
    std::optional<bool> isTabInclusionMode;
};

}

#endif
