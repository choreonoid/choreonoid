/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_TOOL_BAR_H
#define CNOID_BASE_TOOL_BAR_H

#include "Buttons.h"
#include "Action.h"
#include <QLabel>
#include <QWidget>
#include <QBoxLayout>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class ExtensionManager;
class ToolBarArea;
class MainWindow;

class CNOID_EXPORT ToolBar : public QWidget
{
    Q_OBJECT
public:

    ToolBar(const std::string& name);
    virtual ~ToolBar();

    const std::string& name() const { return name_; }

    ToolButton* addButton(const QString& text, const QString& tooltip = QString());
    ToolButton* addButton(const QIcon& icon, const QString& tooltip = QString());
    ToolButton* addButton(const char* const* xpm, const QString& tooltip = QString());

    ToolButton* addToggleButton(const QString& text, const QString& tooltip = QString());
    ToolButton* addToggleButton(const QIcon& icon, const QString& tooltip = QString());
    ToolButton* addToggleButton(const char* const* xpm, const QString& tooltip = QString());

    void requestNewRadioGroup();
    QButtonGroup* currentRadioGroup();
        
    ToolButton* addRadioButton(const QString& text, const QString& tooltip = QString());
    ToolButton* addRadioButton(const QIcon& icon, const QString& tooltip = QString());
    ToolButton* addRadioButton(const char* const* xpm, const QString& tooltip = QString());

    void addAction(QAction* action);
    void addWidget(QWidget* widget);
    QLabel* addLabel(const QString& text);
    QLabel* addImage(const QString& filename);
    QWidget* addSeparator();
    void addSpacing(int spacing = -1);

    ToolBar& setInsertionPosition(int index);

    void setVisibleByDefault(bool on = true) { isVisibleByDefault_ = on; }
    bool isVisibleByDefault() const { return isVisibleByDefault_; }
    void placeOnNewRowByDefault(bool on = true) { isPlacedOnNewRowByDefault_ = on; }
    bool isPlacedOnNewRowByDefault() const { return isPlacedOnNewRowByDefault_; }
    void setStretchable(bool on) { isStretchable_ = on; }
    bool isStretchable() const { return isStretchable_; }
    virtual int stretchableDefaultWidth() const;
    void setAutoRaiseByDefault(bool on = true) { isAutoRaiseByDefault_ = on; }
    bool isAutoRaiseByDefault() const { return isAutoRaiseByDefault_; }
            
    ToolBarArea* toolBarArea() { return toolBarArea_; }

    class LayoutPriorityCmp {
    public:
        bool operator() (ToolBar* bar1, ToolBar* bar2) {
            return (bar1->layoutPriority < bar2->layoutPriority);
        }
    };
        
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

public Q_SLOTS:
    void setEnabled(bool on);
    void changeIconSize(const QSize& iconSize);

private:
    std::string name_;
    QHBoxLayout* hbox;
    int insertionPosition;
    QWidget* handle;
    QButtonGroup* radioGroup;
    MainWindow* mainWindow;
    ToolBarArea* toolBarArea_;
    bool isNewRadioGroupRequested;
    bool isVisibleByDefault_;
    bool isPlacedOnNewRowByDefault_;
    bool isAutoRaiseByDefault_;
    
    // used for layouting tool bars on a ToolBarArea
    int desiredX;
    int layoutPriority;
    bool isStretchable_;

    void setRadioButton(ToolButton* button);
    void changeIconSizeSub(QLayout* layout, const QSize& iconSize);

    friend class ToolBarArea;
};

}

#endif
