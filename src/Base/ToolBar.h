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
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class Listing;
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

    ToolButton* addButton(const QString& text, int id = -1);
    ToolButton* addButton(const QIcon& icon, int id = -1);
    ToolButton* addButton(const char* const* xpm, int id = -1);

    ToolButton* addToggleButton(const QString& text, int id = -1);
    ToolButton* addToggleButton(const QIcon& icon, int id = -1);
    ToolButton* addToggleButton(const char* const* xpm, int id = -1);

    ToolButton* addRadioButton(const QString& text, int id = -1);
    ToolButton* addRadioButton(const QIcon& icon, int id = -1);
    ToolButton* addRadioButton(const char* const* xpm, int id = -1);
    void requestNewRadioGroup();
    QButtonGroup* currentRadioGroup();
        
    void addAction(QAction* action, int id = -1);
    void addWidget(QWidget* widget, int id = -1);
    QLabel* addLabel(const QString& text, int id = -1);
    QLabel* addImage(const QString& filename, int id = -1);
    QWidget* addSeparator(int id = -1);
    void addSpacing(int spacing = -1, int id = -1);

    int numElements() const { return elements.size(); }
    int elementPosition(int id) const;
    ToolBar& setInsertionPosition(int position);

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

    [[deprecated]]
    ToolButton* addButton(const QString& text, const QString& tooltip);
    [[deprecated]]
    ToolButton* addButton(const QIcon& icon, const QString& tooltip);
    [[deprecated]]
    ToolButton* addButton(const char* const* xpm, const QString& tooltip);
    [[deprecated]]
    ToolButton* addToggleButton(const QString& text, const QString& tooltip);
    [[deprecated]]
    ToolButton* addToggleButton(const QIcon& icon, const QString& tooltip);
    [[deprecated]]
    ToolButton* addToggleButton(const char* const* xpm, const QString& tooltip);
    [[deprecated]]
    ToolButton* addRadioButton(const QString& text, const QString& tooltip);
    [[deprecated]]
    ToolButton* addRadioButton(const QIcon& icon, const QString& tooltip);
    [[deprecated]]
    ToolButton* addRadioButton(const char* const* xpm, const QString& tooltip);

public Q_SLOTS:
    void setEnabled(bool on);
    void changeIconSize(const QSize& iconSize);

private:
    std::string name_;
    QHBoxLayout* elementLayout;
    QWidget* handle;
    int insertionPosition;
    int lastId;
    QButtonGroup* radioGroup;
    MainWindow* mainWindow;
    ToolBarArea* toolBarArea_;

    // used for layouting tool bars on a ToolBarArea
    int desiredX;
    int layoutPriority;
    bool isStretchable_;

    bool isNewRadioGroupRequested;
    bool isVisibleByDefault_;
    bool isPlacedOnNewRowByDefault_;
    bool isAutoRaiseByDefault_;

    struct Element {
        QWidget* widget;
        int id;
        Element(QWidget* widget, int id) : widget(widget), id(id) { }
    };
    std::vector<Element> elements;

    ToolButton* addButton(int id);
    void registerElement(QWidget* element, int id);    
    void setRadioButton(ToolButton* button);
    void changeIconSizeSub(QLayout* layout, const QSize& iconSize);
    void setActiveElements(const Listing& ids);
    
    friend class ToolBarArea;
};

}

#endif
