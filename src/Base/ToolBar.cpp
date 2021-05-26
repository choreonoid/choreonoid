/**
   @author Shin'ichiro Nakaoka
*/

#include "ToolBar.h"
#include "ToolBarArea.h"
#include "MainWindow.h"
#include "Separator.h"
#include <QStylePainter>
#include <QStyleOptionToolBar>
#include <QMouseEvent>

#ifdef Q_OS_MAC
#include <QCleanlooksStyle>
#include <QMacStyle>
#endif

using namespace std;
using namespace cnoid;


namespace {

#ifdef Q_OS_MAC
QCleanlooksStyle cleanlooks;
#endif

int separatorExtent = -1;

class ToolBarHandle : public QWidget
{
    ToolBar* toolBar;
    bool isDragging;
    QPoint dragOrgLocalPos;
        
public:
    ToolBarHandle(ToolBar* toolBar)
        : QWidget(toolBar),
          toolBar(toolBar) {

        setFixedWidth(12);
        isDragging = false;
        setCursor(Qt::OpenHandCursor);
    }

    virtual void paintEvent(QPaintEvent*) {

        QStylePainter painter(this);

        QStyleOptionToolBar option;
        option.initFrom(this);
        option.features = QStyleOptionToolBar::Movable;
        option.toolBarArea = Qt::TopToolBarArea;
        option.state = QStyle::State_Horizontal;
        option.rect.setHeight(height());
        option.rect.setWidth(16);

        painter.drawPrimitive(QStyle::PE_IndicatorToolBarHandle, option);
    }

    virtual void mousePressEvent(QMouseEvent* event) {
        if(event->button() == Qt::LeftButton){
            dragOrgLocalPos = event->pos();
            setCursor(Qt::ClosedHandCursor);
            isDragging = true;
        }
    }

    virtual void mouseMoveEvent(QMouseEvent* event) {
        if(isDragging){
            toolBar->toolBarArea()->dragToolBar(toolBar, event->globalPos() - dragOrgLocalPos);
        }
    }

    virtual void mouseReleaseEvent(QMouseEvent*) {
        setCursor(Qt::OpenHandCursor);
        isDragging = false;
    }
};

}


ToolBar::ToolBar(const QString& name)
{
    setWindowTitle(name);
    setObjectName(name);

    mainWindow = MainWindow::instance();

    hbox = new QHBoxLayout(this);
    hbox->setSpacing(0);
    hbox->setContentsMargins(0, 0, 0, 0);
    setLayout(hbox);
    insertionPosition = -1;

    handle = new ToolBarHandle(this);
    hbox->addWidget(handle);
    hbox->addSpacing(4);
    
    radioGroup = 0;
    isNewRadioGroupRequested = true;

    toolBarArea_ = 0;

    isVisibleByDefault_ = false;
    isPlacedOnNewRowByDefault_ = false;
    isAutoRaiseByDefault_ = true;
    
    desiredX = 0;
    layoutPriority = 0;
    isStretchable_ = false;

    connect(mainWindow, SIGNAL(iconSizeChanged(const QSize&)),
            this, SLOT(changeIconSize(const QSize&)));
}


ToolBar::~ToolBar()
{

}


int ToolBar::stretchableDefaultWidth() const
{
    return QWidget::sizeHint().width();
}


ToolButton* ToolBar::addButton(const QString& text, const QString& tooltip)
{
    ToolButton* button = new ToolButton(this);
#ifdef Q_OS_MAC
    // Force auto-raize type
    if(dynamic_cast<QMacStyle*>(button->style())){
        button->setStyle(&cleanlooks);
    }
#endif
    button->setText(text);
    button->setAutoRaise(isAutoRaiseByDefault_);
    button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    if(!tooltip.isEmpty()){
        button->setToolTip(tooltip);
    }
    hbox->insertWidget(insertionPosition, button);
    insertionPosition = -1;
    return button;
}


ToolButton* ToolBar::addButton(const QIcon& icon, const QString& tooltip)
{
    ToolButton* button = new ToolButton(this);
#ifdef Q_OS_MAC
    // Force auto-raize type
    if(dynamic_cast<QMacStyle*>(button->style())){
        button->setStyle(&cleanlooks);
    }
#endif
    button->setIconSize(mainWindow->iconSize());
    button->setIcon(icon);
    button->setAutoRaise(isAutoRaiseByDefault_);
    button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    if(!tooltip.isEmpty()){
        button->setToolTip(tooltip);
    }
    hbox->insertWidget(insertionPosition, button);
    insertionPosition = -1;
    return button;
}


ToolButton* ToolBar::addButton(const char* const* xpm, const QString& tooltip)
{
    return addButton(QIcon(QPixmap(xpm)), tooltip);
}


ToolButton* ToolBar::addToggleButton(const QString& text, const QString& tooltip)
{
    ToolButton* button = addButton(text, tooltip);
    button->setCheckable(true);
    return button;
}


ToolButton* ToolBar::addToggleButton(const QIcon& icon, const QString& tooltip)
{
    ToolButton* button = addButton(icon, tooltip);
    button->setCheckable(true);
    return button;
}


ToolButton* ToolBar::addToggleButton(const char* const* xpm, const QString& tooltip)
{
    ToolButton* button = addButton(xpm, tooltip);
    button->setCheckable(true);
    return button;
}


void ToolBar::requestNewRadioGroup()
{
    radioGroup = 0;
    isNewRadioGroupRequested = true;
}


QButtonGroup* ToolBar::currentRadioGroup()
{
    if(isNewRadioGroupRequested){
        radioGroup = new QButtonGroup(this);
        isNewRadioGroupRequested = false;
    }
    return radioGroup;
}


void ToolBar::setRadioButton(ToolButton* button)
{
    button->setCheckable(true);
    currentRadioGroup()->addButton(button);
}


ToolButton* ToolBar::addRadioButton(const QString& text, const QString& tooltip)
{
    ToolButton* button = addButton(text, tooltip);
    setRadioButton(button);
    return button;
}


ToolButton* ToolBar::addRadioButton(const QIcon& icon, const QString& tooltip)
{
    ToolButton* button = addButton(icon, tooltip);
    setRadioButton(button);
    return button;
}


ToolButton* ToolBar::addRadioButton(const char* const* xpm, const QString& tooltip)
{
    ToolButton* button = addButton(xpm, tooltip);
    setRadioButton(button);
    return button;
}


void ToolBar::addAction(QAction* action)
{
    ToolButton* button = new ToolButton(this);
    button->setAutoRaise(isAutoRaiseByDefault_);
    button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    button->setDefaultAction(action);
    hbox->insertWidget(insertionPosition, button);
    insertionPosition = -1;
}


void ToolBar::addWidget(QWidget* widget)
{
    hbox->insertWidget(insertionPosition, widget);
    insertionPosition = -1;
}


QLabel* ToolBar::addLabel(const QString& text)
{
    QLabel* label = new QLabel(text, this);
    hbox->insertWidget(insertionPosition, label);
    insertionPosition = -1;
    return label;
}



QLabel* ToolBar::addImage(const QString& filename)
{
    QLabel* label = new QLabel(this);
    label->setPixmap(QPixmap(filename));
    hbox->insertWidget(insertionPosition, label);
    insertionPosition = -1;
    return label;
}


QWidget* ToolBar::addSeparator()
{
    if(separatorExtent < 0){
        separatorExtent = style()->pixelMetric(QStyle::PM_ToolBarSeparatorExtent);
    }
    auto sep = new VSeparator(this);
    sep->setMinimumWidth(separatorExtent);
    hbox->insertWidget(insertionPosition, sep);
    insertionPosition = -1;
    return sep;
}


void ToolBar::addSpacing(int spacing)
{
    if(spacing < 0){
        if(separatorExtent < 0){
            separatorExtent = style()->pixelMetric(QStyle::PM_ToolBarSeparatorExtent);
        }
        spacing = separatorExtent;
    }
    hbox->insertSpacing(insertionPosition, spacing);
    insertionPosition = -1;
}


ToolBar& ToolBar::setInsertionPosition(int index)
{
    // The handle and a spacing exist before the first tool bar item
    insertionPosition = index + 2;
    return *this;
}


void ToolBar::setEnabled(bool on)
{
    /*
      if(on){
      QWidget::setEnabled(on);
      } else {
      int n = hbox->count();
      for(int i=0; i < n; ++i){
      QLayoutItem* item = hbox->itemAt(i);
      QWidget* widget = item->widget();
      if(widget){
      widget->setEnabled(false);
      }
      handle->setEnabled(true);
      }
      }
    */
    int n = hbox->count();
    for(int i=0; i < n; ++i){
        QLayoutItem* item = hbox->itemAt(i);
        QWidget* widget = item->widget();
        if(widget){
            widget->setEnabled(on);
        }
        handle->setEnabled(true);
    }
}


void ToolBar::changeIconSize(const QSize& iconSize)
{
    changeIconSizeSub(hbox, iconSize);
}


void ToolBar::changeIconSizeSub(QLayout* layout, const QSize& iconSize)
{
    int n = layout->count();
    for(int i=0; i < n; ++i){
        QLayoutItem* item = layout->itemAt(i);
        QLayout * childLayout = dynamic_cast<QLayout*>(item);
        if(childLayout){
            changeIconSizeSub(childLayout, iconSize);
        }
        QWidgetItem* widgetItem = dynamic_cast<QWidgetItem*>(item);
        if(widgetItem){
            QWidget* widget = widgetItem->widget();
            ToolButton* button = dynamic_cast<ToolButton*>(widget);
            if(button){
                button->setIconSize(mainWindow->iconSize());
            }
        }
    }
}


bool ToolBar::storeState(Archive& /* archive */)
{
    return true;
}


bool ToolBar::restoreState(const Archive& /* archive */)
{
    return true;
}
