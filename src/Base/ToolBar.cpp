#include "ToolBar.h"
#include "ToolBarArea.h"
#include "MainWindow.h"
#include "Separator.h"
#include "Archive.h"
#include <QButtonGroup>
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


ToolBar::ToolBar(const std::string& name)
    : name_(name)
{
    auto topHBox = new QHBoxLayout(this);
    topHBox->setSpacing(4);
    topHBox->setContentsMargins(0, 0, 0, 0);
    setLayout(topHBox);

    handle = new ToolBarHandle(this);
    topHBox->addWidget(handle);

    elementLayout = new QHBoxLayout;
    elementLayout->setSpacing(0);
    elementLayout->setContentsMargins(0, 0, 0, 0);
    topHBox->addLayout(elementLayout);

    insertionPosition = -1;
    radioGroup = nullptr;
    mainWindow = MainWindow::instance();
    toolBarArea_ = nullptr;
    desiredX = 0;
    layoutPriority = 0;
    isStretchable_ = false;
    isNewRadioGroupRequested = true;
    isVisibleByDefault_ = false;
    isPlacedOnNewRowByDefault_ = false;
    isAutoRaiseByDefault_ = true;

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


ToolButton* ToolBar::addButton(int id)
{
    auto button = new ToolButton(this);
    
#ifdef Q_OS_MAC
    // Force auto-raize type
    if(dynamic_cast<QMacStyle*>(button->style())){
        button->setStyle(&cleanlooks);
    }
#endif

    button->setAutoRaise(isAutoRaiseByDefault_);
    button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    elementLayout->insertWidget(insertionPosition, button);

    registerElement(button, id);

    return button;
}


void ToolBar::registerElement(QWidget* element, int id)
{
    if(id >= 0){
        elements.emplace_back(element, id);
    }
    insertionPosition = -1;
}    


ToolButton* ToolBar::addButton(const QString& text, int id)
{
    auto button = addButton(id);
    button->setText(text);
    return button;
}


ToolButton* ToolBar::addButton(const QIcon& icon, int id)
{
    auto button = addButton(id);
    button->setIconSize(mainWindow->iconSize());
    button->setIcon(icon);
    return button;
}


ToolButton* ToolBar::addButton(const char* const* xpm, int id)
{
    return addButton(QIcon(QPixmap(xpm)), id);
}


ToolButton* ToolBar::addButton(const QString& text, const QString& tooltip)
{
    auto button = addButton(text);
    button->setToolTip(tooltip);
    return button;
}


ToolButton* ToolBar::addButton(const QIcon& icon, const QString& tooltip)
{
    auto button = addButton(icon);
    button->setToolTip(tooltip);
    return button;
}


ToolButton* ToolBar::addButton(const char* const* xpm, const QString& tooltip)
{
    auto button = addButton(xpm);
    button->setToolTip(tooltip);
    return button;
}


ToolButton* ToolBar::addToggleButton(const QString& text, int id)
{
    auto button = addButton(text, id);
    button->setCheckable(true);
    return button;
}


ToolButton* ToolBar::addToggleButton(const QIcon& icon, int id)
{
    auto button = addButton(icon, id);
    button->setCheckable(true);
    return button;
}


ToolButton* ToolBar::addToggleButton(const char* const* xpm, int id)
{
    auto button = addButton(xpm, id);
    button->setCheckable(true);
    return button;
}


ToolButton* ToolBar::addToggleButton(const QString& text, const QString& tooltip)
{
    auto button = addToggleButton(text);
    button->setToolTip(tooltip);
    return button;
}


ToolButton* ToolBar::addToggleButton(const QIcon& icon, const QString& tooltip)
{
    auto button = addToggleButton(icon);
    button->setToolTip(tooltip);
    return button;
}


ToolButton* ToolBar::addToggleButton(const char* const* xpm, const QString& tooltip)
{
    auto button = addButton(xpm);
    button->setToolTip(tooltip);
    return button;
}


void ToolBar::requestNewRadioGroup()
{
    radioGroup = nullptr;
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


ToolButton* ToolBar::addRadioButton(const QString& text, int id)
{
    auto button = addButton(text, id);
    setRadioButton(button);
    return button;
}


ToolButton* ToolBar::addRadioButton(const QIcon& icon, int id)
{
    auto button = addButton(icon, id);
    setRadioButton(button);
    return button;
}


ToolButton* ToolBar::addRadioButton(const char* const* xpm, int id)
{
    auto button = addButton(xpm, id);
    setRadioButton(button);
    return button;
}


ToolButton* ToolBar::addRadioButton(const QString& text, const QString& tooltip)
{
    auto button = addRadioButton(text);
    button->setToolTip(tooltip);
    return button;
}


ToolButton* ToolBar::addRadioButton(const QIcon& icon, const QString& tooltip)
{
    auto button = addRadioButton(icon);
    button->setToolTip(tooltip);
    return button;
}


ToolButton* ToolBar::addRadioButton(const char* const* xpm, const QString& tooltip)
{
    auto button = addRadioButton(xpm);
    button->setToolTip(tooltip);
    return button;
}


void ToolBar::addAction(QAction* action, int id)
{
    auto button = addButton(id);
    button->setDefaultAction(action);
}


void ToolBar::addWidget(QWidget* widget, int id)
{
    elementLayout->insertWidget(insertionPosition, widget);
    registerElement(widget, id);
}


QLabel* ToolBar::addLabel(const QString& text, int id)
{
    auto label = new QLabel(text, this);
    elementLayout->insertWidget(insertionPosition, label);
    registerElement(label, id);
    return label;
}


QLabel* ToolBar::addImage(const QString& filename, int id)
{
    auto label = new QLabel(this);
    label->setPixmap(QPixmap(filename));
    elementLayout->insertWidget(insertionPosition, label);
    registerElement(label, id);
    return label;
}


QWidget* ToolBar::addSeparator(int id)
{
    if(separatorExtent < 0){
        separatorExtent = style()->pixelMetric(QStyle::PM_ToolBarSeparatorExtent);
    }
    auto separator = new VSeparator(this);
    separator->setMinimumWidth(separatorExtent);
    elementLayout->insertWidget(insertionPosition, separator);
    registerElement(separator, id);
    return separator;
}


void ToolBar::addSpacing(int spacing, int id)
{
    if(spacing < 0){
        if(separatorExtent < 0){
            separatorExtent = style()->pixelMetric(QStyle::PM_ToolBarSeparatorExtent);
        }
        spacing = separatorExtent;
    }
    auto widget = new QWidget(this);
    widget->setFixedSize(spacing, 1);
    elementLayout->insertWidget(insertionPosition, widget);
    registerElement(widget, id);
}


int ToolBar::elementPosition(int id) const
{
    for(size_t i=0; i < elements.size(); ++i){
        if(elements[i].id == id){
            return i;
        }
    }
    return -1;
}


ToolBar& ToolBar::setInsertionPosition(int index)
{
    insertionPosition = index;
    return *this;
}


void ToolBar::setEnabled(bool on)
{
    int n = elementLayout->count();
    for(int i=0; i < n; ++i){
        QLayoutItem* item = elementLayout->itemAt(i);
        if(auto widget = item->widget()){
            widget->setEnabled(on);
        }
    }
}


void ToolBar::changeIconSize(const QSize& iconSize)
{
    changeIconSizeSub(elementLayout, iconSize);
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


void ToolBar::setActiveElements(const Listing& ids)
{
    QLayoutItem* layoutItem;
    while((layoutItem = elementLayout->takeAt(0)) != 0){
        if(auto widget = layoutItem->widget()){
            widget->hide();
        }
    }
    
    for(int i=0; i < ids.size(); ++i){
        int id = ids[i].toInt();
        auto found = std::find_if(
            elements.begin(), elements.end(),
            [id](const Element& element){ return element.id == id; });
        if(found != elements.end()){
            elementLayout->addWidget(found->widget);
            found->widget->show();
        }
    }

    onActiveElementUpdated();
}


void ToolBar::onActiveElementUpdated()
{

}


bool ToolBar::storeState(Archive& /* archive */)
{
    return true;
}


bool ToolBar::restoreState(const Archive& archive)
{
    auto& ids = *archive.findListing("active_element_ids");
    if(ids.isValid()){
        setActiveElements(ids);
    }
    return true;
}
