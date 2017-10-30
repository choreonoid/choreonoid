/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "GrxUIMenuView.h"
#include <cnoid/ViewManager>
#include <cnoid/MainWindow>
#include <cnoid/MessageView>
#include <cnoid/Dialog>
#include <cnoid/Buttons>
#include <cnoid/CheckBox>
#include <cnoid/LineEdit>
#include <cnoid/PyUtil>
#include <cnoid/PythonExecutor>
#include <QBoxLayout>
#include <QStackedLayout>
#include <QScrollArea>
#include <QFrame>
#include <QLabel>
#include <QMessageBox>
#include <QEventLoop>
#include "gettext.h"

using namespace std;
namespace stdph = std::placeholders;
using namespace cnoid;

namespace {

python::object cancelExceptionType;

class FuncParamEntry : public LineEdit
{
public:
    int typeChar;
    std::string marker;

    FuncParamEntry(int typeChar, const std::string& marker)
        : typeChar(typeChar),
          marker(marker)
    {

    }
        
    virtual QSize sizeHint() const
    {
        QString sizeText;
        if(typeChar == 'D'){
            sizeText = "123.56";
        } else if(typeChar == 'T'){
            sizeText = "Text Input";
        }
        QSize s = LineEdit::sizeHint();
        return QSize(fontMetrics().boundingRect(sizeText).width(), s.height());
    }

    std::string text() const {
        return LineEdit::text().toStdString();
    }
};
        

class FuncButtonBox : public QWidget
{
public:
    PushButton button;
    string funcString;
    vector<FuncParamEntry*> entries;
        
    FuncButtonBox(const string& label, const string& funcString)
    {
        QHBoxLayout* hbox = new QHBoxLayout();
        setLayout(hbox);

        button.setText(label.c_str());
        hbox->addWidget(&button);
        
        size_t pos = 0;
        while(true){
            pos = funcString.find('#', pos);
            if(pos == std::string::npos){
                break;
            }
            if((pos + 1) == funcString.size()){
                break;
            }
            string marker = funcString.substr(pos, 2);
            ++pos;
            int typeChar = toupper(funcString[pos]);
            
            if(typeChar == 'D' || typeChar == 'T'){
                FuncParamEntry* entry = new FuncParamEntry(typeChar, marker);
                hbox->addWidget(entry);
                entries.push_back(entry);
            } else {
                break;
            }
        }
        
        this->funcString = funcString;
    }

    string label() const {
        return button.text().toStdString();
    }
};


class MenuWidget : public QWidget
{
public:
    GrxUIMenuViewImpl* view;

    QStackedLayout pageStack;
    QStackedLayout barStack;

    ToolButton sequenceModeButton;
    ToolButton terminateButton1;
        
    ToolButton regularModeButton;
    ToolButton prevPageButton;
    QLabel pageIndexLabel;
    ToolButton nextPageButton;
    CheckBox sequentialCheck;
    ToolButton retryButton;
    ToolButton terminateButton2;

    bool isLocalSequentialMode;
    int currentSequencePageIndex;
    int currentButtonPage;
    int currentButton;
    int lastIndexInPage;
    vector<vector<QWidget*> > buttonList;
    PythonExecutor pythonExecutor;

    MenuWidget(const python::list& menu, bool isLocalSequentialMode, bool doBackgroundExecution, GrxUIMenuViewImpl* view);
    void createPages(const python::list& menu);
    void addSection(const python::list& section);
    void onButtonClicked(int indexInPage, FuncButtonBox* box);
    void onScriptFinished();
    void moveNext();
    void moveToNextButton();
    bool isSequenceMode();
    void showSequencePages();
    void showRegularPage();
    void moveSequentialPage(int direction);
    void setCurrentSequentialPage(int index);
    void onSequentialCheckToggled(bool on);
    void onRetryClicked();
    void onTerminateClicked();
    void onQuitClicked();
};

}

namespace cnoid {

class GrxUIMenuViewImpl
{
public:
    GrxUIMenuView* self;
    QVBoxLayout vbox;
    QLabel noMenuLabel;
    MenuWidget* menuWidget;
    int menuButtonsBlockCount;

    GrxUIMenuViewImpl(GrxUIMenuView* self);
    ~GrxUIMenuViewImpl();
    void clearMenu(bool doDelete, bool showLabel);
    void setMenu(const python::list& menu, bool isLocalSequentialMode, bool doBackgroundExecution);
    void blockMenuButtons(bool on);
};

}


namespace {

struct MenuButtonBlock {
    GrxUIMenuViewImpl* impl;
    MenuButtonBlock(GrxUIMenuViewImpl* impl) : impl(impl) {
        impl->blockMenuButtons(true);
    }
    ~MenuButtonBlock(){
        impl->blockMenuButtons(false);
    }
};
}


void GrxUIMenuView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<GrxUIMenuView>(
        "GrxUIMenuView", N_("GrxUI Menu"), ViewManager::SINGLE_OPTIONAL);
}


GrxUIMenuView* GrxUIMenuView::instance()
{
    return ViewManager::getOrCreateView<GrxUIMenuView>();
}


void GrxUIMenuView::setCancelExceptionType(python::object exceptionType)
{
    cancelExceptionType = exceptionType;
}


GrxUIMenuView::GrxUIMenuView()
{
    impl = new GrxUIMenuViewImpl(this);
}


GrxUIMenuViewImpl::GrxUIMenuViewImpl(GrxUIMenuView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::CENTER);

    self->setLayout(&vbox);
    noMenuLabel.setText(_("There is no menu now."));
    noMenuLabel.setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    vbox.addWidget(&noMenuLabel, 0, Qt::AlignHCenter);
    
    menuWidget = 0;
    menuButtonsBlockCount = 0;

    clearMenu(false, true);
}


GrxUIMenuView::~GrxUIMenuView()
{
    delete impl;
}


GrxUIMenuViewImpl::~GrxUIMenuViewImpl()
{

}


void GrxUIMenuViewImpl::clearMenu(bool doDelete, bool showLabel)
{
    if(menuWidget){
        vbox.removeWidget(menuWidget);
        if(doDelete){
            delete menuWidget;
        }
        menuWidget = 0;
    }

    if(showLabel){
        noMenuLabel.show();
    } else {
        noMenuLabel.hide();
    }
}


void GrxUIMenuView::setMenu(const python::list& menu, bool isLocalSequentialMode, bool doBackgroundExecution)
{
    impl->setMenu(menu, isLocalSequentialMode, doBackgroundExecution);
}


void GrxUIMenuViewImpl::setMenu(const python::list& menu, bool isLocalSequentialMode, bool doBackgroundExecution)
{
    clearMenu(true, false);

    menuWidget = new MenuWidget(menu, isLocalSequentialMode, doBackgroundExecution, this);
    vbox.addWidget(menuWidget);
    noMenuLabel.hide();

    MessageView::instance()->notify(_("A new menu has been set to the GrxUI Menu View."));
}


void GrxUIMenuViewImpl::blockMenuButtons(bool on)
{
    if(on){
        ++menuButtonsBlockCount;
    } else{
        menuButtonsBlockCount = std::max(0, menuButtonsBlockCount - 1);
    }
    if(menuWidget){
        bool enabled = (menuButtonsBlockCount <= 0);
        QStackedLayout& pageStack = menuWidget->pageStack;
        const int n = pageStack.count();
        for(int i=0; i < n; ++i){
            pageStack.widget(i)->setEnabled(enabled);
        }
    }
}


/*
  This function is defined here instead of PyGrxUI.cpp so that the message translations can be edited with one file
*/
QMessageBox::StandardButton GrxUIMenuView::waitInputSelect(const std::string& message)
{
    QMessageBox box(MainWindow::instance());
    box.setWindowTitle(_("Wait input select"));
    box.setText(message.c_str());
    box.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Yes);
    box.setModal(false);
    box.show();

    MenuButtonBlock block(instance()->impl);
    QEventLoop eventLoop;
    connect(&box, SIGNAL(finished(int)), &eventLoop, SLOT(quit()));
    eventLoop.exec();

    return (QMessageBox::StandardButton)box.result();
}


/*
  This function is defined here instead of PyGrxUI.cpp so that the message translations can be edited with one file
*/
bool GrxUIMenuView::waitInputConfirm(const std::string& message)
{
    QMessageBox box(MainWindow::instance());
    box.setWindowTitle(_("Wait input confirm"));
    box.setText(message.c_str());
    box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    box.setDefaultButton(QMessageBox::Ok);
    box.setModal(false);
    box.show();

    MenuButtonBlock block(instance()->impl);
    QEventLoop eventLoop;
    connect(&box, SIGNAL(finished(int)), &eventLoop, SLOT(quit()));
    eventLoop.exec();

    return (box.result() == QMessageBox::Ok);
}


/*
  This function is defined here instead of PyGrxUI.cpp so that the message translations can be edited with one file
*/
std::string GrxUIMenuView::waitInputMessage(const std::string& message)
{
    Dialog dialog;
    dialog.setWindowTitle(_("Wait input message"));
    QVBoxLayout* vbox = new QVBoxLayout();
    dialog.setLayout(vbox);

    vbox->addWidget(new QLabel(message.c_str()));
    
    LineEdit* lineEdit = new LineEdit();
    connect(lineEdit, SIGNAL(returnPressed()), &dialog, SLOT(accept()));
    vbox->addWidget(lineEdit);

    PushButton* okButton = new PushButton(_("&OK"));
    okButton->setDefault(true);
    connect(okButton, SIGNAL(clicked()), &dialog, SLOT(accept()));
    vbox->addWidget(okButton);
    vbox->addStretch();
    
    dialog.show();

    MenuButtonBlock block(instance()->impl);
    QEventLoop eventLoop;
    connect(&dialog, SIGNAL(finished(int)), &eventLoop, SLOT(quit()));
    eventLoop.exec();

    return lineEdit->string();
}


MenuWidget::MenuWidget
(const python::list& menu, bool isLocalSequentialMode, bool doBackgroundExecution, GrxUIMenuViewImpl* view)
    : view(view)
{
    QVBoxLayout* vbox = new QVBoxLayout();
    
    QFrame* barFrame = new QFrame();
    barFrame->setFrameShape(QFrame::StyledPanel);
    barFrame->setLayout(&barStack);
            
    vbox->addWidget(barFrame);
    vbox->addLayout(&pageStack, 1);

    QHBoxLayout* hbox;

    QWidget* regularBar = new QWidget(this);
    regularBar->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

    hbox = new QHBoxLayout();

    this->isLocalSequentialMode = isLocalSequentialMode;
    sequenceModeButton.setText("V");
    sequenceModeButton.setToolTip(_("Show buttons executed sequentially"));
    sequenceModeButton.sigClicked().connect(std::bind(&MenuWidget::showSequencePages, this));
    hbox->addWidget(&sequenceModeButton);
    hbox->addStretch();
            
    hbox->addWidget(new QLabel(_("Command Always Enabled")));
    hbox->addStretch();
            
    terminateButton1.setText("X");
    terminateButton1.setToolTip(_("Terminate the command being executed"));
    terminateButton1.sigClicked().connect(std::bind(&MenuWidget::onTerminateClicked, this));
    hbox->addWidget(&terminateButton1);
            
    regularBar->setLayout(hbox);
    barStack.addWidget(regularBar);
            
    QWidget* sequenceBar = new QWidget(this);
    sequenceBar->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
            
    hbox = new QHBoxLayout();

    regularModeButton.setText("^");
    regularModeButton.setToolTip(_("Show buttons always enabled"));
    regularModeButton.sigClicked().connect(std::bind(&MenuWidget::showRegularPage, this));
    hbox->addWidget(&regularModeButton);
            
    hbox->addStretch();

    prevPageButton.setText("<");
    prevPageButton.setToolTip(_("Show previous menu"));
    prevPageButton.sigClicked().connect(std::bind(&MenuWidget::moveSequentialPage, this, -1));
    hbox->addWidget(&prevPageButton);
            
    hbox->addWidget(&pageIndexLabel);
            
    nextPageButton.setText(">");
    nextPageButton.setToolTip(_("Show next menu"));
    nextPageButton.sigClicked().connect(std::bind(&MenuWidget::moveSequentialPage, this, +1));
    hbox->addWidget(&nextPageButton);
            
    sequentialCheck.setText(_("sequential"));
    sequentialCheck.setToolTip(_("Enable sequential execution"));
    sequentialCheck.setChecked(true);
    sequentialCheck.sigToggled().connect(std::bind(&MenuWidget::onSequentialCheckToggled, this, stdph::_1));
    hbox->addWidget(&sequentialCheck);

    hbox->addStretch();
            
    retryButton.setText("<-|");
    retryButton.setToolTip(_("Retry from first"));
    retryButton.sigClicked().connect(std::bind(&MenuWidget::onRetryClicked, this));
    hbox->addWidget(&retryButton);
            
    terminateButton2.setText("X");
    terminateButton2.setToolTip(_("Terminate the command being executed"));
    terminateButton2.sigClicked().connect(std::bind(&MenuWidget::onTerminateClicked, this));
    hbox->addWidget(&terminateButton2);
            
    sequenceBar->setLayout(hbox);
    barStack.addWidget(sequenceBar);
            
    setLayout(vbox);

    createPages(menu);

    pythonExecutor.setBackgroundMode(doBackgroundExecution);
    pythonExecutor.sigFinished().connect(std::bind(&MenuWidget::onScriptFinished, this));

    currentButton = 0;
    currentButtonPage = 0;
    currentSequencePageIndex = 0;

    bool hasSequencePages = false;
    if(buttonList.size() >= 2){
        for(size_t i=1; i < buttonList.size(); ++i){
            if(!buttonList[i].empty()){
                hasSequencePages = true;
                break;
            }
        }
    }
    if(hasSequencePages){
        showSequencePages();
    } else {
        showRegularPage();
    }
}


void MenuWidget::createPages(const python::list& menu)
{
    int numSections = python::len(menu);
    for(int i=0; i < numSections; ++i){
        const python::list section =
#ifdef CNOID_USE_PYBIND11
            menu[i].cast<pybind11::list>();
#else
            python::extract<python::list>(menu[i]);
#endif
        addSection(section);
    }

    while(pageStack.count() < 2){
        addSection(python::list());
    }
}


void MenuWidget::addSection(const python::list& section)
{                        
    QWidget* page = new QWidget();
    page->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QVBoxLayout* vbox = new QVBoxLayout();
    page->setLayout(vbox);

    buttonList.push_back(vector<QWidget*>());
    vector<QWidget*>& buttons = buttonList.back();

    int numItems = python::len(section) / 2;

    for(int j=0; j < numItems; ++j){
        // extract a pair of elements
#ifdef CNOID_USE_PYBIND11
        const string label = section[j*2].cast<string>();
        const string function = section[j*2+1].cast<string>();
#else
        const string label = python::extract<string>(section[j*2]);
        const string function = python::extract<string>(section[j*2+1]);
#endif

        if(function == "#label"){
            vbox->addWidget(new QLabel(label.c_str()), 0, Qt::AlignCenter);

        } else if(label == "#monitor"){

        } else {
            FuncButtonBox* box = new FuncButtonBox(label, function);
            box->button.sigClicked().connect(std::bind(&MenuWidget::onButtonClicked, this, buttons.size(), box));
            vbox->addWidget(box, 0, Qt::AlignCenter);
            buttons.push_back(box);
        }
    }

    vbox->addStretch();

    QScrollArea* area = new QScrollArea();
    area->setWidgetResizable(true);
    area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    area->setAlignment(Qt::AlignHCenter);
    area->setWidget(page);
    pageStack.addWidget(area);
}


void MenuWidget::onButtonClicked(int indexInPage, FuncButtonBox* box)
{
    string code = box->funcString;

    size_t pos = 0;
    for(size_t i=0; i < box->entries.size(); ++i){
        const FuncParamEntry* entry = box->entries[i];
        pos = code.find(entry->marker, pos);
        if(pos != string::npos){
            string value = entry->text();
            code.replace(pos, entry->marker.length(), value);
            pos += value.length();
        }
    }

    lastIndexInPage = indexInPage;

    if(pythonExecutor.state() == PythonExecutor::RUNNING_BACKGROUND){
        QMessageBox mbox;
        mbox.setWindowTitle(_("Python Script Termination"));
        mbox.setText(_("The previously executed Python script is still running in the background thread. "
                       "Do you want to terminate it and execute the command you clicked or cancel the command you clicked?"));
        mbox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        mbox.setDefaultButton(QMessageBox::Cancel);
        int result = mbox.exec();
                           
        if(result == QMessageBox::Cancel){
            MessageView::instance()->putln(MessageView::WARNING, _("The script was canceled."));
            return;
        } else if(!pythonExecutor.terminate()){
            showWarningDialog(_("The script cannot be terminated and the command you clicked cannot be executed."));
            return;
        }
    }

    view->blockMenuButtons(true);
    pythonExecutor.execCode(code);
}


void MenuWidget::onScriptFinished()
{
    bool completed = false;

    view->blockMenuButtons(false);

    if(pythonExecutor.isTerminated()){
        MessageView::instance()->putln(_("The script has been terminated."));
        
    } else if(pythonExecutor.hasException()){
        python::gil_scoped_acquire lock;
#ifdef CNOID_USE_PYBIND11
        bool isCancelException = pythonExecutor.exceptionType().is(cancelExceptionType);
#else
        bool isCancelException = (pythonExecutor.exceptionType() == cancelExceptionType);
#endif
        if(isCancelException){
            showWarningDialog(_("The script has been cancelled."));
        } else {
            MessageView::instance()->putln(pythonExecutor.exceptionText());
        }
    } else {
        completed = true;
    }

    if(completed){
        moveNext();
    }
}


void MenuWidget::moveNext()
{
    if(isSequenceMode() && sequentialCheck.isChecked()){
        if(isLocalSequentialMode){
            moveToNextButton();
        } else if(lastIndexInPage == 0){
            if(currentSequencePageIndex + 1 < static_cast<int>(buttonList.size()) - 1){
                setCurrentSequentialPage(currentSequencePageIndex + 1);
            }
        }
    }
}


void MenuWidget::moveToNextButton()
{
    vector<QWidget*> buttons = buttonList[currentSequencePageIndex+1];
    if(currentButton < static_cast<int>(buttons.size()) - 1){
        ++currentButton;
        setCurrentSequentialPage(currentSequencePageIndex);
    } else if(currentSequencePageIndex < static_cast<int>(buttonList.size()) - 1){
        currentButton = 0;
        ++currentButtonPage; 
        setCurrentSequentialPage(currentSequencePageIndex + 1);
    }
}


bool MenuWidget::isSequenceMode() {
    return (barStack.currentIndex() == 1);
}


void MenuWidget::showSequencePages()
{
    barStack.setCurrentIndex(1);
    setCurrentSequentialPage(currentSequencePageIndex);
}


void MenuWidget::showRegularPage()
{
    barStack.setCurrentIndex(0);
    pageStack.setCurrentIndex(0);
}


void MenuWidget::moveSequentialPage(int direction)
{
    const int lastPage = pageStack.count() - 2;
    const int index = std::max(0, std::min(currentSequencePageIndex + direction, lastPage));
    setCurrentSequentialPage(index);
}


void MenuWidget::setCurrentSequentialPage(int index)
{
    pageStack.setCurrentIndex(index + 1);

    if(isLocalSequentialMode){
        const bool hasCurrentButton = (index == currentButtonPage);
        vector<QWidget*>& buttons = buttonList[index + 1];
        for(int i=0; i < static_cast<int>(buttons.size()); ++i){
            buttons[i]->setEnabled(
                !sequentialCheck.isChecked() ||(hasCurrentButton && (i == currentButton)));
        }
    }
            
    prevPageButton.setEnabled(!sequentialCheck.isChecked() && index > 0);
    const int numPages = pageStack.count() - 1;
    nextPageButton.setEnabled(!sequentialCheck.isChecked() && index < numPages - 1);
    currentSequencePageIndex = index;
    pageIndexLabel.setText(QString("%1/%2").arg(index + 1).arg(numPages));
}


void MenuWidget::onSequentialCheckToggled(bool on)
{
    setCurrentSequentialPage(currentSequencePageIndex);
}


void MenuWidget::onRetryClicked()
{
    currentButton = 0;
    currentButtonPage = 0;
    setCurrentSequentialPage(0);
}


void MenuWidget::onTerminateClicked()
{
    if(pythonExecutor.state() == PythonExecutor::RUNNING_BACKGROUND){
        bool doTermination =
            showConfirmDialog(
                _("Python Script Termination"),
                _("Do you really want to terminate the script being executed?"));
        if(doTermination){
            if(!pythonExecutor.terminate()){
                showWarningDialog(_("The script cannot be terminated."));
                view->blockMenuButtons(false);
            }
        }
    } else {
        view->blockMenuButtons(false);
    }
}


void MenuWidget::onQuitClicked()
{
    view->clearMenu(false, true);
    deleteLater();
}
