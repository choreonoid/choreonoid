/**
   @author Shin'ichiro Nakaoka
*/

#include "PythonConsoleView.h"
#include "PythonUtil.h"
#include <cnoid/PyUtil>
#include <cnoid/MessageView>
#include <cnoid/ViewManager>
#include <QPlainTextEdit>
#include <QBoxLayout>
#include <QTextBlock>
#include <QEventLoop>
#include <list>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
const unsigned int HISTORY_SIZE = 100;

class PythonConsoleOut
{
    PythonConsoleViewImpl* console;
public:
    void setConsole(PythonConsoleViewImpl* console);
    void write(std::string const& text);
};

class PythonConsoleIn
{
public:
    PythonConsoleViewImpl* console;
    void setConsole(PythonConsoleViewImpl* console);
    python::object readline();
};


}

namespace cnoid {
    
class PythonConsoleViewImpl : public QPlainTextEdit
{
public:
    PythonConsoleViewImpl(PythonConsoleView* self);
    ~PythonConsoleViewImpl();

    PythonConsoleView* self;
    bool isConsoleInMode;
    QEventLoop eventLoop;
    string stringFromConsoleIn;
    int inputColumnOffset;
    QString prompt;
    std::list<QString>::iterator histIter;
    std::list<QString> history;

    python::object consoleOut;
    python::object consoleIn;
    python::object sys;
    python::object orgStdout;
    python::object orgStderr;
    python::object orgStdin;
    python::object interpreter;

    void setPrompt(const char* newPrompt);
    void put(const QString& message);
    void putln(const QString& message);
    void putPrompt();
    void execCommand();
    QString getInputString();
    void setInputString(const QString& command);
    void addToHistory(const QString& command);
    QString getPrevHistoryEntry();
    QString getNextHistoryEntry();

    string getInputFromConsoleIn();
    void fixInput();
    
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void insertFromMimeData(const QMimeData* source);
};

}


void PythonConsoleOut::setConsole(PythonConsoleViewImpl* console)
{
    this->console = console;
}


void PythonConsoleOut::write(std::string const& text)
{
    console->put(QString(text.c_str()));
}


void PythonConsoleIn::setConsole(PythonConsoleViewImpl* console)
{
    this->console = console;
}


python::object PythonConsoleIn::readline()
{
    //! \todo release the GIL inside this function
    return python::str(console->getInputFromConsoleIn());
}


void PythonConsoleView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<PythonConsoleView>(
        "PythonConsoleView", N_("Python Console"), ViewManager::SINGLE_OPTIONAL);
}


PythonConsoleView::PythonConsoleView()
{
    impl = new PythonConsoleViewImpl(this);
}


PythonConsoleViewImpl::PythonConsoleViewImpl(PythonConsoleView* self)
    : self(self)
{
    isConsoleInMode = false;
    inputColumnOffset = 0;
    
    self->setDefaultLayoutArea(View::BOTTOM);

    setFrameShape(QFrame::NoFrame);
    setReadOnly(false);
    setWordWrapMode(QTextOption::WrapAnywhere);
    setUndoRedoEnabled(false);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(this);
    self->setLayout(hbox);

    PyGILock lock;
    
#ifdef _WIN32
    try { interpreter = python::import("code").attr("InteractiveConsole")(pythonMainNamespace());
    } catch (...) { /* ignore the exception on windows. this module is loaded already. */} 
#else
    interpreter = python::import("code").attr("InteractiveConsole")(pythonMainNamespace());
#endif

    python::object consoleOutClass =
        python::class_<PythonConsoleOut>("PythonConsoleOut", python::init<>())
        .def("write", &PythonConsoleOut::write);
    consoleOut = consoleOutClass();
    PythonConsoleOut& consoleOut_ = python::extract<PythonConsoleOut&>(consoleOut);
    consoleOut_.setConsole(this);

    python::object consoleInClass =
        python::class_<PythonConsoleIn>("PythonConsoleIn", python::init<>())
        .def("readline", &PythonConsoleIn::readline);
    consoleIn = consoleInClass();
    PythonConsoleIn& consoleIn_ = python::extract<PythonConsoleIn&>(consoleIn);
    consoleIn_.setConsole(this);
    
    sys = pythonSysModule();

    histIter = history.end();

    putln(QString("Python %1").arg(Py_GetVersion()));
    
    prompt = ">>> ";
    putPrompt();
}


PythonConsoleView::~PythonConsoleView()
{
    PyGILock lock;
    delete impl;
}


PythonConsoleViewImpl::~PythonConsoleViewImpl()
{

}


void PythonConsoleViewImpl::setPrompt(const char* newPrompt)
{
    prompt = newPrompt;
}


void PythonConsoleView::put(const std::string& message)
{
    impl->put(message.c_str());
}


void PythonConsoleViewImpl::put(const QString& message)
{
    moveCursor(QTextCursor::End);
    insertPlainText(message);
    moveCursor(QTextCursor::End);
}


void PythonConsoleView::putln(const std::string& message)
{
    impl->putln(message.c_str());
}


void PythonConsoleViewImpl::putln(const QString& message)
{
    put(message + "\n");
    MessageView::instance()->flush();
}


void PythonConsoleView::flush()
{
    MessageView::instance()->flush();
}


void PythonConsoleView::clear()
{
    impl->clear();
}


void PythonConsoleViewImpl::putPrompt()
{
    put(prompt);
    inputColumnOffset = textCursor().columnNumber();
}


void PythonConsoleViewImpl::execCommand()
{
    PyGILock lock;
    
    orgStdout = sys.attr("stdout");
    orgStderr = sys.attr("stderr");
    orgStdin = sys.attr("stdin");
    
    sys.attr("stdout") = consoleOut;
    sys.attr("stderr") = consoleOut;
    sys.attr("stdin") = consoleIn;
    
    QString command = getInputString();
    
    put("\n"); // This must be done after getInputString().
        
    if(python::extract<bool>(interpreter.attr("push")(command.toStdString()))){
        setPrompt("... ");
    } else {
        setPrompt(">>> ");
    }
    
    if(PyErr_Occurred()){
        PyErr_Print();
    }
    
    sys.attr("stdout") = orgStdout;
    sys.attr("stderr") = orgStderr;
    sys.attr("stdin") = orgStdin;

    addToHistory(command);

    putPrompt();
}


QString PythonConsoleViewImpl::getInputString()
{
    QTextDocument* doc = document();
    QString line = doc->findBlockByLineNumber(doc->lineCount() - 1).text();
    line.remove(0, inputColumnOffset);
    return line;
}


void PythonConsoleViewImpl::setInputString(const QString& command)
{
    if(getInputString() == command){
        return;
    }

    QTextCursor cursor = textCursor();
    cursor.movePosition(QTextCursor::End);
    cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
    cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, inputColumnOffset);
    cursor.removeSelectedText();
    cursor.insertText(command);
    moveCursor(QTextCursor::End);
}


void PythonConsoleViewImpl::addToHistory(const QString& command)
{
    if(!command.isEmpty()){
        if(history.empty() || history.back() != command){
            if(HISTORY_SIZE <= history.size()){
                history.pop_front();
            }
            history.push_back(command);
        }
        histIter = history.end();
    }
}


QString PythonConsoleViewImpl::getPrevHistoryEntry()
{
    if(!history.empty()){
        if(histIter != history.begin()){
            --histIter;
        }
        return *histIter;
    }
    return QString();
}


QString PythonConsoleViewImpl::getNextHistoryEntry()
{
    if(!history.empty()){
        if(histIter != history.end()){
            ++histIter;
            if(histIter != history.end()){
                return *histIter;
            }
        }
    }
    return QString();
}


string PythonConsoleViewImpl::getInputFromConsoleIn()
{
    sys.attr("stdout") = orgStdout;
    sys.attr("stderr") = orgStderr;
    sys.attr("stdin") = orgStdin;

    int result;

    Py_BEGIN_ALLOW_THREADS
        
    isConsoleInMode = true;
    inputColumnOffset = textCursor().columnNumber();
    
    result = eventLoop.exec();
    isConsoleInMode = false;

    Py_END_ALLOW_THREADS

    sys.attr("stdout") = consoleOut;
    sys.attr("stderr") = consoleOut;
    sys.attr("stdin") = consoleIn;

    if(result == 0){
        return stringFromConsoleIn + "\n";
    } else {
        put("\n");
        //! \todo put an error message here
        return string();
    }
}


void PythonConsoleViewImpl::fixInput()
{
    stringFromConsoleIn = getInputString().toStdString();
    put("\n");
    eventLoop.exit();
}


void PythonConsoleViewImpl::keyPressEvent(QKeyEvent* event)
{
    bool done = false;

    switch(event->key()){

    case Qt::Key_Left:
    case Qt::Key_Backspace:
        if(textCursor().columnNumber() <= inputColumnOffset){
            done = true;
        }
        break;
        
    case Qt::Key_Up:
        setInputString(getPrevHistoryEntry());
        done = true;
        break;
        
    case Qt::Key_Down:
        setInputString(getNextHistoryEntry());
        done = true;
        break;
        
    case Qt::Key_Return:
        if(isConsoleInMode){
            fixInput();
        } else {
            execCommand();
        }
        done = true;
        break;
        
    default:
        break;
    }

    if(!done){
        QPlainTextEdit::keyPressEvent(event);
    }
}


/**
   \todo Implement this virtual function to correctly process a pasted text block
*/
void PythonConsoleViewImpl::insertFromMimeData(const QMimeData* source)
{
    if(!source->hasText()){
        QPlainTextEdit::insertFromMimeData(source);

    } else {
        QString text = source->text();
        QStringList lines = text.split(QRegExp("(\r\n|\r|\n)"));
        int n = lines.size();
        if(n > 0){
            for(int i=0; i < n - 1; ++i){
                put(lines[i]);
                execCommand();
            }
            const QString& lastLine = lines[n-1];
            if(!lastLine.isEmpty()){
                put(lastLine);
                if(text.contains(QRegExp("[\r\n]$"))){
                    execCommand();
                }
            }
        }
    }
}
