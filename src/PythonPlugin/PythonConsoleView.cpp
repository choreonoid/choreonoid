/**
   @author Shin'ichiro Nakaoka
*/

#include "PythonConsoleView.h"
#include "PythonUtil.h"
#include <cnoid/MessageView>
#include <cnoid/ViewManager>
#include <QPlainTextEdit>
#include <QBoxLayout>
#include <QTextBlock>
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
}

namespace cnoid {
    
class PythonConsoleViewImpl : public QPlainTextEdit
{
public:
    PythonConsoleViewImpl(PythonConsoleView* self);
    ~PythonConsoleViewImpl();

    PythonConsoleView* self;
    QString prompt;
    std::list<QString>::iterator histIter;
    std::list<QString> history;

    python::object consoleOut;
    python::object sys;
    python::object orgStdout;
    python::object orgStderr;
    python::object interpreter;

    void setPrompt(const char* newPrompt);
    void put(const QString& message);
    void putln(const QString& message);
    void execCommand();
    QString getCommand();
    void setCommand(const QString& command);
    void addToHistory(const QString& command);
    QString getPrevHistoryEntry();
    QString getNextHistoryEntry();

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
        
    sys = pythonSysModule();

    histIter = history.end();

    putln(QString("Python %1").arg(Py_GetVersion()));
    
    prompt = ">>> ";
    put(prompt);
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


void PythonConsoleViewImpl::execCommand()
{
    PyGILock lock;
    
    orgStdout = sys.attr("stdout");
    orgStderr = sys.attr("stderr");
    
    sys.attr("stdout") = consoleOut;
    sys.attr("stderr") = consoleOut;
    
    QString command = getCommand();
    
    put("\n"); // This must be done after getCommand().
        
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

    addToHistory(command);

    put(prompt);
}


QString PythonConsoleViewImpl::getCommand()
{
    QTextDocument* doc = document();
    QString line = doc->findBlockByLineNumber(doc->lineCount() - 1).text();
    line.remove(0, prompt.length());
    line.remove(QRegExp("\\s*$"));
    return line;
}


void PythonConsoleViewImpl::setCommand(const QString& command)
{
    if(getCommand() == command){
        return;
    }

    moveCursor(QTextCursor::End);
    moveCursor(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
    for(int i=0; i < prompt.length(); ++i){
        moveCursor(QTextCursor::Right, QTextCursor::KeepAnchor);
    }
    textCursor().removeSelectedText();
    textCursor().insertText(command);
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


void PythonConsoleViewImpl::keyPressEvent(QKeyEvent* event)
{
    bool done = false;

    switch(event->key()){

    case Qt::Key_Left:
    case Qt::Key_Backspace:
        if((textCursor().columnNumber() - prompt.length()) <= 0){
            done = true;
        }
        break;
        
    case Qt::Key_Up:
        setCommand(getPrevHistoryEntry());
        done = true;
        break;
        
    case Qt::Key_Down:
        setCommand(getNextHistoryEntry());
        done = true;
        break;
        
    case Qt::Key_Return:
        execCommand();
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
