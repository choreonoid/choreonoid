/**
   @author Shin'ichiro Nakaoka
*/

#include "PythonConsoleView.h"
#include <cnoid/PyUtil>
#include <cnoid/MessageView>
#include <cnoid/ViewManager>
#include <QPlainTextEdit>
#include <QBoxLayout>
#include <QTextBlock>
#include <QEventLoop>
#include <QMimeData>
#include <boost/algorithm/string.hpp>
#include <QMimeData>
#include <list>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

PythonConsoleView* pythonConsoleView = nullptr;

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

// defined in PythonPlugin.cpp
python::module getMainModule();
python::module getSysModule();
python::object getGlobalNamespace();
    
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
    std::vector<string> splitStringVec;
    std::vector<string> keywords;
    Signal<void(const std::string& output)> sigOutput;

    python::module mainModule;
    python::object globalNamespace;
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
    python::object getMemberObject(std::vector<string>& moduleNames, python::object& parentObject);
    std::vector<string> getMemberNames(python::object& moduleObject);
    void tabComplete();
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
    console->sigOutput(text);
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
    pythonConsoleView =
        ext->viewManager().registerClass<PythonConsoleView>(
            "PythonConsoleView", N_("Python Console"), ViewManager::SINGLE_DEFAULT);
}


PythonConsoleView* PythonConsoleView::instance()
{
    return pythonConsoleView;
}


PythonConsoleView::PythonConsoleView()
{
    impl = new PythonConsoleViewImpl(this);
    setFocusProxy(impl);
}


PythonConsoleViewImpl::PythonConsoleViewImpl(PythonConsoleView* self)
    : self(self)
{
    isConsoleInMode = false;
    inputColumnOffset = 0;

    splitStringVec = {
        " ", "{",  "}", "(",  ")", "[", "]", "<", ">", ":", ";", "^",
        "@", "\"", ",", "\\", "!", "#", "'", "=", "|", "*", "?", "\t" };
    
    self->setDefaultLayoutArea(View::BOTTOM);

    setFrameShape(QFrame::NoFrame);
    setReadOnly(false);
    setWordWrapMode(QTextOption::WrapAnywhere);
    setUndoRedoEnabled(false);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(this);
    self->setLayout(hbox);

    python::gil_scoped_acquire lock;

    mainModule = getMainModule();
    globalNamespace = getGlobalNamespace();
    
#ifdef _WIN32
    try { interpreter = python::module::import("code").attr("InteractiveConsole")(globalNamespace);
    } catch (...) { /* ignore the exception on windows. this module is loaded already. */} 
#else
    interpreter = python::module::import("code").attr("InteractiveConsole")(globalNamespace);
#endif

    
    pybind11::object consoleOutClass =
        pybind11::class_<PythonConsoleOut>(mainModule, "PythonConsoleOut")
        .def(pybind11::init<>())
        .def("write", &PythonConsoleOut::write);
    
    consoleOut = consoleOutClass();

    PythonConsoleOut& consoleOut_ = consoleOut.cast<PythonConsoleOut&>();
    consoleOut_.setConsole(this);

    python::object consoleInClass =
        pybind11::class_<PythonConsoleIn>(mainModule, "PythonConsoleIn")
        .def(pybind11::init<>())
        .def("readline", &PythonConsoleIn::readline);
    
    consoleIn = consoleInClass();
    
    PythonConsoleIn& consoleIn_ = consoleIn.cast<PythonConsoleIn&>();
    consoleIn_.setConsole(this);
    
    sys = getSysModule();

    python::object keyword = python::module::import("keyword");
    pybind11::list kwlist = pybind11::cast<pybind11::list>(keyword.attr("kwlist"));
    for(size_t i = 0; i < pybind11::len(kwlist); ++i){
        keywords.push_back(pybind11::cast<string>(kwlist[i]));
    }

    histIter = history.end();
    
    putln(QString("Python %1").arg(Py_GetVersion()));
    
    prompt = ">>> ";
    putPrompt();
}


PythonConsoleView::~PythonConsoleView()
{
    python::gil_scoped_acquire lock;
    delete impl;
}


PythonConsoleViewImpl::~PythonConsoleViewImpl()
{

}


void PythonConsoleViewImpl::setPrompt(const char* newPrompt)
{
    prompt = newPrompt;
}


void PythonConsoleViewImpl::put(const QString& message)
{
    moveCursor(QTextCursor::End);
    insertPlainText(message);
    moveCursor(QTextCursor::End);
}


void PythonConsoleViewImpl::putln(const QString& message)
{
    put(message + "\n");
    MessageView::instance()->flush();
}


void PythonConsoleView::inputCommand(const std::string& command)
{
    impl->put(command.c_str());
    impl->execCommand();
}


SignalProxy<void(const std::string& output)> PythonConsoleView::sigOutput()
{
    return impl->sigOutput;
}


void PythonConsoleViewImpl::putPrompt()
{
    put(prompt);
    sigOutput(prompt.toStdString());
    inputColumnOffset = textCursor().columnNumber();
}


void PythonConsoleViewImpl::execCommand()
{
    python::gil_scoped_acquire lock;
    
    orgStdout = sys.attr("stdout");
    orgStderr = sys.attr("stderr");
    orgStdin = sys.attr("stdin");
    
    sys.attr("stdout") = consoleOut;
    sys.attr("stderr") = consoleOut;
    sys.attr("stdin") = consoleIn;
    
    QString command = getInputString();
    
    put("\n"); // This must be done after getInputString().

    if(interpreter.attr("push")(command.toStdString()).cast<bool>()){
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
    

python::object PythonConsoleViewImpl::getMemberObject(std::vector<string>& moduleNames, python::object& parentObject)
{
    if(moduleNames.size() == 0){
        return parentObject;
    }else{
        string moduleName = moduleNames.front();
        moduleNames.erase(moduleNames.begin());
        std::vector<string> memberNames = getMemberNames(parentObject);
        if(std::find(memberNames.begin(),memberNames.end(),moduleName) == memberNames.end()){
            return python::object();
        }else{
            python::object childObject = parentObject.attr(moduleName.c_str());
            return getMemberObject(moduleNames, childObject);
        }
    }
}


std::vector<string> PythonConsoleViewImpl::getMemberNames(python::object& moduleObject)
{
    PyObject* pPyObject = moduleObject.ptr();
    if(pPyObject == NULL){
        return std::vector<string>();
    }
    pybind11::handle h( PyObject_Dir(pPyObject) );
    pybind11::list memberNames = h.cast<pybind11::list>();
    std::vector<string> retNames;
    for(size_t i=0; i < python::len(memberNames); ++i){
        if(!strstr(string(memberNames[i].cast<string>()).c_str(), "__" )){
            retNames.push_back(string(memberNames[i].cast<string>()));
        }
    }
    return retNames;
}


void PythonConsoleViewImpl::tabComplete()
{
    python::gil_scoped_acquire lock;

    orgStdout = sys.attr("stdout");
    orgStderr = sys.attr("stderr");
    orgStdin = sys.attr("stdin");
    
    sys.attr("stdout") = consoleOut;
    sys.attr("stderr") = consoleOut;
    sys.attr("stdin") = consoleIn;
    
    QTextCursor cursor = textCursor();
    string beforeCursorString = getInputString().toStdString();
    beforeCursorString = beforeCursorString.substr(0,cursor.columnNumber()-inputColumnOffset);
    QString afterCursorString = getInputString();
    afterCursorString.remove(0, cursor.columnNumber()-inputColumnOffset);
    size_t maxSplitIdx = 0;
    for(std::vector<string>::iterator it = splitStringVec.begin(); it != splitStringVec.end();  ++it){
        size_t splitIdx = beforeCursorString.find_last_of(*it);
        maxSplitIdx = std::max(splitIdx == string::npos ? 0 : splitIdx + 1, maxSplitIdx);
    }
    string lastWord = beforeCursorString.substr(maxSplitIdx);
    beforeCursorString = beforeCursorString.substr(0,maxSplitIdx);

    std::vector<string> dottedStrings;
    boost::split(dottedStrings, lastWord, boost::is_any_of("."));
    string lastDottedString = dottedStrings.back();// word after last dot

    std::vector<string> moduleNames = dottedStrings;// words before last dot
    moduleNames.pop_back();

    python::object targetMemberObject = getMemberObject(moduleNames, mainModule); //member object before last dot
    std::vector<string> memberNames = getMemberNames(targetMemberObject);

    // builtin function and syntax completions
    if(dottedStrings.size() == 1){
        python::object builtinsObject =  mainModule.attr("__builtins__");
        std::vector<string> builtinMethods = getMemberNames(builtinsObject);
        memberNames.insert(memberNames.end(), builtinMethods.begin(), builtinMethods.end());
        memberNames.insert(memberNames.end(), keywords.begin(), keywords.end());
    }

    std::vector<string> completions;
    unsigned long int maxLength = std::numeric_limits<long>::max();
    for(size_t i=0; i < memberNames.size(); ++i){
        if(memberNames[i].substr(0,lastDottedString.size()) == lastDottedString){
            completions.push_back(memberNames[i]);
            maxLength = std::min((unsigned long int)memberNames[i].size(),maxLength);
        }
    }

    if(PyErr_Occurred()){
        PyErr_Print();
    }
    
    sys.attr("stdout") = orgStdout;
    sys.attr("stderr") = orgStderr;
    sys.attr("stdin") = orgStdin;

    if(completions.size() != 0){
        // max common string among completions
        std::string maxCommonStr = lastDottedString;
        for(size_t i=maxCommonStr.size(); i < maxLength; ++i){
            bool commomFlg = true;
            for(size_t j=1; j < completions.size(); ++j){
                if(completions[0].at(i) != completions[j].at(i)){
                    commomFlg = false;
                    break;
                }
            }
            if( commomFlg ){
                maxCommonStr.push_back(completions[0].at(i));
            } else {
                break;
            }
        }

        string beforeLastDotStr = "";
        for(std::vector<string>::iterator it = dottedStrings.begin(); it != dottedStrings.end()-1; ++it){
            beforeLastDotStr.append(*it);
            beforeLastDotStr.append(".");
        }

        if(lastDottedString == maxCommonStr){
            put("\n"); // This must be done after getInputString().

            string str = "";
            for(size_t i=0; i < completions.size(); ++i){
                str.append(beforeLastDotStr);
                str.append(completions[i]);
                str.append("     ");
            }
            putln(QString(str.c_str()));
            putPrompt();
        }

        string str = "";
        str.append(beforeCursorString);
        str.append(beforeLastDotStr);
        str.append(maxCommonStr);
        str.append(afterCursorString.toStdString());
        setInputString(QString(str.c_str()));
        for(size_t i=0; i < afterCursorString.toStdString().size(); ++i){
            moveCursor(QTextCursor::Left);
        }
    }
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
    cursor.movePosition(QTextCursor::StartOfBlock, QTextCursor::KeepAnchor);
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

    case Qt::Key_F:
        if(event->modifiers() == Qt::ControlModifier){
            moveCursor(QTextCursor::Right);
            done = true;
        }
        break;
        
    case Qt::Key_B:
        if(event->modifiers() == Qt::ControlModifier){
            QTextCursor cursor = textCursor();
            if(cursor.position() > cursor.block().position() + inputColumnOffset ){
                moveCursor(QTextCursor::Left);
            }
            done = true;
        }
        break;
        
    case Qt::Key_H:
        if(event->modifiers() == Qt::ControlModifier){
            QTextCursor cursor = textCursor();
            if(cursor.position() > cursor.block().position() + inputColumnOffset ){
                cursor.movePosition(QTextCursor::Left, QTextCursor::KeepAnchor, 1);
                cursor.removeSelectedText();
             }
            done = true;
        }
        break;
        
    case Qt::Key_Left:
    case Qt::Key_Backspace:{
        QTextCursor cursor = textCursor();
        if(cursor.position() <= cursor.block().position() + inputColumnOffset ){
            done = true;
        }
        break;
    }
    case Qt::Key_P:
        if(event->modifiers() != Qt::ControlModifier){
            break;
        }
    case Qt::Key_Up:
        setInputString(getPrevHistoryEntry());
        done = true;
        break;
        
    case Qt::Key_N:
        if(event->modifiers() != Qt::ControlModifier){
            break;
        }
    case Qt::Key_Down:
        setInputString(getNextHistoryEntry());
        done = true;
        break;
        
    case Qt::Key_A:
        if(event->modifiers() == Qt::ControlModifier){
            moveCursor(QTextCursor::StartOfBlock);
            for(int i=0; i < inputColumnOffset; ++i) moveCursor(QTextCursor::Right);
            done = true;
        }
        break;
        
    case Qt::Key_E:
        if( event->modifiers() == Qt::ControlModifier ){
            moveCursor(QTextCursor::End);
        }
        break;
        
    case Qt::Key_Return:
    case Qt::Key_Enter:
        if(isConsoleInMode){
            fixInput();
        } else {
            execCommand();
        }
        done = true;
        break;
        
    case Qt::Key_Tab:
        {
            QString inputString = getInputString();
            if(inputString.toStdString().empty() || *(inputString.toStdString().end()-1) == '\t'){
                done = false;
            }else{
                tabComplete();
                done = true;
            }
        }
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
