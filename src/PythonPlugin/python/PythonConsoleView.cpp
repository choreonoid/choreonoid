#include "PythonConsoleView.h"
#include "PythonPlugin.h"
#include "PyCApiUtil.h"
#include <cnoid/MessageView>
#include <cnoid/ViewManager>
#include <QPlainTextEdit>
#include <QBoxLayout>
#include <QTextBlock>
#include <QEventLoop>
#include <QMimeData>
#include <QRegularExpression>
#include <algorithm>
#include <limits>
#include <list>
#include <cstring>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const unsigned int HISTORY_SIZE = 100;

}

namespace cnoid {

class PythonConsoleView::Impl : public QPlainTextEdit
{
public:
    Impl(PythonConsoleView* self);
    ~Impl();

    PythonConsoleView* self;
    PythonPlugin* pythonPlugin;
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

    PyObjectHandle mainModule;
    PyObjectHandle globalNamespace;
    PyObjectHandle consoleOut;
    PyObjectHandle consoleIn;
    PyObjectHandle sys;
    PyObjectHandle orgStdout;
    PyObjectHandle orgStderr;
    PyObjectHandle orgStdin;
    PyObjectHandle interpreter;

    void setPrompt(const char* newPrompt);
    void put(const QString& message);
    void putln(const QString& message);
    void putPrompt();
    void execCommand();
    PyObjectHandle getMemberObject(std::vector<string>& moduleNames, PyObject* parentObject);
    std::vector<string> getMemberNames(PyObject* moduleObject);
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


namespace {

// The console stdout/stderr/stdin replacements are implemented as Python types
// created with PyType_FromSpec, each holding a pointer to the console Impl.

struct ConsoleObject {
    PyObject_HEAD
    PythonConsoleView::Impl* console;
};

PyObject* PythonConsoleOut_write(PyObject* self, PyObject* args)
{
    const char* text = nullptr;
    if(!PyArg_ParseTuple(args, "s", &text)){
        return nullptr;
    }
    auto console = reinterpret_cast<ConsoleObject*>(self)->console;
    console->put(QString(text));
    console->sigOutput(text);
    Py_RETURN_NONE;
}

PyMethodDef PythonConsoleOut_methods[] = {
    { "write", PythonConsoleOut_write, METH_VARARGS, nullptr },
    { nullptr, nullptr, 0, nullptr }
};

PyType_Slot PythonConsoleOut_slots[] = {
    { Py_tp_methods, PythonConsoleOut_methods },
    { 0, nullptr }
};

PyType_Spec PythonConsoleOut_spec = {
    "cnoid.PythonPlugin.PythonConsoleOut",
    sizeof(ConsoleObject),
    0,
    Py_TPFLAGS_DEFAULT,
    PythonConsoleOut_slots
};

PyObject* PythonConsoleIn_readline(PyObject* self, PyObject* /* args */)
{
    auto console = reinterpret_cast<ConsoleObject*>(self)->console;
    return PyUnicode_FromString(console->getInputFromConsoleIn().c_str());
}

PyMethodDef PythonConsoleIn_methods[] = {
    { "readline", PythonConsoleIn_readline, METH_NOARGS, nullptr },
    { nullptr, nullptr, 0, nullptr }
};

PyType_Slot PythonConsoleIn_slots[] = {
    { Py_tp_methods, PythonConsoleIn_methods },
    { 0, nullptr }
};

PyType_Spec PythonConsoleIn_spec = {
    "cnoid.PythonPlugin.PythonConsoleIn",
    sizeof(ConsoleObject),
    0,
    Py_TPFLAGS_DEFAULT,
    PythonConsoleIn_slots
};

PyObjectHandle createConsoleObject(PyType_Spec* spec, PythonConsoleView::Impl* console)
{
    PyObject* type = PyType_FromSpec(spec);
    if(!type){
        return PyObjectHandle();
    }
    PyObject* obj = PyObject_CallObject(type, nullptr);
    Py_DECREF(type);
    if(!obj){
        return PyObjectHandle();
    }
    reinterpret_cast<ConsoleObject*>(obj)->console = console;
    return PyObjectHandle::steal(obj);
}

}


void PythonConsoleView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<PythonConsoleView>(
        N_("PythonConsoleView"), N_("Python Console"), ViewManager::Permanent);
}


PythonConsoleView::PythonConsoleView()
{
    impl = new Impl(this);
    setFocusProxy(impl);
}


PythonConsoleView::Impl::Impl(PythonConsoleView* self)
    : self(self),
      pythonPlugin(PythonPlugin::instance())
{
    isConsoleInMode = false;
    inputColumnOffset = 0;

    splitStringVec = {
        " ", "{",  "}", "(",  ")", "[", "]", "<", ">", ":", ";", "^",
        "@", "\"", ",", "\\", "!", "#", "'", "=", "|", "*", "?", "\t" };

    self->setDefaultLayoutArea(BottomCenterArea);

    setFrameShape(QFrame::NoFrame);
    setReadOnly(false);
    setWordWrapMode(QTextOption::WrapAnywhere);
    setUndoRedoEnabled(false);

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(this);
    self->setLayout(hbox);

    GilScopedAcquire lock;

    mainModule = PyObjectHandle::borrow(pythonPlugin->mainModule());
    globalNamespace = PyObjectHandle::borrow(pythonPlugin->globalNamespace());

    {
        PyObject* codeModule = PyImport_ImportModule("code");
        if(codeModule){
            PyObject* consoleClass = PyObject_GetAttrString(codeModule, "InteractiveConsole");
            if(consoleClass){
                interpreter = PyObjectHandle::steal(
                    PyObject_CallFunctionObjArgs(consoleClass, globalNamespace.get(), nullptr));
                Py_DECREF(consoleClass);
            }
            Py_DECREF(codeModule);
        }
#ifdef _WIN32
        // ignore the exception on windows. this module is loaded already.
        if(!interpreter){
            PyErr_Clear();
        }
#endif
    }

    consoleOut = createConsoleObject(&PythonConsoleOut_spec, this);
    consoleIn = createConsoleObject(&PythonConsoleIn_spec, this);

    sys = PyObjectHandle::borrow(pythonPlugin->sysModule());

    {
        PyObject* keyword = PyImport_ImportModule("keyword");
        if(keyword){
            PyObject* kwlist = PyObject_GetAttrString(keyword, "kwlist");
            if(kwlist){
                Py_ssize_t n = PySequence_Size(kwlist);
                for(Py_ssize_t i = 0; i < n; ++i){
                    PyObject* item = PySequence_GetItem(kwlist, i);
                    if(item){
                        const char* s = PyUnicode_AsUTF8(item);
                        if(s){
                            keywords.push_back(s);
                        }
                        Py_DECREF(item);
                    }
                }
                Py_DECREF(kwlist);
            }
            Py_DECREF(keyword);
        }
    }

    histIter = history.end();

    putln(QString("Python %1").arg(Py_GetVersion()));

    prompt = ">>> ";
    putPrompt();
}


PythonConsoleView::~PythonConsoleView()
{
    GilScopedAcquire lock;
    delete impl;
}


PythonConsoleView::Impl::~Impl()
{

}


void PythonConsoleView::onActivated()
{
    impl->setFocus();
}


void PythonConsoleView::Impl::setPrompt(const char* newPrompt)
{
    prompt = newPrompt;
}


void PythonConsoleView::Impl::put(const QString& message)
{
    moveCursor(QTextCursor::End);
    insertPlainText(message);
    moveCursor(QTextCursor::End);
}


void PythonConsoleView::Impl::putln(const QString& message)
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


void PythonConsoleView::Impl::putPrompt()
{
    put(prompt);
    sigOutput(prompt.toStdString());
    inputColumnOffset = textCursor().columnNumber();
}


void PythonConsoleView::Impl::execCommand()
{
    GilScopedAcquire lock;

    orgStdout = PyObjectHandle::steal(PyObject_GetAttrString(sys.get(), "stdout"));
    orgStderr = PyObjectHandle::steal(PyObject_GetAttrString(sys.get(), "stderr"));
    orgStdin = PyObjectHandle::steal(PyObject_GetAttrString(sys.get(), "stdin"));

    PyObject_SetAttrString(sys.get(), "stdout", consoleOut.get());
    PyObject_SetAttrString(sys.get(), "stderr", consoleOut.get());
    PyObject_SetAttrString(sys.get(), "stdin", consoleIn.get());

    QString command = getInputString();

    put("\n"); // This must be done after getInputString().

    bool isIncomplete = false;
    PyObjectHandle r = PyObjectHandle::steal(
        PyObject_CallMethod(interpreter.get(), "push", "s", command.toStdString().c_str()));
    if(r){
        isIncomplete = PyObject_IsTrue(r.get());
    }
    if(isIncomplete){
        setPrompt("... ");
    } else {
        setPrompt(">>> ");
    }

    if(PyErr_Occurred()){
        PyErr_Print();
    }

    PyObject_SetAttrString(sys.get(), "stdout", orgStdout.get());
    PyObject_SetAttrString(sys.get(), "stderr", orgStderr.get());
    PyObject_SetAttrString(sys.get(), "stdin", orgStdin.get());

    addToHistory(command);

    putPrompt();
}


PyObjectHandle PythonConsoleView::Impl::getMemberObject(
    std::vector<string>& moduleNames, PyObject* parentObject)
{
    if(moduleNames.size() == 0){
        return PyObjectHandle::borrow(parentObject);
    } else {
        string moduleName = moduleNames.front();
        moduleNames.erase(moduleNames.begin());
        std::vector<string> memberNames = getMemberNames(parentObject);
        if(std::find(memberNames.begin(), memberNames.end(), moduleName) == memberNames.end()){
            return PyObjectHandle();
        } else {
            PyObjectHandle childObject =
                PyObjectHandle::steal(PyObject_GetAttrString(parentObject, moduleName.c_str()));
            return getMemberObject(moduleNames, childObject.get());
        }
    }
}


std::vector<string> PythonConsoleView::Impl::getMemberNames(PyObject* moduleObject)
{
    std::vector<string> retNames;
    if(moduleObject == nullptr){
        return retNames;
    }
    PyObject* dir = PyObject_Dir(moduleObject);
    if(!dir){
        PyErr_Clear();
        return retNames;
    }
    Py_ssize_t n = PyList_Size(dir);
    for(Py_ssize_t i = 0; i < n; ++i){
        PyObject* item = PyList_GetItem(dir, i); // borrowed
        const char* s = PyUnicode_AsUTF8(item);
        if(s && !strstr(s, "__")){
            retNames.push_back(s);
        }
    }
    Py_DECREF(dir);
    return retNames;
}


void PythonConsoleView::Impl::tabComplete()
{
    GilScopedAcquire lock;

    orgStdout = PyObjectHandle::steal(PyObject_GetAttrString(sys.get(), "stdout"));
    orgStderr = PyObjectHandle::steal(PyObject_GetAttrString(sys.get(), "stderr"));
    orgStdin = PyObjectHandle::steal(PyObject_GetAttrString(sys.get(), "stdin"));

    PyObject_SetAttrString(sys.get(), "stdout", consoleOut.get());
    PyObject_SetAttrString(sys.get(), "stderr", consoleOut.get());
    PyObject_SetAttrString(sys.get(), "stdin", consoleIn.get());

    QTextCursor cursor = textCursor();
    string beforeCursorString = getInputString().toStdString();
    beforeCursorString = beforeCursorString.substr(0, cursor.columnNumber() - inputColumnOffset);
    QString afterCursorString = getInputString();
    afterCursorString.remove(0, cursor.columnNumber() - inputColumnOffset);
    size_t maxSplitIdx = 0;
    for(std::vector<string>::iterator it = splitStringVec.begin(); it != splitStringVec.end(); ++it){
        size_t splitIdx = beforeCursorString.find_last_of(*it);
        maxSplitIdx = std::max(splitIdx == string::npos ? 0 : splitIdx + 1, maxSplitIdx);
    }
    string lastWord = beforeCursorString.substr(maxSplitIdx);
    beforeCursorString = beforeCursorString.substr(0, maxSplitIdx);

    std::vector<string> dottedStrings;
    size_t pos = 0;
    while(true){
        size_t dotpos = lastWord.find('.', pos);
        if(dotpos == pos){
            break;
        }
        size_t n = (dotpos == string::npos) ? string::npos : (dotpos - pos);
        dottedStrings.push_back(lastWord.substr(pos, n));
        if(dotpos == string::npos){
            break;
        }
        pos = dotpos + 1;
    }
    string lastDottedString = dottedStrings.back();// word after last dot

    std::vector<string> moduleNames = dottedStrings;// words before last dot
    moduleNames.pop_back();

    PyObjectHandle targetMemberObject = getMemberObject(moduleNames, mainModule.get()); //member object before last dot
    std::vector<string> memberNames = getMemberNames(targetMemberObject.get());

    // builtin function and syntax completions
    if(dottedStrings.size() == 1){
        PyObjectHandle builtinsObject =
            PyObjectHandle::steal(PyObject_GetAttrString(mainModule.get(), "__builtins__"));
        std::vector<string> builtinMethods = getMemberNames(builtinsObject.get());
        memberNames.insert(memberNames.end(), builtinMethods.begin(), builtinMethods.end());
        memberNames.insert(memberNames.end(), keywords.begin(), keywords.end());
    }

    std::vector<string> completions;
    unsigned long int maxLength = std::numeric_limits<long>::max();
    for(size_t i=0; i < memberNames.size(); ++i){
        if(memberNames[i].substr(0, lastDottedString.size()) == lastDottedString){
            completions.push_back(memberNames[i]);
            maxLength = std::min((unsigned long int)memberNames[i].size(), maxLength);
        }
    }

    if(PyErr_Occurred()){
        PyErr_Print();
    }

    PyObject_SetAttrString(sys.get(), "stdout", orgStdout.get());
    PyObject_SetAttrString(sys.get(), "stderr", orgStderr.get());
    PyObject_SetAttrString(sys.get(), "stdin", orgStdin.get());

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

QString PythonConsoleView::Impl::getInputString()
{
    QTextDocument* doc = document();
    QString line = doc->findBlockByLineNumber(doc->lineCount() - 1).text();
    line.remove(0, inputColumnOffset);
    return line;
}


void PythonConsoleView::Impl::setInputString(const QString& command)
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


void PythonConsoleView::Impl::addToHistory(const QString& command)
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


QString PythonConsoleView::Impl::getPrevHistoryEntry()
{
    if(!history.empty()){
        if(histIter != history.begin()){
            --histIter;
        }
        return *histIter;
    }
    return QString();
}


QString PythonConsoleView::Impl::getNextHistoryEntry()
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


string PythonConsoleView::Impl::getInputFromConsoleIn()
{
    PyObject_SetAttrString(sys.get(), "stdout", orgStdout.get());
    PyObject_SetAttrString(sys.get(), "stderr", orgStderr.get());
    PyObject_SetAttrString(sys.get(), "stdin", orgStdin.get());

    int result;

    Py_BEGIN_ALLOW_THREADS

    isConsoleInMode = true;
    inputColumnOffset = textCursor().columnNumber();

    result = eventLoop.exec();
    isConsoleInMode = false;

    Py_END_ALLOW_THREADS

    PyObject_SetAttrString(sys.get(), "stdout", consoleOut.get());
    PyObject_SetAttrString(sys.get(), "stderr", consoleOut.get());
    PyObject_SetAttrString(sys.get(), "stdin", consoleIn.get());

    if(result == 0){
        return stringFromConsoleIn + "\n";
    } else {
        put("\n");
        //! \todo put an error message here
        return string();
    }
}


void PythonConsoleView::Impl::fixInput()
{
    stringFromConsoleIn = getInputString().toStdString();
    put("\n");
    eventLoop.exit();
}


void PythonConsoleView::Impl::keyPressEvent(QKeyEvent* event)
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
void PythonConsoleView::Impl::insertFromMimeData(const QMimeData* source)
{
    if(!source->hasText()){
        QPlainTextEdit::insertFromMimeData(source);

    } else {
        QString text = source->text();
        QStringList lines = text.split(QRegularExpression("(\r\n|\r|\n)"));
        int n = lines.size();
        if(n > 0){
            for(int i=0; i < n - 1; ++i){
                put(lines[i]);
                execCommand();
            }
            const QString& lastLine = lines[n-1];
            if(!lastLine.isEmpty()){
                put(lastLine);
                if(text.contains(QRegularExpression("[\r\n]$"))){
                    execCommand();
                }
            }
        }
    }
}
