/**
   @author Shin'ichiro Nakaoka
*/

#include "LuaConsoleView.h"
#include "LuaInterpreter.h"
#include <cnoid/ViewManager>
#include <cnoid/LazyCaller>
#include <cnoid/Config>
#include <QPlainTextEdit>
#include <QBoxLayout>
#include <QTextBlock>
#include <lua.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <regex>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const unsigned int HISTORY_SIZE = 100;

class TextSink : public boost::iostreams::sink
{
public:
    TextSink(LuaConsoleViewImpl* view) : view(view) { }
    std::streamsize write(const char* s, std::streamsize n);
    LuaConsoleViewImpl* view;
};

}

namespace cnoid {
    
class LuaConsoleViewImpl : public QPlainTextEdit
{
public:
    LuaConsoleView* self;
    int inputColumnOffset;
    QString chunk;
    std::list<QString>::iterator histIter;
    std::list<QString> history;
    std::vector<string> splitStringVec;
    std::vector<string> keywords;

    TextSink textSink;
    boost::iostreams::stream_buffer<TextSink> sbuf;
    std::ostream os;

    LuaConsoleViewImpl(LuaConsoleView* self);
    ~LuaConsoleViewImpl();
    virtual void keyPressEvent(QKeyEvent* event);
    std::streamsize write(const char* s, std::streamsize n);
    void put(const QString& message);
    void putln(const QString& message);
    void putPrompt();
    QString getInputString();
    void setInputString(const QString& command);
    void printResults(lua_State* L);
    void fixLine();
    void addToHistory(const QString& command);
    QString getPrevHistoryEntry();
    QString getNextHistoryEntry();
    void stackDump();
};

}


void LuaConsoleView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LuaConsoleView>(
        "LuaConsoleView", N_("Lua Console"), ViewManager::SINGLE_DEFAULT);
}


LuaConsoleView::LuaConsoleView()
{
    impl = new LuaConsoleViewImpl(this);
    setFocusProxy(impl);
}


LuaConsoleViewImpl::LuaConsoleViewImpl(LuaConsoleView* self)
    : self(self),
      textSink(this),
      sbuf(textSink),
      os(&sbuf)
{
    self->setDefaultLayoutArea(View::BOTTOM);

    setFrameShape(QFrame::NoFrame);
    setReadOnly(false);
    setWordWrapMode(QTextOption::WrapAnywhere);
    setUndoRedoEnabled(false);

    auto hbox = new QHBoxLayout();
    hbox->addWidget(this);
    self->setLayout(hbox);

    inputColumnOffset = 0;

    histIter = history.end();
    
    putln(LUA_COPYRIGHT);

    putPrompt();
}


LuaConsoleView::~LuaConsoleView()
{
    delete impl;
}


LuaConsoleViewImpl::~LuaConsoleViewImpl()
{

}


void LuaConsoleViewImpl::keyPressEvent(QKeyEvent* event)
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
            if(textCursor().columnNumber() > inputColumnOffset){
                moveCursor(QTextCursor::Left);
            }
            done = true;
        }
        break;
        
    case Qt::Key_H:
        if(event->modifiers() == Qt::ControlModifier){
            if(textCursor().columnNumber() > inputColumnOffset){
                QTextCursor cursor = textCursor();
                cursor.movePosition(QTextCursor::Left, QTextCursor::KeepAnchor, 1);
                cursor.removeSelectedText();
             }
            done = true;
        }
        break;
        
    case Qt::Key_Left:
    case Qt::Key_Backspace:
        if(textCursor().columnNumber() <= inputColumnOffset){
            done = true;
        }
        break;
        
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
            moveCursor(QTextCursor::StartOfLine);
            for(int i=0; i < inputColumnOffset; ++i){
                moveCursor(QTextCursor::Right);
            }
            done = true;
        }
        break;
        
    case Qt::Key_E:
        if( event->modifiers() == Qt::ControlModifier ){
            moveCursor(QTextCursor::End);
        }
        break;
        
    case Qt::Key_Return:
        fixLine();
        done = true;
        break;
        
    default:
        break;
    }

    if(!done){
        QPlainTextEdit::keyPressEvent(event);
    }
}


std::streamsize TextSink::write(const char* s, std::streamsize n)
{
    auto text = QString::fromLocal8Bit(s, n);
    callFromMainThread([=](){ view->put(text); });
    return n;
}


void LuaConsoleViewImpl::put(const QString& message)
{
    moveCursor(QTextCursor::End);
    insertPlainText(message);
    moveCursor(QTextCursor::End);
}


void LuaConsoleViewImpl::putln(const QString& message)
{
    put(message + "\n");
}


void LuaConsoleViewImpl::putPrompt()
{
    if(chunk.isEmpty()){
        put("> ");
    } else {
        put(">> ");
    }        
    inputColumnOffset = textCursor().columnNumber();
}


QString LuaConsoleViewImpl::getInputString()
{
    auto doc = document();
    auto line = doc->findBlockByLineNumber(doc->lineCount() - 1).text();
    line.remove(0, inputColumnOffset);
    return line;
}


void LuaConsoleViewImpl::setInputString(const QString& command)
{
    if(getInputString() == command){
        return;
    }

    auto cursor = textCursor();
    cursor.movePosition(QTextCursor::End);
    cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
    cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, inputColumnOffset);
    cursor.removeSelectedText();
    cursor.insertText(command);
    moveCursor(QTextCursor::End);
}


void LuaConsoleViewImpl::printResults(lua_State* L)
{
    int n = lua_gettop(L);
    if(n > 0){  /* any result to be printed? */
        luaL_checkstack(L, LUA_MINSTACK, "too many results to print");
        lua_getglobal(L, "print");
        lua_insert(L, 1);
        if(lua_pcall(L, n, 0, 0) != LUA_OK){
            putln(lua_pushfstring(L, "error calling 'print' (%s)", lua_tostring(L, -1)));
        }
    }
}


void LuaConsoleViewImpl::fixLine()
{
    bool isFirstLine = chunk.isEmpty();
    if(!isFirstLine){
        chunk.append("\n");
    }
    auto line = getInputString();
    chunk.append(line);
    addToHistory(line);

    put("\n"); // This must be done after getInputString().

    int status = -1;

    bool isIncomplete = true;

    auto interpreter = LuaInterpreter::mainInstance();
    auto L = interpreter->state();
    
    interpreter->beginRedirect(os);

    if(isFirstLine){
        auto retline = QString("return %1;").arg(chunk);
        auto buff = retline.toUtf8();
        status = luaL_loadbuffer(L, buff.data(), buff.size(), "=console");
        if(status == LUA_OK){
            isIncomplete = false;
        } else {
            lua_pop(L, 1);
        }
    }

    if(isIncomplete){
        auto buff = chunk.toUtf8();
        status = luaL_loadbuffer(L, buff.data(), buff.size(), "=console");
        if(status == LUA_OK){
            isIncomplete = false;
        } else if(status == LUA_ERRSYNTAX) {
            const string msg(lua_tostring(L, -1));
            static const regex eofmark(".*<eof>");
            if(regex_match(msg, eofmark)){
                lua_pop(L, 1);
                isIncomplete = true;
            } else {
                isIncomplete = false;
            }
        }
    }

    if(!isIncomplete){
        if(status == LUA_OK){
            status = lua_pcall(L, 0, LUA_MULTRET, 0);
        }
        if(status == LUA_OK){
            printResults(L);
        } else {
            putln(lua_tostring(L, -1));
            lua_pop(L, 1);
        }
        chunk.clear();
    }

    interpreter->endRedirect();
        
    putPrompt();
}


void LuaConsoleViewImpl::addToHistory(const QString& command)
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


QString LuaConsoleViewImpl::getPrevHistoryEntry()
{
    if(!history.empty()){
        if(histIter != history.begin()){
            --histIter;
        }
        return *histIter;
    }
    return QString();
}


QString LuaConsoleViewImpl::getNextHistoryEntry()
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
