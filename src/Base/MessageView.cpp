/**
   @author Shin'ichiro Nakaoka
*/

#include "MessageView.h"
#include "MainWindow.h"
#include "ViewManager.h"
#include "InfoBar.h"
#include "Item.h"
#include "TextEdit.h"
#include <cnoid/Tokenizer>
#include <fmt/format.h>
#include <QBoxLayout>
#include <QMessageBox>
#include <QCoreApplication>
#include <QThread>
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <stack>
#include <regex>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace iostreams = boost::iostreams;

namespace {

MessageView* messageView = nullptr;

struct PendingMessage
{
    string message;
    int type;
    PendingMessage(const string& message, int type)
        : message(message), type(type) { }
};
vector<PendingMessage> initialPendingMessages;

int flushingRef = 0;

const bool PUT_COUT_TOO = false;

class TextSink : public iostreams::sink
{
public:
    TextSink(MessageView::Impl* viewImpl, bool doFlush);
    std::streamsize write(const char* s, std::streamsize n);

    MessageView::Impl* viewImpl;
    bool doFlush;
};

struct StdioInfo {
    streambuf* cout;
    streambuf* cerr;
};

enum MvCommand { MV_PUT, MV_CLEAR };


class MessageViewEvent : public QEvent
{
public:
    MessageViewEvent(const string& message, bool doLF, bool doNotify, bool doFlush)
        : QEvent(QEvent::User),
          command(MV_PUT),
          message(message),
          doLF(doLF),
          doNotify(doNotify),
          doFlush(doFlush)
    {
        
    }

    MessageViewEvent(string&& message, bool doLF, bool doNotify, bool doFlush)
        : QEvent(QEvent::User),
          command(MV_PUT),
          message(message),
          doLF(doLF),
          doNotify(doNotify),
          doFlush(doFlush)
    {
        
    }
    
    MessageViewEvent(MvCommand command)
        : QEvent(QEvent::User),
          command(command)
    {

    }
        
    MvCommand command;
    string message;
    bool doLF;
    bool doNotify;
    bool doFlush;
};

class TextEditEx : public TextEdit
{
public:
    MessageView::Impl* viewImpl;
    TextEditEx(MessageView::Impl* viewImpl) : viewImpl(viewImpl) { }
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
    bool isLatestMessageVisible();
};


}

namespace cnoid {

class MessageView::Impl
{
public:
    MessageView* self;

    Qt::HANDLE mainThreadId;
        
    QHBoxLayout* layout;
    TextEditEx* textEdit;
    QTextCursor cursor;
    QTextCharFormat orgCharFormat;
    QTextCharFormat currentCharFormat;
    QColor orgForeColor;
    QColor orgBackColor;
    vector<int> escseqParams;

    TextSink textSink;
    iostreams::stream_buffer<TextSink> sbuf;
    std::ostream os;
        
    TextSink textSink_flush;
    iostreams::stream_buffer<TextSink> sbuf_flush;
    std::ostream os_flush;
        
    std::stack<StdioInfo> stdios;
    bool exitEventLoopRequested;

    Signal<void(const std::string& text)> sigMessage;

    Impl(MessageView* self);
    void createTextEdit();

    void put(const string& message, int type, bool doLF, bool doNotify, bool doFlush, bool isMovable);
    void put(const std::string& message, bool doLF, bool doNotify, bool doFlush, bool isMovable);
    void doPut(const string& message, bool doLF, bool doNotify, bool doFlush, bool isMovable);
    void handleMessageViewEvent(MessageViewEvent* event);
    void flush();
    void doClear();
    void clear();
    void insertPlainText(const string& message, bool doLF);
    int setdefault1(const vector<int>& n);
    int setdefault0(const vector<int>& n);
    void inttoColor(int n, QColor& col);
    void applySelectGraphicRenditionCommands(const vector<int>& commands);
    void extractEscapeSequence(string& txt);
};

}


namespace {

TextSink::TextSink(MessageView::Impl* viewImpl, bool doFlush)
    : viewImpl(viewImpl),
      doFlush(doFlush)
{

}


std::streamsize TextSink::write(const char* s, std::streamsize n)
{
    viewImpl->put(string(s, n), false, false, doFlush, true);
    return n;
}


void TextEditEx::keyPressEvent(QKeyEvent* event)
{
    if ((event->modifiers().testFlag(Qt::ControlModifier))){
        switch(event->key()){
        case Qt::Key_C:
        case Qt::Key_A:
            TextEdit::keyPressEvent(event);
            break;
        }
    }
    switch(event->key()){
    case Qt::Key_Return:
        moveCursor(QTextCursor::End);
        insertPlainText("\n");
        ensureCursorVisible();
        break;
    default:
        break;
        //TextEdit::keyPressEvent(event);
    }
}


void TextEditEx::resizeEvent(QResizeEvent* event)
{
    bool isLatest = isLatestMessageVisible();
    TextEdit::resizeEvent(event);
    if(isLatest){
        moveCursor(QTextCursor::End);
    }
}


bool TextEditEx::isLatestMessageVisible()
{
    int scrollPos = getScrollPos();
    return (scrollPos > maxScrollPos() - 3 * scrollSingleStep());
}

}


void MessageView::postMessageBeforeInitialization(const std::string& message, int type)
{
    initialPendingMessages.emplace_back(message, type);
}


void MessageView::initializeClass(ExtensionManager* ext)
{
    messageView = ext->viewManager().registerClass<MessageView>(
        "MessageView", N_("Message"), ViewManager::SINGLE_DEFAULT);

    if(!initialPendingMessages.empty()){
        for(auto& m : initialPendingMessages){
            messageView->putln(m.message, m.type);
        }
        initialPendingMessages.clear();
        initialPendingMessages.shrink_to_fit();
    }
}


/**
   @return The main instance of MessageView.
*/
MessageView* MessageView::instance()
{
    return messageView;
}


/**
   Obsolete. Please use MessageView::instance().
*/
MessageView* MessageView::mainInstance()
{
    return messageView;
}


MessageView::MessageView()
{
    impl = new Impl(this);
}


MessageView::Impl::Impl(MessageView* self) :
    self(self),
    mainThreadId(QThread::currentThreadId()),
    textEdit(nullptr),
    textSink(this, false),
    sbuf(textSink),
    os(&sbuf),
    textSink_flush(this, true),
    sbuf_flush(textSink_flush),
    os_flush(&sbuf_flush)
{
    self->setDefaultLayoutArea(View::BOTTOM);

    layout = new QHBoxLayout;
    self->setLayout(layout);

    createTextEdit();
}


void MessageView::Impl::createTextEdit()
{
    if(textEdit){
        delete textEdit;
    }
    textEdit = new TextEditEx(this);

    textEdit->setObjectName("TextEdit");
    textEdit->setFrameShape(QFrame::NoFrame);
    //textEdit->setReadOnly(true);
    textEdit->setTextInteractionFlags(
        Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    textEdit->setWordWrapMode(QTextOption::WrapAnywhere);
    textEdit->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);

    layout->addWidget(textEdit);

    QFont font("monospace");
    font.setStyleHint(QFont::TypeWriter);
    textEdit->setFont(font);

    orgForeColor = textEdit->palette().color(QPalette::Text);
    orgBackColor = textEdit->palette().color(QPalette::Base);

    cursor = textEdit->textCursor();
    currentCharFormat = cursor.charFormat();
    orgCharFormat = currentCharFormat;
    orgCharFormat.setForeground(orgForeColor);
    orgCharFormat.setBackground(orgBackColor);
}


MessageView::~MessageView()
{
    delete impl;
}


std::ostream& MessageView::cout(bool doFlush)
{
    return doFlush ? impl->os_flush : impl->os;
}


void MessageView::beginStdioRedirect()
{
    /*
    StdioInfo info;
    info.cout = std::cout.rdbuf();
    info.cerr = std::cerr.rdbuf();
    impl->stdios.push(info);
    std::cout.rdbuf(impl->os.rdbuf());
    std::cerr.rdbuf(impl->os.rdbuf());
    */
}


void MessageView::endStdioRedirect()
{
    /*
    if(!impl->stdios.empty()){
        StdioInfo& info = impl->stdios.top();
        std::cout.rdbuf(info.cout);
        std::cerr.rdbuf(info.cerr);
        impl->stdios.pop();
    }
    */
}


void MessageView::Impl::doClear()
{
    /*
      The QTextEdit::clear function should be used to clear the text in a QTextEdit widget,
      but it seems that executing the function may cause memory corruption, and the process
      will crash during execution or at the end if the function is used.
      To avoid the problem, the function is currently not used here and the QTextEdit instance
      is recreated to clear the text. When the cause of this problem is found or improved,
      the QTextEdit::clear function should be used again to avoid the unnecessary cost of
      recreation.
    */
    createTextEdit();

    /*
    textEdit->clear();
    cursor = textEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    currentCharFormat = cursor.charFormat();
    orgCharFormat = currentCharFormat;
    orgCharFormat.setForeground(orgForeColor);
    orgCharFormat.setBackground(orgBackColor);
    */
}


void MessageView::clear()
{
    impl->clear();
}


void MessageView::Impl::clear()
{
    if(QThread::currentThreadId() == mainThreadId){
        doClear();
    } else {
        QCoreApplication::postEvent(self, new MessageViewEvent(MV_CLEAR), Qt::NormalEventPriority);
    }
}


void MessageView::put(const std::string& message, int type)
{
    impl->put(message, type, false, false, false, false);
}


void MessageView::put(std::string&& message, int type)
{
    impl->put(message, type, false, false, false, true);
}


void MessageView::put(const char* message, int type)
{
    impl->put(message, type, false, false, false, true);
}


void MessageView::put(const QString& message, int type)
{
    impl->put(message.toStdString(), type, false, false, false, true);
}


void MessageView::putln(const std::string& message, int type)
{
    impl->put(message, type, true, false, false, false);
}


void MessageView::putln(std::string&& message, int type)
{
    impl->put(message, type, true, false, false, true);
}


void MessageView::putln(const char* message, int type)
{
    impl->put(message, type, true, false, false, true);
}


void MessageView::putln(const QString& message, int type)
{
    impl->put(message.toStdString(), type, true, false, false, true);
}


void MessageView::putln()
{
    impl->put(string(), Normal, true, false, false, true);
}


void MessageView::notify(const std::string& message, int type)
{
    impl->put(message, type, true, true, false, false);
}


void MessageView::notify(std::string&& message, int type)
{
    impl->put(message, type, true, true, false, true);
}


void MessageView::notify(const char* message, int type)
{
    impl->put(message, type, true, true, false, true);
}


void MessageView::notify(const QString& message, int type)
{
    impl->put(message.toStdString(), type, true, true, false, true);
}


void MessageView::put(int type, const std::string& message)
{
    impl->put(message, type, false, false, false, false);
}


void MessageView::put(int type, const char* message)
{
    impl->put(message, type, false, false, false, true);
}


void MessageView::put(int type, const QString& message)
{
    impl->put(message.toStdString(), type, false, false, false, true);
}


void MessageView::putln(int type, const std::string& message)
{
    impl->put(message, type, true, false, false, false);
}


void MessageView::putln(int type, const char* message)
{
    impl->put(message, type, true, false, false, true);
}


void MessageView::putln(int type, const QString& message)
{
    impl->put(message.toStdString(), type, true, false, false, true);
}


void MessageView::Impl::put
(const string& message, int type, bool doLF, bool doNotify, bool doFlush, bool isMovable)
{
    if(type == MessageView::Normal){
        put(message, doLF, doNotify, doFlush, isMovable);
        
    } else {
        const char* prefix = "";
        if(type == Error){
            prefix = _("Error: ");
        } else if(type == Warning){
            prefix = _("Warning: ");
        }
        put(format("\x1b[31m{0} {1}\x1b[0m", prefix, message), doLF, doNotify, doFlush, true);
    }
}


void MessageView::Impl::put(const std::string& message, bool doLF, bool doNotify, bool doFlush, bool isMovable)
{
    if(QThread::currentThreadId() == mainThreadId){
        doPut(message, doLF, doNotify, doFlush, isMovable);
    } else {
        MessageViewEvent* event;
        if(isMovable){
            event = new MessageViewEvent(std::move(message), doLF, doNotify, false);
        } else {
            event = new MessageViewEvent(message, doLF, doNotify, false);
        }
        QCoreApplication::postEvent(self, event, Qt::NormalEventPriority);
    }
}


void MessageView::Impl::doPut(const string& message, bool doLF, bool doNotify, bool doFlush, bool isMovable)
{
    bool isLatestMessageVisible = textEdit->isLatestMessageVisible();
    if(isLatestMessageVisible){
        textEdit->moveCursor(QTextCursor::End);
    }

    if(PUT_COUT_TOO){
        std::cout << message;
        if(doLF){
            std::cout << endl;
        }
    }
    if(doNotify){
        InfoBar::instance()->notify(message);
    }
    if(!sigMessage.empty()){
        if(doLF){
            sigMessage(message + "\n");
        } else {
            sigMessage(message);
        }
    }

    auto pos = message.find_first_of("\x1b");
    if(pos == string::npos){
        insertPlainText(message, doLF);

    } else {
        string text;
        if(isMovable){
            text = std::move(message);
        } else {
            text = message;
        }
        while(true){
            if(pos > 0){
                insertPlainText(text.substr(0, pos), false);
            }
            text = text.substr(pos + 1);
            extractEscapeSequence(text);
            
            pos = text.find_first_of("\x1b");
            if(pos == string::npos){
                break;
            }
        }
        if(text.empty()){
            if(doLF){
                cursor.insertText("\n");
            }
        } else {
            insertPlainText(text, doLF);
        }
    }

    if(isLatestMessageVisible){
        textEdit->ensureCursorVisible();
    }
    
    if(doFlush){
        flush();
    }
}


bool MessageView::event(QEvent* e)
{
    MessageViewEvent* event = dynamic_cast<MessageViewEvent*>(e);
    if(event){
        impl->handleMessageViewEvent(event);
        return true;
    }
    return false;
}


void MessageView::Impl::handleMessageViewEvent(MessageViewEvent* event)
{
    switch(event->command){
    case MV_PUT:
        doPut(event->message, event->doLF, event->doNotify, event->doFlush, true);
        break;
    case MV_CLEAR:
        doClear();
        break;
    default:
        break;
    }
}


int MessageView::currentColumn()
{
    QTextCursor cursor = impl->textEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    return cursor.columnNumber();
}


/**
   @note Don't call this function from an expose event handler
   because it may cause a hangup of rendering.
*/
void MessageView::flush()
{
    impl->flush();
}


void MessageView::Impl::flush()
{
    ++flushingRef;
        
    QCoreApplication::processEvents(
        QEventLoop::ExcludeUserInputEvents | QEventLoop::ExcludeSocketNotifiers, 1.0);
    
    --flushingRef;
}


bool MessageView::isFlushing()
{
    return (flushingRef > 0);
}


SignalProxy<void(const std::string& text)> MessageView::sigMessage()
{
    return impl->sigMessage;
}


void MessageView::Impl::insertPlainText(const string& message, bool doLF)
{
    cursor.insertText(message.c_str());
    if(doLF){
        cursor.insertText("\n");
    }
}


void MessageView::Impl::inttoColor(int n, QColor& col)
{
    switch(n){
    case 0:
        col = QColor("black");
        return;
    case 1:
        col = QColor("red");
        return;
    case 2:
        col = QColor("green");
        return;
    case 3:
        col = QColor("yellow");
        return;
    case 4:
        col = QColor("blue");
        return;
    case 5:
        col = QColor("magenta");
        return;
    case 6:
        col = QColor("cyan");
        return;
    case 7:
        col = QColor("white");
        return;
    case 8:
        col = QColor("gray");
        return;
    case 9:
        col = QColor("darkRed");
        return;
    case 10:
        col = QColor("darkGreen");
        return;
    case 11:
        col = QColor("darkYellow");
        return;
    case 12:
        col = QColor("darkBlue");
        return;
    case 13:
        col = QColor("darkMagenta");
        return;
    case 14:
        col = QColor("darkCyan");
        return;
    case 15:
        col = QColor("darkGray");
        return;
    }
}


void MessageView::Impl::applySelectGraphicRenditionCommands(const vector<int>& commands)
{
    QColor col;
    for(size_t i=0; i < commands.size(); ++i){
        int command = commands[i];
        switch(command){
        case 0:  // Reset all
            currentCharFormat = orgCharFormat;
            break;
        case 1:  // Bold text
            currentCharFormat.setFontWeight(QFont::Bold);
            break;
        case 4:  // Underline
            currentCharFormat.setFontUnderline(true);
            break;
        case 5:  // Blinking
            break;
        case 7:  // Flipping
        case 27: // Clear flipping
        {
            QColor fore = currentCharFormat.foreground().color();
            QColor back = currentCharFormat.background().color();
            currentCharFormat.setForeground(back);
            currentCharFormat.setBackground(fore);
            break;
        }
        case 22:  // Clear bold
            currentCharFormat.setFontWeight(QFont::Normal);
            break;
        case 24:  // Clear underline
            currentCharFormat.setFontUnderline(false);
            break;
        case 25:  // Clear blinking
            break;
        case 30:  // Black
        case 31:  // Red
        case 32:  // Green
        case 33:  // Yellow
        case 34:  // Blue
        case 35:  // Magenda
        case 36:  // Cyan
        case 37:  // White
            inttoColor(command - 30, col);
            currentCharFormat.setForeground(col);
            break;
        case 38:
        {
            size_t j = i + 1;
            if(j < commands.size()){
                if(commands[j] == 2){ // RGB color specification
                    if(j + 3 < commands.size()){
                        col = QColor(commands[j+1], commands[j+2], commands[j+3]);
                        currentCharFormat.setForeground(col);
                        i += 4;
                    }
                } else if(commands[j] == 5){ // Color id specification
                    if(j + 1 < commands.size()){
                        inttoColor(commands[j+1], col);
                        currentCharFormat.setForeground(col);
                        i += 2;
                    }
                }
            }
            break;
        }
        case 39:  // Normal color
            currentCharFormat.setForeground(orgForeColor);
            break;
        case 40:  // Black background
        case 41:  // Red background
        case 42:  // Green background
        case 43:  // Yellow background
        case 44:  // Blue background
        case 45:  // Magenda background
        case 46:  // Cyan background
        case 47:  // White background
            inttoColor(command - 40, col);
            currentCharFormat.setBackground(col);
            break;
        case 48:
        {
            size_t j = i + 1;
            if(j < commands.size()){
                if(commands[j] == 2){ // Background color specified by RGB
                    if(j + 3 < commands.size()){
                        col = QColor(commands[j+1], commands[j+2], commands[j+3]);
                        currentCharFormat.setBackground(col);
                        i += 4;
                    }
                }else if(commands[j] == 5){ // Background color specified by id
                    if(j + 1 < commands.size()){
                        inttoColor(commands[j+1], col);
                        currentCharFormat.setBackground(col);
                        i += 2;
                    }
                }
            }
            break;
        }
        case 49:  // Normal background color
            currentCharFormat.setBackground(orgBackColor);
            break;
        case 90:
        case 91:
        case 92:
        case 93:
        case 94:
        case 95:
        case 96:
        case 97:
            inttoColor(command - 82, col);
            currentCharFormat.setForeground(col);
            break;
        case 100:
        case 101:
        case 102:
        case 103:
        case 104:
        case 105:
        case 106:
        case 107:
            inttoColor(command - 92, col);
            currentCharFormat.setBackground(col);
            break;
        }
    }
    cursor.setCharFormat(currentCharFormat);
    textEdit->setTextCursor(cursor);
}


void MessageView::Impl::extractEscapeSequence(string& txt)
{
    static regex escseqPattern("^\\[([0-9;]*)([A-z])");

    std::smatch match;
    if(!regex_search(txt, match, escseqPattern, std::regex_constants::format_first_only)){
        return;
    }

    int command = match.str(2)[0];

    auto& params = escseqParams;
    params.clear();
    int param0 = 1;
    int param1 = 1;
    const auto& paramString = match.str(1);
    if(!paramString.empty()){
        static Tokenizer<CharSeparator<char>> tokens(CharSeparator<char>(";"));
        tokens.assign(paramString);
        for(auto& token : tokens){
            params.push_back(std::stoi(token));
        }
        if(!params.empty()){
            param0 = params[0];
            if(params.size() >= 2){
                param1 = params[1];
            }
        }
    }

    switch(command){

    case 'm': // Set character attribute
        if(!escseqParams.empty()){
            applySelectGraphicRenditionCommands(params);
        } else {
            currentCharFormat = orgCharFormat;
            textEdit->setCurrentCharFormat(currentCharFormat);            
        }
        break;
        
    case '@': // Space insertion
        cursor.insertText(QString(param0, ' '));
        break;

    case 'A': // Cursor up
    case 'k':
        cursor.movePosition(QTextCursor::Up, QTextCursor::MoveAnchor, param0);
        textEdit->setTextCursor(cursor);
        break;

    case 'B': // Cursor down
    case 'e':
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, param0);
        textEdit->setTextCursor(cursor);
        break;
        
    case 'C': // Cursor right
    case 'a':
        cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, param0);
        textEdit->setTextCursor(cursor);
        break;

    case 'D': // Cursor left
    case 'j':
        cursor.movePosition(QTextCursor::Left, QTextCursor::MoveAnchor, param0);
        textEdit->setTextCursor(cursor);
        break;

    case 'E': // Move the cursor to the first column of the next line
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, param0);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        textEdit->setTextCursor(cursor);
        break;

    case 'F': // Move the cursor the the first column of the previous line
        cursor.movePosition(QTextCursor::Up, QTextCursor::MoveAnchor, param0);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        textEdit->setTextCursor(cursor);
        break;

    case 'G': // Cursor right by nn
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, param0 - 1);
        textEdit->setTextCursor(cursor);
        break;

    case 'H': // Cursor down by nn0 and right by nn1
    case 'f':
        cursor.movePosition(QTextCursor::Start, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, param0 - 1);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, param1 - 1);
        textEdit->setTextCursor(cursor);
        break;

    case 'J':
        switch(params.empty() ? 0 : params[0]){
        case 0: // Erase from the cursor position to the end
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::End, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        case 1: // Erase from the beginning to the cursor position
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::Start, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        case 2: // Erase all
            textEdit->clear();
            break;
        default:
            break;
        }
        break;

    case 'K':
        switch(params.empty() ? 0 : params[0]){
        case 0: // Erase from the cursor position to the end of the line
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        case 1: // Erase from the start of the line to the cursor position
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        case 2: // Erase a line
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
            cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        default:
            break;
        }
        break;

    case 'L': // Insert empty line
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        for(int i=0; i < param0; ++i){
            textEdit->insertPlainText("\n");
        }
        break;
        
    case 'M': // Erase nn lines
        cursor.clearSelection();
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Down, QTextCursor::KeepAnchor, param0);
        cursor.removeSelectedText();
        break;

    case 'P': // Delete nn characters
        cursor.clearSelection();
        cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, param0);
        cursor.removeSelectedText();
        break;

    case 'S': // Scroll up nn lines
        break;

    case 'T': // Scroll down nn lines
        break;

    case 'X': // Make nn characters blank
        cursor.clearSelection();
        cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, param0);
        cursor.insertText(QString(param0, ' '));
        break;

    case 'Z': // Move the cursor to nn previous tab stop
        break;

    case 'c': // Terminal characteristics
        break;
        
    case 'd': // Move the cursor to the nn line without changing the current column
    {        
        int column = cursor.columnNumber();
        cursor.movePosition(QTextCursor::Start, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, param0 - 1);
        cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, column);
        textEdit->setTextCursor(cursor);
        break;
    }

    case 'g': // Delete tab stop
        switch(param0){
        case 0: // Delete one
            break;
        case 3: // Delete all
            break;
        default:
            break;
        }
        break;
        
    case 'h': // Set mode
        break;

    case 'i': // Print mode
        break;

    case 'l': // Mode release
        break;

    case 'n': // State of the terminal
        break;

    case 'r': // Set up and down margin
        break;

    case 's': // Save cursor position
        break;

    case 't': // Window operation
        break;

    case 'u': // Restore saved cursor position
        break;

    default:
        break;
    }
    
    txt = txt.substr(match.length(0));
}


std::ostream& cnoid::mvout(bool doFlush)
{
    return MessageView::instance()->cout(doFlush);
}


void cnoid::showMessageBox(const QString& message)
{
    QMessageBox::information(MainWindow::instance(), _("Message"), message);
}

void cnoid::showMessageBox(const char* message)
{
    showMessageBox(QString(message));
}

    
void cnoid::showMessageBox(const std::string& message)
{
    showMessageBox(QString(message.c_str()));
}


void cnoid::showWarningDialog(const QString& message)
{
    QMessageBox::warning(MainWindow::instance(), _("Warning"), message);
}


void cnoid::showWarningDialog(const char* message)
{
    showWarningDialog(QString(message));
}


void cnoid::showWarningDialog(const std::string& message)
{
    showWarningDialog(QString(message.c_str()));
}


bool cnoid::showConfirmDialog(const QString& caption, const QString& message)
{
    QMessageBox::StandardButton clicked =
        QMessageBox::question(
            MainWindow::instance(), caption, message, QMessageBox::Ok | QMessageBox::Cancel);
    return (clicked == QMessageBox::Ok);
}


bool cnoid::showConfirmDialog(const std::string& caption, const std::string& message)
{
    return showConfirmDialog(QString(caption.c_str()), QString(message.c_str()));
}


bool cnoid::showConfirmDialog(const char* caption, const char* message)
{
    return showConfirmDialog(QString(caption), QString(message));
}
