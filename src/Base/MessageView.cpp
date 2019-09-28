/**
   @author Shin'ichiro Nakaoka
*/

#include "MessageView.h"
#include "MainWindow.h"
#include "ViewManager.h"
#include "InfoBar.h"
#include "Item.h"
#include "TextEdit.h"
#include <QBoxLayout>
#include <QMessageBox>
#include <QCoreApplication>
#include <QThread>
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <stack>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace iostreams = boost::iostreams;

namespace {

MessageView* messageView = 0;

const bool PUT_COUT_TOO = false;

class TextSink : public iostreams::sink
{
public:
    TextSink(MessageViewImpl* messageViewImpl, bool doFlush);
    std::streamsize write(const char* s, std::streamsize n);

    MessageViewImpl* messageViewImpl;
    QTextEdit& textEdit;
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
    MessageViewEvent(const QString& message, bool doLF, bool doNotify, bool doFlush)
        : QEvent(QEvent::User),
          command(MV_PUT),
          message(message),
          doLF(doLF),
          doNotify(doNotify),
          doFlush(doFlush) {
    }

    MessageViewEvent(MvCommand command)
        : QEvent(QEvent::User),
          command(command) {

    }
        
    MvCommand command;
    QString message;
    bool doLF;
    bool doNotify;
    bool doFlush;
};

int flushingRef = 0;
Signal<void()> sigFlushFinished_;

class TextEditEx : public TextEdit
{
public:
    MessageViewImpl* viewImpl;
    TextEditEx(MessageViewImpl* viewImpl) : viewImpl(viewImpl) { }
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void resizeEvent(QResizeEvent* event);
    bool isLatestMessageVisible();
};


}

namespace cnoid {

class MessageViewImpl
{
public:
    MessageView* self;

    Qt::HANDLE mainThreadId;
        
    TextEditEx textEdit;
    QTextCursor cursor;
    QTextCharFormat orgCharFormat;
    QTextCharFormat currentCharFormat;
    QColor orgForeColor;
    QColor orgBackColor;

    TextSink textSink;
    iostreams::stream_buffer<TextSink> sbuf;
    std::ostream os;
        
    TextSink textSink_flush;
    iostreams::stream_buffer<TextSink> sbuf_flush;
    std::ostream os_flush;
        
    std::stack<StdioInfo> stdios;
    bool exitEventLoopRequested;

    Signal<void(const std::string& text)> sigMessage;

    MessageViewImpl(MessageView* self);

    void put(const QString& message, bool doLF, bool doNotify, bool doFlush);
    void put(int type, const QString& message, bool doLF, bool doNotify, bool doFlush);
    void doPut(const QString& message, bool doLF, bool doNotify, bool doFlush);
    void handleMessageViewEvent(MessageViewEvent* event);
    void flush();
    void doClear();
    void clear();

    void insertPlainText(const QString& message, bool doLF);
    bool paramtoInt(const QString& txt, vector<int>& n);
    int setdefault1(const vector<int>& n);
    int setdefault0(const vector<int>& n);
    void inttoColor(int n, QColor& col);
    void textProperties(const vector<int>& n);
    void escapeSequence(QString& txt);
};

}


TextSink::TextSink(MessageViewImpl* messageViewImpl, bool doFlush)
    : messageViewImpl(messageViewImpl),
      textEdit(messageViewImpl->textEdit),
      doFlush(doFlush)
{

}


std::streamsize TextSink::write(const char* s, std::streamsize n)
{
    messageViewImpl->put(QString::fromLocal8Bit(s, n), false, false, doFlush);
    //messageViewImpl->put(QString::fromUtf8(s, n), false, false, doFlush);
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
    

void MessageView::initializeClass(ExtensionManager* ext)
{
    messageView = ext->viewManager().registerClass<MessageView>(
        "MessageView", N_("Message"), ViewManager::SINGLE_DEFAULT);
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
    impl = new MessageViewImpl(this);
}


MessageViewImpl::MessageViewImpl(MessageView* self) :
    self(self),
    mainThreadId(QThread::currentThreadId()),
    textEdit(this),
    textSink(this, false),
    sbuf(textSink),
    os(&sbuf),
    textSink_flush(this, true),
    sbuf_flush(textSink_flush),
    os_flush(&sbuf_flush)
{
    self->setDefaultLayoutArea(View::BOTTOM);

    textEdit.setObjectName("TextEdit");
    textEdit.setFrameShape(QFrame::NoFrame);
    //textEdit.setReadOnly(true);
    textEdit.setTextInteractionFlags(
        Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    textEdit.setWordWrapMode(QTextOption::WrapAnywhere);
    textEdit.setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);

    QHBoxLayout* layout = new QHBoxLayout();
    layout->addWidget(&textEdit);
    self->setLayout(layout);

    textEdit.moveCursor(QTextCursor::End);
    cursor = textEdit.textCursor();
    
    QFont font("monospace");
    font.setStyleHint(QFont::TypeWriter);
    textEdit.setFont(font);

    orgForeColor = textEdit.palette().color(QPalette::Text);
    orgBackColor = textEdit.palette().color(QPalette::Base);
    
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


void MessageView::put(const char* message, int type)
{
    impl->put(type, message, false, false, false);
}


void MessageView::put(const std::string& message, int type)
{
    impl->put(type, message.c_str(), false, false, false);
}


void MessageView::put(const QString& message, int type)
{
    impl->put(type, message, false, false, false);
}


void MessageView::putln()
{
    impl->put(QString(), true, false, false);
}


void MessageView::putln(const std::string& message, int type)
{
    impl->put(type, message.c_str(), true, false, false);
}


void MessageView::putln(const char* message, int type)
{
    impl->put(type, message, true, false, false);
}


void MessageView::putln(const QString& message, int type)
{
    impl->put(type, message, true, false, false);
}


void MessageView::notify(const std::string& message, int type)
{
    impl->put(type, message.c_str(), true, true, false);
}


void MessageView::notify(const char* message, int type)
{
    impl->put(type, message, true, true, false);
}


void MessageView::notify(const QString& message, int type)
{
    impl->put(type, message, true, true, false);
}


//! \deprecated
void MessageView::put(int type, const char* message)
{
    impl->put(type, message, false, false, false);
}


//! \deprecated
void MessageView::put(int type, const std::string& message)
{
    impl->put(type, message.c_str(), false, false, false);
}


//! \deprecated
void MessageView::put(int type, const QString& message)
{
    impl->put(type, message, false, false, false);
}


//! \deprecated
void MessageView::putln(int type, const char* message)
{
    impl->put(type, message, true, false, false);
}


//! \deprecated
void MessageView::putln(int type, const std::string& message)
{
    impl->put(type, message.c_str(), true, false, false);
}


//! \deprecated
void MessageView::putln(int type, const QString& message)
{
    impl->put(type, message, true, false, false);
}


void MessageViewImpl::put(const QString& message, bool doLF, bool doNotify, bool doFlush)
{
    if(QThread::currentThreadId() == mainThreadId){
        doPut(message, doLF, doNotify, doFlush);
    } else {
        MessageViewEvent* event = new MessageViewEvent(message, doLF, doNotify, doFlush);
        QCoreApplication::postEvent(self, event, Qt::NormalEventPriority);
    }
}


void MessageViewImpl::put(int type, const QString& message, bool doLF, bool doNotify, bool doFlush)
{
    if(type == MessageView::NORMAL){
        put(message, doLF, doNotify, doFlush);
    } else {
        // add the escape sequence to make the text red
        QString highlighted("\033[31m");
        if(type == MessageView::ERROR){
            highlighted.append("Error: ");
        } else if(type == MessageView::WARNING){
            highlighted.append("Warning: ");
        }
        highlighted.append(message);
        highlighted.append("\033[0m");

        put(highlighted, doLF, doNotify, doFlush);
    }
}


void MessageViewImpl::doPut(const QString& message, bool doLF, bool doNotify, bool doFlush)
{
    bool isLatestMessageVisible = textEdit.isLatestMessageVisible();
    if(isLatestMessageVisible){
        textEdit.moveCursor(QTextCursor::End);
    }

    QString txt(message);
    while(true){
        int i = txt.indexOf("\x1b");
        if(i < 0){
            break;
        }
        if(i > 0){
            insertPlainText(txt.left(i), false);
        }
        txt = txt.mid(++i);
        escapeSequence(txt);
    }
    insertPlainText(txt, doLF);

    if(isLatestMessageVisible){
        textEdit.ensureCursorVisible();
    }
    
    if(PUT_COUT_TOO){
        cout << message.toStdString();
        if(doLF){
            cout << endl;
        }
    }

    if(!sigMessage.empty()){
        std::string text(message.toStdString());
        if(doLF){
            text += "\n";
        }
        sigMessage(boost::ref(text));
    }
    
    if(doNotify){
        InfoBar::instance()->notify(message);
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


void MessageViewImpl::handleMessageViewEvent(MessageViewEvent* event)
{
    switch(event->command){
    case MV_PUT:
        doPut(event->message, event->doLF, event->doNotify, event->doFlush);
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
    QTextCursor cursor = impl->textEdit.textCursor();
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


void MessageViewImpl::flush()
{
    if(QThread::currentThreadId() == mainThreadId){
        ++flushingRef;
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents|QEventLoop::ExcludeSocketNotifiers, 1.0);
        --flushingRef;
        if(flushingRef == 0){
            sigFlushFinished_();
        }
    }
}


SignalProxy<void(const std::string& text)> MessageView::sigMessage()
{
    return impl->sigMessage;
}


bool MessageView::isFlushing()
{
    return (flushingRef > 0);
}


SignalProxy<void()> MessageView::sigFlushFinished()
{
    return sigFlushFinished_;
}


void MessageViewImpl::doClear()
{
    textEdit.clear();
}


void MessageView::clear()
{
    impl->clear();
}


void MessageViewImpl::clear()
{
    if(QThread::currentThreadId() == mainThreadId){
        doClear();
    } else {
        QCoreApplication::postEvent(self, new MessageViewEvent(MV_CLEAR), Qt::NormalEventPriority);
    }
}


void MessageViewImpl::insertPlainText(const QString& message, bool doLF)
{
    if(!doLF){
        cursor.insertText(message);
    } else {
        cursor.insertText(message + "\n");
    }
}


bool MessageViewImpl::paramtoInt(const QString& txt, vector<int>& n)
{
    QStringList stringlist = txt.split(';');
    for(int j=0; j<stringlist.size(); j++){
        bool ret;
        int n0 = stringlist[j].toInt(&ret);
        if(ret){
            n.push_back(n0);
        } else {
            return false;
        }
    }
    return true;
}


int MessageViewImpl::setdefault1(const vector<int>& n)
{
    if(n.size()){
        return n[0];
    } else {
        return 1;
    }
}


int MessageViewImpl::setdefault0(const vector<int>& n)
{
    if(n.size()){
        return n[0];
    } else {
        return 0;
    }
}


void MessageViewImpl::inttoColor(int n, QColor& col)
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


void MessageViewImpl::textProperties(const vector<int>& n)
{
    QColor col;
    for(size_t i=0; i < n.size(); i++){
        switch(n[i]){
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
            inttoColor(n[i]-30, col);
            currentCharFormat.setForeground(col);
            break;
        case 38:
        {
            size_t j = i + 1;
            if(j < n.size()){
                if(n[j] == 2){  // RGB color specification
                    if(j + 3 < n.size()){
                        col = QColor(n[j+1], n[j+2], n[j+3]);
                        currentCharFormat.setForeground(col);
                        i += 4;
                    }
                }else if(n[j] == 5){  // Color id specification
                    if(j + 1 < n.size()){
                        inttoColor(n[j+1], col);
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
            inttoColor(n[i]-40, col);
            currentCharFormat.setBackground(col);
            break;
        case 48:
        {
            size_t j = i + 1;
            if(j < n.size()){
                if(n[j] == 2){  // Background color specified by RGB
                    if(j + 3 <n.size()){
                        col = QColor(n[j+1], n[j+2], n[j+3]);
                        currentCharFormat.setBackground(col);
                        i += 4;
                    }
                }else if(n[j] == 5){  // Background color specified by id
                    if(j + 1 <n.size()){
                        inttoColor(n[j+1], col);
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
            inttoColor(n[i]-82, col);
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
            inttoColor(n[i]-92, col);
            currentCharFormat.setBackground(col);
            break;
        }
    }
    cursor.setCharFormat(currentCharFormat);
    textEdit.setTextCursor(cursor);
}


void MessageViewImpl::escapeSequence(QString& txt)
{
    if(!txt.startsWith("[")){
        return;
    }

    int i = txt.indexOf(QRegExp("[@A-z`]"), 1);
    if(i < 0){
        return;
    }
    QChar c = txt.at(i);
    vector<int> n;
    if(i > 1){
        if(!paramtoInt(txt.mid(1, i-1), n)){
            return;
        }
    }
    
    if(c=='@'){  // Space insertion
        int nn=setdefault1(n);
        QString sp(nn,' ');
        insertPlainText(sp, false);
    }else if(c=='A' || c=='k'){  // Cursor up
        int nn=setdefault1(n);
        cursor.movePosition(QTextCursor::Up, QTextCursor::MoveAnchor, nn);
        textEdit.setTextCursor(cursor);
    }else if(c=='B' || c=='e'){  // Cursor down
        int nn=setdefault1(n);
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, nn);
        textEdit.setTextCursor(cursor);
    }else if(c=='C' || c=='a'){  // Cursor right
        int nn=setdefault1(n);
        cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, nn);
        textEdit.setTextCursor(cursor);
    }else if(c=='D'  || c=='j'){  // Cursor left
        int nn=setdefault1(n);
        cursor.movePosition(QTextCursor::Left, QTextCursor::MoveAnchor, nn);
        textEdit.setTextCursor(cursor);
    }else if(c=='E'){  // Move the cursor to the first column of the next line
        int nn=setdefault1(n);
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, nn);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        textEdit.setTextCursor(cursor);
    }else if(c=='F'){  // Move the cursor the the first column of the previous line
        int nn=setdefault1(n);
        cursor.movePosition(QTextCursor::Up, QTextCursor::MoveAnchor, nn);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        textEdit.setTextCursor(cursor);
    }else if(c=='G'){  // Cursor right by nn
        int nn=setdefault1(n);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, nn-1);
        textEdit.setTextCursor(cursor);
    }else if(c=='H' || c=='f'){  // Cursor down by nn0 and right by nn1
        int nn0, nn1;
        switch(n.size()){
        case 0:
            nn0 = nn1 = 1;
            break;
        case 1:
            nn0 = n[0];
            nn1 = 1;
            break;
        case 2 :
        default :
            nn0 = n[0];
            nn1 = n[1];
        }
        cursor.movePosition(QTextCursor::Start, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, nn0-1);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, nn1-1);
        textEdit.setTextCursor(cursor);
    }else if(c=='J'){
        int nn=setdefault0(n);
        switch(nn){
        case 0:  // Erase from the cursor position to the end
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::End, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        case 1:  // Erase from the beginning to the cursor position
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::Start, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        case 2:  // Erase all
            textEdit.clear();
            break;
        default:
            break;
        }
    }else if(c=='K'){
        int nn=setdefault0(n);
        switch(nn){
        case 0:  // Erase from the cursor position to the end of the line
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        case 1:  // Erase from the start of the line to the cursor position
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        case 2:  // Erase a line
            cursor.clearSelection();
            cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
            cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);
            cursor.removeSelectedText();
            break;
        default:
            break;
        }
    }else if(c=='L'){  // Insert empty line
        int nn=setdefault1(n);
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        for(int i=0; i<nn; i++)
            textEdit.insertPlainText("\n");
    }else if(c=='M'){  // Erase nn lines
        int nn=setdefault1(n);
        cursor.clearSelection();
        cursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Down, QTextCursor::KeepAnchor, nn);
        cursor.removeSelectedText();
    }else if(c=='P'){  // Delete nn characters
        int nn=setdefault1(n);
        cursor.clearSelection();
        cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, nn);
        cursor.removeSelectedText();
        //}else if(c=='S'){  // Scroll up nn lines
        //	int nn=setdefault1(n);
        //}else if(c=='T'){  // Scroll down nn lines
        //	int nn=setdefault1(n);
    }else if(c=='X'){  // Make nn characters blank
        int nn=setdefault1(n);
        cursor.clearSelection();
        cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, nn);
        QString sp(nn,' ');
        cursor.insertText(sp);
        //}else if(c=='Z'){  // Move the cursor to nn previous tab stop
        //	int nn=setdefault1(n);
        //}else if(c=='a'){   // Same as 'C'
        //}else if(c=='c'){  // Terminal characteristics
        //	int nn=setdefault0(n);
    }else if(c=='d'){  // Move the cursor to the nn line without changing the current column
        int nn=setdefault1(n);
        int i = cursor.columnNumber();
        cursor.movePosition(QTextCursor::Start, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, nn-1);
        cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, i);
        textEdit.setTextCursor(cursor);
        //}else if(c=='e'){  // Same as 'B'
        //}else if(c=='f'){  // Same as 'H'
        //}else if(c=='g'){  // Delete tab stop
        //	int nn=setdefault1(0);
        //	switch(nn){
        //	case 0:  // Delete one
        //		break;
        //	case 3:  // Delete all
        //		break;
        //	}
        //}else if(c=='h'){  // Set mode
        //}else if(c=='i'){  // Print mode
        //}else if(c=='j'){  // Same as 'D'
        //}else if(c=='k'){  // Same as 'A'
        //}else if(c=='l'){  // Mode release
    }else if(c=='m'){  // Set character attribute
        if(!n.size()){
            currentCharFormat = orgCharFormat;
            cout << "!n.size()" << endl;
            textEdit.setCurrentCharFormat(currentCharFormat);            
            return;
        }else{
            textProperties(n);
        }
        //}else if(c=='n'){  // State of the terminal
        //}else if(c=='r'){  // Set up and down margin
        //}else if(c=='s'){  // Save cursor position
        //}else if(c=='t'){  // Window operation
        //}else if(c=='u'){  // Restore saved cursor position
    }
    txt = txt.mid(++i);
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
        QMessageBox::question(MainWindow::instance(), caption, message,
                              QMessageBox::Ok | QMessageBox::Cancel);
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
