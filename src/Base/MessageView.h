/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MESSAGE_VIEW_H
#define CNOID_BASE_MESSAGE_VIEW_H

#include <cnoid/View>
#include <QString>
#include <string>
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {

class MessageViewImpl;

class CNOID_EXPORT MessageView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    static MessageView* mainInstance();
    static MessageView* instance();
      
    MessageView();
    ~MessageView();

#if defined(_WIN32) && defined(ERROR)
#undef ERROR
#endif

    enum MessageType { NORMAL, ERROR, WARNING, HIGHLIGHT };

    void put(const char* message, int type = NORMAL);
    void put(const std::string& message, int type = NORMAL);
    void put(const QString& message, int type = NORMAL);

    void putln();
    void putln(const char* message, int type = NORMAL);
    void putln(const std::string& message, int type = NORMAL);
    void putln(const QString& message, int type = NORMAL);

    void notify(const char* message, int type = NORMAL);
    void notify(const std::string& message, int type = NORMAL);
    void notify(const QString& message, int type = NORMAL);

    //! \deprecated
    void put(int type, const char* message);
    //! \deprecated
    void put(int type, const std::string& message);
    //! \deprecated
    void put(int type, const QString& message);
    
    //! \deprecated
    void putln(int type, const char* message);
    //! \deprecated
    void putln(int type, const std::string& message);
    //! \deprecated
    void putln(int type, const QString& message);

    int currentColumn();
        
    void flush();
    void clear();
      
    std::ostream& cout(bool doFlush = false);

    void beginStdioRedirect();
    void endStdioRedirect();

    SignalProxy<void(const std::string& text)> sigMessage();

    static bool isFlushing();
    static SignalProxy<void()> sigFlushFinished();

protected:
    virtual bool event(QEvent* e);

private:
    MessageViewImpl* impl;
};

#ifndef CNOID_BASE_MVOUT_DECLARED
#define CNOID_BASE_MVOUT_DECLARED
CNOID_EXPORT std::ostream& mvout(bool doFlush = false);
#endif

CNOID_EXPORT void showMessageBox(const std::string& message);
CNOID_EXPORT void showMessageBox(const char* message);
CNOID_EXPORT void showMessageBox(const QString& message);

CNOID_EXPORT void showWarningDialog(const std::string& message);
CNOID_EXPORT void showWarningDialog(const char* message);
CNOID_EXPORT void showWarningDialog(const QString& message);

CNOID_EXPORT bool showConfirmDialog(const char* caption, const char* message);
CNOID_EXPORT bool showConfirmDialog(const std::string& caption, const std::string& message);
CNOID_EXPORT bool showConfirmDialog(const QString& caption, const QString& message);

}

#endif
