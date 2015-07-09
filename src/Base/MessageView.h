/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MESSAGE_VIEW_H
#define CNOID_BASE_MESSAGE_VIEW_H

#include <cnoid/View>
#include <string>
#include <iosfwd>
#include <boost/format.hpp>
#include <QString>
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

    enum MessageType { NORMAL, ERROR, WARNING, HIGHLIGHT };

    void put(const char* message);
    void put(const std::string& message);
    void put(const boost::format& message);
    void put(const QString& message);

    void put(int type, const char* message);
    void put(int type, const std::string& message);
    void put(int type, const boost::format& message);
    void put(int type, const QString& message);
    
    void putln();
    void putln(const char* message);
    void putln(const std::string& message);
    void putln(const boost::format& message);
    void putln(const QString& message);

    void putln(int type, const char* message);
    void putln(int type, const std::string& message);
    void putln(int type, const boost::format& message);
    void putln(int type, const QString& message);

    void notify(const char* message);
    void notify(const std::string& message);
    void notify(const boost::format& message);
    void notify(const QString& message);

    /*
      int start(const char* message);
      int start(const std::string& message);
      int start(const boost::format& message);
      int start(const QString& message);

      void progress(int id, const char* message);
      void progress(int id, const std::string& message);
      void progress(int id, const boost::format& message);
      void progress(int id, const QString& message);

      void finish(int id, const char* message);
      void finish(int id, const std::string& message);
      void finish(int id, const boost::format& message);
      void finish(int id, const QString& message);
    */

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

CNOID_EXPORT void showWarningDialog(const std::string& message);
CNOID_EXPORT void showWarningDialog(const boost::format& message);
CNOID_EXPORT void showWarningDialog(const char* message);
CNOID_EXPORT void showWarningDialog(const QString& message);

CNOID_EXPORT bool showConfirmDialog(const char* caption, const char* message);
CNOID_EXPORT bool showConfirmDialog(const std::string& caption, const std::string& message);
CNOID_EXPORT bool showConfirmDialog(const QString& caption, const QString& message);
}

#endif
