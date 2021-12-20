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

class CNOID_EXPORT MessageView : public View
{
public:
    static void postMessageBeforeInitialization(const std::string& message, int type = Normal);
    static void initializeClass(ExtensionManager* ext);

    static MessageView* mainInstance();
    static MessageView* instance();
      
    MessageView();
    ~MessageView();

#if defined(_WIN32) && defined(ERROR)
#undef ERROR
#endif

    enum MessageType {
        Normal, Error, Warning, Highlight,
        // deprecated
        NORMAL = Normal, ERROR = Error, WARNING = Warning, HIGHLIGHT = Highlight
    };

    void put(const std::string& message, int type = Normal);
    void put(std::string&& message, int type = Normal);
    void put(const char* message, int type = Normal);
    void put(const QString& message, int type = Normal);

    void putln(const std::string& message, int type = Normal);
    void putln(std::string&& message, int type = Normal);
    void putln(const char* message, int type = Normal);
    void putln(const QString& message, int type = Normal);
    void putln();

    void notify(const std::string& message, int type = Normal);
    void notify(std::string&& message, int type = Normal);
    void notify(const char* message, int type = Normal);
    void notify(const QString& message, int type = Normal);

    [[deprecated("Use put(const std::string& message, int type = Normal)")]]
    void put(int type, const std::string& message);
    [[deprecated("Use put(const char* message, int type = Normal)")]]
    void put(int type, const char* message);
    [[deprecated("Use put(const QString& message, int type = Normal)")]]
    void put(int type, const QString& message);
    
    [[deprecated("Use putln(const std::string& message, int type = Normal)")]]
    void putln(int type, const std::string& message);
    [[deprecated("Use putln(const char* message, int type = Normal)")]]
    void putln(int type, const char* message);
    [[deprecated("Use putln(const QString& message, int type = Normal)")]]
    void putln(int type, const QString& message);

    int currentColumn();
        
    void flush();
    static bool isFlushing();
    
    void clear();
      
    std::ostream& cout(bool doFlush = true);

    [[deprecated]]
    void beginStdioRedirect();
    [[deprecated]]
    void endStdioRedirect();

    SignalProxy<void(const std::string& text)> sigMessage();

    bool hasErrorMessages() const;

    std::string messages() const;

    class Impl;

protected:
    virtual bool event(QEvent* e);

private:
    Impl* impl;
};

#ifndef CNOID_BASE_MVOUT_DECLARED
#define CNOID_BASE_MVOUT_DECLARED
CNOID_EXPORT std::ostream& mvout(bool doFlush = true);
#endif

CNOID_EXPORT void showMessageBox(const std::string& message);
CNOID_EXPORT bool showWarningDialog(const std::string& message, bool doConfirmation = false);
CNOID_EXPORT void showErrorDialog(const std::string& message);
CNOID_EXPORT bool showConfirmDialog(const std::string& caption, const std::string& message);

}

#endif
