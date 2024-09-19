#ifndef CNOID_UTIL_MESSAGE_OUT_H
#define CNOID_UTIL_MESSAGE_OUT_H

#include "Referenced.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MessageOut : public Referenced
{
public:
    static MessageOut* master();
    static MessageOut* interactive();
    static MessageOut* nullout();

    MessageOut();
    MessageOut(
        std::function<void(const std::string& message, int type)> messageFunc,
        std::function<void(const std::string& message, int type)> notifyFunc,
        std::function<void()> flushFunc);
    MessageOut(std::ostream& sink);
    MessageOut(MessageOut* parent);
    ~MessageOut();

    enum MessageType { Normal, Highlighted, Warning, Error };

    typedef ReferencedPtr SinkHandle;

    void clearSinks();
    SinkHandle addSink(
        std::function<void(const std::string& message, int type)> messageFunc,
        std::function<void(const std::string& message, int type)> notifyFunc,
        std::function<void()> flushFunc);
    int numSinks() const;
    SinkHandle sinkHandle(int index);
    void removeSink(SinkHandle sink);

    void put(const std::string& message, int type = Normal);
    void putln(const std::string& message, int type = Normal);
    void notify(const std::string& message, int type = Normal);

    void putHighlighted(const std::string& message){
        put(message, Highlighted);
    }
    void putHighlightedln(const std::string& message){
        putln(message, Highlighted);
    }
    void notifyHighlighted(const std::string& message){
        notify(message, Highlighted);
    }
    void putWarning(const std::string& message){
        put(message, Warning);
    }
    void putWarningln(const std::string& message){
        putln(message, Warning);
    }
    void notifyWarning(const std::string& message){
        notify(message, Warning);
    }
    void putError(const std::string& message){
        put(message, Error);
    }
    void putErrorln(const std::string& message){
        putln(message, Error);
    }
    void notifyError(const std::string& message){
        notify(message, Error);
    }

    std::ostream& cout();
    std::ostream& cerr();

    void flush();

    void setPendingMode(bool on);
    void flushPendingMessages();

    // Used for continuous integration
    bool hasErrors() const;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MessageOut> MessageOutPtr;

}

#endif
