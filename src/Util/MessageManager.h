#ifndef CNOID_UTIL_MESSAGE_MANAGER_H
#define CNOID_UTIL_MESSAGE_MANAGER_H

#include "Referenced.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MessageManager : public Referenced
{
public:
    static MessageManager* master();

    MessageManager();
    MessageManager(std::function<void(const std::string& message, int type)> sink);
    MessageManager(MessageManager* parent);
    ~MessageManager();

    enum MessageType { Normal, Highlighted, Warning, Error };

    typedef ReferencedPtr SinkHandle;

    void clearSinks();
    SinkHandle addSink(std::function<void(const std::string& message, int type)> sink);
    int numSinks() const;
    SinkHandle sinkHandle(int index);
    void removeSink(SinkHandle sink);

    void put(const std::string& message, int type);
    void putln(const std::string& message, int type);

    void put(const std::string& message){
        put(message, Normal);
    }
    void putln(const std::string& message){
        putln(message, Normal);
    }
    void putHighlighted(const std::string& message){
        put(message, Highlighted);
    }
    void putHighlightedln(const std::string& message){
        putln(message, Highlighted);
    }
    void putWarning(const std::string& message){
        put(message, Warning);
    }
    void putWarningln(const std::string& message){
        putln(message, Warning);
    }
    void putError(const std::string& message){
        put(message, Error);
    }
    void putErrorln(const std::string& message){
        putln(message, Error);
    }

    std::ostream& cout();
    std::ostream& cerr();

    void setPendingMode(bool on);
    void flushPendingMessages();

    // Used for continuous integration
    bool hasErrors() const;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MessageManager> MessageManagerPtr;

}

#endif
