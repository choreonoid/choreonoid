#include "MessageManager.h"
#include <sstream>
#include <vector>
#include <mutex>

using namespace std;
using namespace cnoid;

namespace {

mutex sinkMutex;

class MessageManagerStreamBuf : public std::stringbuf
{
public:
    MessageManager& messageManager;
    MessageManagerStreamBuf(MessageManager& messageManager)
        : messageManager(messageManager) { }
    virtual int sync() override {
        messageManager.put(str());
        return 0;
    }
};

class MessageManagerErrorStreamBuf : public std::stringbuf
{
public:
    MessageManager& messageManager;
    MessageManagerErrorStreamBuf(MessageManager& messageManager)
        : messageManager(messageManager) { }
    virtual int sync() override {
        messageManager.putError(str());
        return 0;
    }
};

class Sink : public Referenced
{
public:
    std::function<void(const std::string& message, int type)> function;

    Sink(const std::function<void(const std::string& message, int type)>& func)
        : function(func) { }
};
typedef ref_ptr<Sink> SinkPtr;
    
}

namespace cnoid {

class MessageManager::Impl
{
public:
    MessageManager* self;
    vector<SinkPtr> sinks;

    struct Message {
        string text;
        int type;
        Message(const string& text, int type)
            : text(text), type(type) { }
    };
    vector<Message> pendingMessages;

    bool isPendingMode;
    bool hasErrors;

    MessageManagerStreamBuf streamBuf;
    ostream cout;
    MessageManagerErrorStreamBuf errorStreamBuf;
    ostream cerr;

    Impl(MessageManager* self);
};

}


MessageManager* MessageManager::master()
{
    static MessageManagerPtr master_ = new MessageManager;
    return master_;
}


MessageManager::MessageManager()
{
    impl = new Impl(this);
}


MessageManager::Impl::Impl(MessageManager* self)
    : self(self),
      streamBuf(*self),
      cout(&streamBuf),
      errorStreamBuf(*self),
      cerr(&errorStreamBuf)
{
    isPendingMode = false;
    hasErrors = false;
}


MessageManager::~MessageManager()
{
    delete impl;
}


void MessageManager::clearSinks()
{
    lock_guard<mutex> lock(sinkMutex);
    impl->sinks.clear();
}
    

MessageManager::SinkHandle MessageManager::addSink(std::function<void(const std::string& message, int type)> func)
{
    lock_guard<mutex> lock(sinkMutex);
    impl->sinks.push_back(new Sink(func));
    return impl->sinks.back();
}


void MessageManager::removeSink(SinkHandle sink)
{
    lock_guard<mutex> lock(sinkMutex);
    for(auto it = impl->sinks.begin(); it != impl->sinks.end(); ++it){
        if(*it == sink){
            impl->sinks.erase(it);
            break;
        }
    }
}


void MessageManager::put(const std::string& message, int type)
{
    lock_guard<mutex> lock(sinkMutex);
    if(type == Error){
        impl->hasErrors = true;
    }
    if(!impl->isPendingMode){
        for(auto& sink : impl->sinks){
            sink->function(message, type);
        }
    } else {
        impl->pendingMessages.emplace_back(message, type);
    }
}


void MessageManager::putln(const std::string& message, int type)
{
    put(message + "\n", type);
}


std::ostream& MessageManager::cout()
{
    return impl->cout;
}


bool MessageManager::hasErrors() const
{
    return impl->hasErrors;
}


void MessageManager::setPendingMode(bool on)
{
    impl->isPendingMode = on;
}
    

void MessageManager::flushPendingMessages()
{
    lock_guard<mutex> lock(sinkMutex);
    for(auto& message : impl->pendingMessages){
        for(auto& sink : impl->sinks){
            sink->function(message.text, message.type);
        }
    }
    impl->pendingMessages.clear();
    impl->pendingMessages.shrink_to_fit();
}
