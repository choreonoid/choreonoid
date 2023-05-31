#include "MessageOut.h"
#include <sstream>
#include <vector>
#include <mutex>
#include <memory>

using namespace std;
using namespace cnoid;

namespace {

mutex sinkMutex;

class MessageOutStreamBuf : public std::stringbuf
{
public:
    MessageOut& mout;
    MessageOutStreamBuf(MessageOut& mout)
        : mout(mout) { }
    virtual int sync() override {
        mout.put(str());
        return 0;
    }
};

class MessageOutErrorStreamBuf : public std::stringbuf
{
public:
    MessageOut& mout;
    MessageOutErrorStreamBuf(MessageOut& mout)
        : mout(mout) { }
    virtual int sync() override {
        mout.putError(str());
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

class MessageOut::Impl
{
public:
    MessageOut* self;
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

    unique_ptr<MessageOutStreamBuf> streamBuf;
    unique_ptr<ostream> cout;
    unique_ptr<MessageOutErrorStreamBuf> errorStreamBuf;
    unique_ptr<ostream> cerr;

    Impl(MessageOut* self);
};

}


MessageOut* MessageOut::master()
{
    static MessageOutPtr instance = new MessageOut;
    return instance;
}


MessageOut* MessageOut::interactive()
{
    static MessageOutPtr instance = new MessageOut;
    return instance;
}


MessageOut::MessageOut()
{
    impl = new Impl(this);
}


MessageOut::Impl::Impl(MessageOut* self)
    : self(self)
{
    isPendingMode = false;
    hasErrors = false;
}


MessageOut::~MessageOut()
{
    delete impl;
}


void MessageOut::clearSinks()
{
    lock_guard<mutex> lock(sinkMutex);
    impl->sinks.clear();
}
    

MessageOut::SinkHandle MessageOut::addSink(std::function<void(const std::string& message, int type)> func)
{
    lock_guard<mutex> lock(sinkMutex);
    impl->sinks.push_back(new Sink(func));
    return impl->sinks.back();
}


void MessageOut::removeSink(SinkHandle sink)
{
    lock_guard<mutex> lock(sinkMutex);
    for(auto it = impl->sinks.begin(); it != impl->sinks.end(); ++it){
        if(*it == sink){
            impl->sinks.erase(it);
            break;
        }
    }
}


void MessageOut::put(const std::string& message, int type)
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


void MessageOut::putln(const std::string& message, int type)
{
    put(message + "\n", type);
}


std::ostream& MessageOut::cout()
{
    lock_guard<mutex> lock(sinkMutex);
    if(!impl->cout){
        impl->streamBuf = make_unique<MessageOutStreamBuf>(*this);
        impl->cout = make_unique<ostream>(impl->streamBuf.get());
    }
    return *impl->cout;
}


std::ostream& MessageOut::cerr()
{
    lock_guard<mutex> lock(sinkMutex);
    if(!impl->cout){
        impl->errorStreamBuf = make_unique<MessageOutErrorStreamBuf>(*this);
        impl->cerr = make_unique<ostream>(impl->errorStreamBuf.get());
    }
    return *impl->cerr;
}


bool MessageOut::hasErrors() const
{
    return impl->hasErrors;
}


void MessageOut::setPendingMode(bool on)
{
    lock_guard<mutex> lock(sinkMutex);
    impl->isPendingMode = on;
}
    

void MessageOut::flushPendingMessages()
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
