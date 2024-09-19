#include "MessageOut.h"
#include <streambuf>
#include <ostream>
#include <vector>
#include <mutex>
#include <memory>

using namespace std;
using namespace cnoid;

namespace {

mutex sinkMutex;

class MessageOutStreamBuf : public std::basic_streambuf<char>
{
public:
    MessageOutStreamBuf(MessageOut& mout, int messageType);

    virtual int_type overflow(int_type c) override;
    virtual int sync() override;

    MessageOut& mout;
    int messageType;
    vector<char> buf;
};

class Sink : public Referenced
{
public:
    std::function<void(const std::string& message, int type)> messageFunc;
    std::function<void(const std::string& message, int type)> notifyFunc;
    std::function<void()> flushFunc;

    Sink(const std::function<void(const std::string& message, int type)>& messageFunc,
         const std::function<void(const std::string& message, int type)>& notifyFunc,
         const std::function<void()>& flushFunc)
        : messageFunc(messageFunc), notifyFunc(notifyFunc), flushFunc(flushFunc) { }
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

    ostream* direct_cout;
    unique_ptr<ostream> cout;
    unique_ptr<MessageOutStreamBuf> streamBuf;
    ostream* direct_cerr;
    unique_ptr<ostream> cerr;
    unique_ptr<MessageOutStreamBuf> errorStreamBuf;

    Impl(MessageOut* self);
};

}


MessageOutStreamBuf::MessageOutStreamBuf(MessageOut& mout, int messageType)
    : mout(mout),
      messageType(messageType)
{
    buf.resize(4096);
    auto p = &buf.front();
    setp(p, p + buf.size());
}


MessageOutStreamBuf::int_type MessageOutStreamBuf::overflow(int_type c)
{
    sync();

    if(c != traits_type::eof()){
        buf[0] = c;
        pbump(1);
        return traits_type::not_eof(c);
    } else {
        return traits_type::eof();
    }
}


int MessageOutStreamBuf::sync()
{
    auto p = &buf.front();
    mout.put(string(p, pptr() - p), messageType);
    mout.flush();
    setp(p, p + buf.size());
    return 0;
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


MessageOut* MessageOut::nullout()
{
    static MessageOutPtr instance = new MessageOut;
    return instance;
}


MessageOut::MessageOut()
{
    impl = new Impl(this);
}


MessageOut::MessageOut
(std::function<void(const std::string& message, int type)> messageFunc,
 std::function<void(const std::string& message, int type)> notifyFunc,
 std::function<void()> flushFunc)
{
    impl = new Impl(this);
    addSink(messageFunc, notifyFunc, flushFunc);
}


MessageOut::MessageOut(std::ostream& sink)
{
    impl = new Impl(this);

    addSink(
        [&sink](const std::string& message, int /* type */){ sink << message; },
        [](const std::string& /* message */, int /* type */){ },
        [&sink]{ sink.flush(); });
    
    impl->direct_cout = &sink;
    impl->direct_cerr = &sink;
}


MessageOut::Impl::Impl(MessageOut* self)
    : self(self)
{
    isPendingMode = false;
    hasErrors = false;
    direct_cout = nullptr;
    direct_cerr = nullptr;
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
    

MessageOut::SinkHandle MessageOut::addSink
(std::function<void(const std::string& message, int type)> messageFunc,
 std::function<void(const std::string& message, int type)> notifyFunc,
 std::function<void()> flushFunc)
{
    lock_guard<mutex> lock(sinkMutex);
    impl->sinks.push_back(new Sink(messageFunc, notifyFunc, flushFunc));
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
            sink->messageFunc(message, type);
        }
    } else {
        impl->pendingMessages.emplace_back(message, type);
    }
}


void MessageOut::putln(const std::string& message, int type)
{
    put(message + "\n", type);
    flush();
}


void MessageOut::notify(const std::string& message, int type)
{
    putln(message, type);
    for(auto& sink : impl->sinks){
        sink->notifyFunc(message, type);
    }
}


void MessageOut::flush()
{
    if(!impl->isPendingMode){
        for(auto& sink : impl->sinks){
            sink->flushFunc();
        }
    }
}


std::ostream& MessageOut::cout()
{
    lock_guard<mutex> lock(sinkMutex);
    if(impl->direct_cout){
        return *impl->direct_cout;
    }
    if(!impl->cout){
        impl->streamBuf = make_unique<MessageOutStreamBuf>(*this, Normal);
        impl->cout = make_unique<ostream>(impl->streamBuf.get());
    }
    return *impl->cout;
}


std::ostream& MessageOut::cerr()
{
    lock_guard<mutex> lock(sinkMutex);
    if(impl->direct_cerr){
        return *impl->direct_cerr;
    }
    if(!impl->cout){
        impl->errorStreamBuf = make_unique<MessageOutStreamBuf>(*this, Error);
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
            sink->messageFunc(message.text, message.type);
            sink->flushFunc();
        }
    }
    impl->pendingMessages.clear();
    impl->pendingMessages.shrink_to_fit();
}
