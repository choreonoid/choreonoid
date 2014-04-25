/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SIGNAL_PROXY_H_INCLUDED
#define CNOID_UTIL_SIGNAL_PROXY_H_INCLUDED

#include <boost/signals.hpp>

namespace cnoid {

template <class SignalType>
class SignalProxy
{
public:
    inline SignalProxy() : signal(0) { }
    inline SignalProxy(SignalType& signal) : signal(&signal) { }
    inline SignalProxy(const SignalProxy& org) : signal(org.signal) { }

    inline boost::signals::connection connect(
        typename SignalType::slot_function_type f,
        boost::signals::connect_position at = boost::signals::at_back){
        if(signal){
            return signal->connect(f, at);
        } else {
            return boost::signals::connection();
        }
    };

    template<typename Slot> void disconnect(const Slot& slot) {
        if(signal){
            signal->template disconnect<Slot>(slot);
        }
    }

private:
    SignalProxy& operator=(const SignalProxy& rhs) { } // disabled
    SignalType* signal;
};

}

#endif
