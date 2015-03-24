/**
   @file Implementation of Signal template classes
   @note The classes are written by using the boost.signals library as a reference.
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SIGNAL_H
#define CNOID_UTIL_SIGNAL_H

#include "Referenced.h"
#include <boost/type_traits/function_traits.hpp>

#define CNOID_SIGNAL_CONCAT( X, Y ) CNOID_SIGNAL_DO_CONCAT( X, Y )
#define CNOID_SIGNAL_DO_CONCAT( X, Y ) CNOID_SIGNAL_DO_CONCAT2(X,Y)
#define CNOID_SIGNAL_DO_CONCAT2( X, Y ) X##Y

namespace cnoid {

class Connection;

namespace signal_private {

template<typename T>
struct last_value {
    typedef T result_type;
    
    template<typename InputIterator>
    T operator()(InputIterator first, InputIterator last) const {
        T value;
        while (first != last)
            value = *first++;
        return value;
    }
};


template<>
struct last_value<void> {
  public:
    template<typename InputIterator>
    void operator()(InputIterator first, InputIterator last) const{
        while (first != last)
            *first++;
    }
};


class SlotHolderBase : public Referenced
{
public:
    SlotHolderBase() : isBlocked(false) { }
    virtual ~SlotHolderBase() { }
    virtual void disconnect() = 0;
    virtual bool connected() const = 0;
    virtual void changeOrder(int orderId) = 0;

    bool isBlocked;
};


template<typename SlotType, typename ArgSetType>
class SlotCallIterator
{
    typedef typename SlotType::result_type result_type;

    SlotType* currentSlot;
    ArgSetType& args;

public:
    void seekUnblockedSlot(){
        while(currentSlot && currentSlot->isBlocked){
            currentSlot = currentSlot->next;
        }
    }
    
    SlotCallIterator(SlotType* firstSlot, ArgSetType& args)
        : currentSlot(firstSlot), args(args) {
        seekUnblockedSlot();
    }

    SlotCallIterator(const SlotCallIterator& org)
        : currentSlot(org.currentSlot), args(org.args) {
        seekUnblockedSlot();
    }

    bool operator==(const SlotCallIterator& rhs) const {
        return (currentSlot == rhs.currentSlot);
    }

    bool operator!=(const SlotCallIterator& rhs) const {
        return (currentSlot != rhs.currentSlot);
    }

    SlotCallIterator operator++(int) {
        SlotCallIterator iter(*this);
        currentSlot = currentSlot->next;
        seekUnblockedSlot();
        return iter;
    }
    
    result_type operator*() const { return args.call(currentSlot); }
};


} // namespace signal_private

class Connection
{
    ref_ptr<signal_private::SlotHolderBase> slot;

public:
    Connection() { }
    
    Connection(signal_private::SlotHolderBase* slot) : slot(slot) { }

    Connection(const Connection& org) : slot(org.slot) { }

    Connection& operator=(const Connection& rhs) {
        slot = rhs.slot;
        return *this;
    }

    void disconnect() {
        if(slot) {
            slot->disconnect();
            slot = 0;
        }
    }

    bool connected() {
        return slot && slot->connected();
    }

    void block() {
        if(slot){
            slot->isBlocked = true;
        }
    }

    void unblock() {
        if(slot){
            slot->isBlocked = false;
        }
    }

    enum Order { FIRST = 0, LAST };
    
    Connection& changeOrder(Order order) {
        if(slot){
            slot->changeOrder(order);
        }
        return *this;
    }
};

class ScopedConnection : private Connection
{
public:
    ScopedConnection() { }
    ScopedConnection(const Connection& org) : Connection(org) { }
    ~ScopedConnection() { Connection::disconnect(); }
    void reset(const Connection& c) { Connection::disconnect(); Connection::operator=(c); }
    void disconnect() { Connection::disconnect(); }
    bool connected() { return Connection::connected(); }
    void block() { Connection::block(); }
    void unblock() { Connection::unblock(); }
    ScopedConnection& changeOrder(Order order) { Connection::changeOrder(order); return *this; }

private:
    ScopedConnection(const ScopedConnection& org);
    ScopedConnection& operator=(const ScopedConnection& rhs);
};

}


#define CNOID_SIGNAL_NUM_ARGS 0
#define CNOID_SIGNAL_TEMPLATE_PARMS
#define CNOID_SIGNAL_TEMPLATE_ARGS
#define CNOID_SIGNAL_PARMS
#define CNOID_SIGNAL_ARGS
#define CNOID_SIGNAL_ARGS_AS_MEMBERS
#define CNOID_SIGNAL_COPY_PARMS
#define CNOID_SIGNAL_INIT_ARGS

#include "SignalTemplate.h"

#undef CNOID_SIGNAL_INIT_ARGS
#undef CNOID_SIGNAL_COPY_PARMS
#undef CNOID_SIGNAL_ARGS_AS_MEMBERS
#undef CNOID_SIGNAL_ARGS
#undef CNOID_SIGNAL_PARMS
#undef CNOID_SIGNAL_TEMPLATE_ARGS
#undef CNOID_SIGNAL_TEMPLATE_PARMS
#undef CNOID_SIGNAL_NUM_ARGS

#define CNOID_SIGNAL_NUM_ARGS 1
#define CNOID_SIGNAL_TEMPLATE_PARMS typename T1
#define CNOID_SIGNAL_TEMPLATE_ARGS T1
#define CNOID_SIGNAL_PARMS T1 a1
#define CNOID_SIGNAL_ARGS a1
#define CNOID_SIGNAL_ARGS_AS_MEMBERS T1 a1;
#define CNOID_SIGNAL_COPY_PARMS T1 ia1
#define CNOID_SIGNAL_INIT_ARGS :a1(ia1)

#include "SignalTemplate.h"

#undef CNOID_SIGNAL_INIT_ARGS
#undef CNOID_SIGNAL_COPY_PARMS
#undef CNOID_SIGNAL_ARGS_AS_MEMBERS
#undef CNOID_SIGNAL_ARGS
#undef CNOID_SIGNAL_PARMS
#undef CNOID_SIGNAL_TEMPLATE_ARGS
#undef CNOID_SIGNAL_TEMPLATE_PARMS
#undef CNOID_SIGNAL_NUM_ARGS

#define CNOID_SIGNAL_NUM_ARGS 2
#define CNOID_SIGNAL_TEMPLATE_PARMS typename T1, typename T2
#define CNOID_SIGNAL_TEMPLATE_ARGS T1, T2
#define CNOID_SIGNAL_PARMS T1 a1, T2 a2
#define CNOID_SIGNAL_ARGS a1, a2
#define CNOID_SIGNAL_ARGS_AS_MEMBERS T1 a1;T2 a2;
#define CNOID_SIGNAL_COPY_PARMS T1 ia1, T2 ia2
#define CNOID_SIGNAL_INIT_ARGS :a1(ia1), a2(ia2)

#include "SignalTemplate.h"

#undef CNOID_SIGNAL_INIT_ARGS
#undef CNOID_SIGNAL_COPY_PARMS
#undef CNOID_SIGNAL_ARGS_AS_MEMBERS
#undef CNOID_SIGNAL_ARGS
#undef CNOID_SIGNAL_PARMS
#undef CNOID_SIGNAL_TEMPLATE_ARGS
#undef CNOID_SIGNAL_TEMPLATE_PARMS
#undef CNOID_SIGNAL_NUM_ARGS

#define CNOID_SIGNAL_NUM_ARGS 3
#define CNOID_SIGNAL_TEMPLATE_PARMS typename T1, typename T2, typename T3
#define CNOID_SIGNAL_TEMPLATE_ARGS T1, T2, T3
#define CNOID_SIGNAL_PARMS T1 a1, T2 a2, T3 a3
#define CNOID_SIGNAL_ARGS a1, a2, a3
#define CNOID_SIGNAL_ARGS_AS_MEMBERS T1 a1;T2 a2;T3 a3;
#define CNOID_SIGNAL_COPY_PARMS T1 ia1, T2 ia2, T3 ia3
#define CNOID_SIGNAL_INIT_ARGS :a1(ia1), a2(ia2), a3(ia3)

#include "SignalTemplate.h"

#undef CNOID_SIGNAL_INIT_ARGS
#undef CNOID_SIGNAL_COPY_PARMS
#undef CNOID_SIGNAL_ARGS_AS_MEMBERS
#undef CNOID_SIGNAL_ARGS
#undef CNOID_SIGNAL_PARMS
#undef CNOID_SIGNAL_TEMPLATE_ARGS
#undef CNOID_SIGNAL_TEMPLATE_PARMS
#undef CNOID_SIGNAL_NUM_ARGS


namespace cnoid {

namespace signal_private {

template<int Arity, typename Signature, typename Combiner>
class real_get_signal_impl;

template<typename Signature, typename Combiner>
class real_get_signal_impl<0, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef Signal0<typename traits::result_type, Combiner> type;
};

template<typename Signature,typename Combiner>
class real_get_signal_impl<1, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef Signal1<typename traits::result_type,
                    typename traits::arg1_type,
                    Combiner> type;
};

template<typename Signature,typename Combiner>
class real_get_signal_impl<2, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef Signal2<typename traits::result_type,
                    typename traits::arg1_type,
                    typename traits::arg2_type,
                    Combiner> type;
};


template<typename Signature, typename Combiner>
class real_get_signal_impl<3, Signature, Combiner>
{
    typedef boost::function_traits<Signature> traits;
public:
    typedef Signal3<typename traits::result_type,
                    typename traits::arg1_type,
                    typename traits::arg2_type,
                    typename traits::arg3_type,
                    Combiner> type;
};
    
template<typename Signature, typename Combiner>
struct get_signal_impl : public real_get_signal_impl<
    (boost::function_traits<Signature>::arity), Signature, Combiner>
{

};
    
} // namespace signal_private

template<
    typename TSignature, 
    typename Combiner = signal_private::last_value<typename boost::function_traits<TSignature>::result_type>
    >
class Signal : public signal_private::get_signal_impl<TSignature, Combiner>::type
{
public:
    typedef TSignature Signature;
};


template<typename Signal> struct signal_traits { };

template<typename Signature, typename Combiner>
struct signal_traits< Signal<Signature, Combiner> >
{
    typedef Connection connection_type;
};


// for boost.signals
/*  
template<typename Signature, typename Combiner, typename Group, typename GroupCompare, typename SlotFunction>
struct signal_traits< boost::signal<Signature, Combiner, Group, GroupCompare, SlotFunction> >
{
    typedef boost::signals::connection connection_type;
};
*/


template<
    typename Signature,
    typename Combiner = signal_private::last_value<typename boost::function_traits<Signature>::result_type>
    >
class SignalProxy
{
public:
    typedef Signal<Signature, Combiner> SignalType;

    SignalProxy() : signal(0) { }
    SignalProxy(SignalType& signal) : signal(&signal) { }
    SignalProxy(const SignalProxy& org) : signal(org.signal) { }

    SignalProxy& operator=(const SignalProxy& rhs) { signal = rhs.signal; }

    typedef typename signal_traits<SignalType>::connection_type connection_type;

    connection_type connect(typename SignalType::slot_function_type f){
        if(signal){
            return signal->connect(f);
        } else {
            return connection_type();
        }
    };

private:
    SignalType* signal;
};


} // namespace cnoid;

#endif
