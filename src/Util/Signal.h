/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SIGNAL_H
#define CNOID_UTIL_SIGNAL_H

#include "Referenced.h"
#include <functional>
#include <tuple>

namespace cnoid {

class Connection;

template<
    typename TSignature,
    typename Combiner = signal_private::last_value<typename std::function_traits<TSignature>::result_type>
    >
class Signal;


namespace signal_private {

template<class F, class... Ts, class... Us>
typename std::enable_if<
    sizeof...(Us) == sizeof...(Ts),
    typename std::function<F>::result_type>::type
apply_impl(F& func, std::tuple<Ts...>& args, Us*... us) 
{
    return func(*us...);
}

template<class F, class... Ts, class... Us>
typename std::enable_if<
    sizeof...(Us) < sizeof...(Ts),
    typename std::function<F>::result_type>::type
apply_impl(F& func, std::tuple<Ts...>& args, Us*... us) 
{
    return apply_impl(func, args, us..., &std::get<sizeof...(Us)>(args));
}

template<class F, class... Ts>
typename std::function<F>::result_type
apply(F& fun, std::tuple<Ts...>& args) 
{
    return apply_impl(fun, args);
}

template<typename T>
struct last_value
{
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


template<typename SlotType, typename... Args>
class SlotCallIterator
{
    typedef typename SlotType::result_type result_type;

    SlotType* currentSlot;
    std::tuple<Args...>& args;

public:
    void seekUnblockedSlot(){
        while(currentSlot && currentSlot->isBlocked){
            currentSlot = currentSlot->next;
        }
    }
    
    SlotCallIterator(SlotType* firstSlot, std::tuple<Args...>& args)
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
    
    result_type operator*() const {
        return apply(currentSlot, args);
    }
};


template<typename Signature, typename Combiner>
class SlotHolder : public SlotHolderBase
{
    typedef std::function<TSignature> FuncType;
    FuncType func;
    
    typedef ref_ptr<SlotHolder> SlotHolderPtr;
    SlotHolderPtr next;
    SlotHolder* prev;

    typedef Signal<Signature, Combiner> SignalType;
    SignalType* owner;

public:
    typedef R result_type;
    
    SlotHolder(const FuncType& func)
        : func(func), prev(0), owner(0) {
    }

    virtual void disconnect() {
        if(owner) owner->remove(this);
    }

    virtual bool connected() const {
        return owner != 0;
    }

    virtual void changeOrder(int orderId) {
        if(owner) owner->changeOrder(this, orderId);
    }
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


template<
    typename TSignature,
    typename Combiner = signal_private::last_value<typename std::function_traits<TSignature>::result_type>
    >
class Signal
{
public:
    typedef TSignature Signature;
    typedef std::function_traits<Signature> traits;
    typedef traits::result_type result_type;
    typedef std::function<Signature> slot_function_type;
    typedef slot_function_type slot_type;
    typedef slot_function_type Slot;

    typedef Combiner combiner_type;       
    typedef struct { } group_type;          
    typedef struct { } group_compare_type;  

private:
    typedef signal_private::SlotHolder<Signature, Combiner> SlotHolderType;
    typedef ref_ptr<SlotHolderType> SlotHolderPtr;

    SlotHolderPtr firstSlot;
    SlotHolderType* lastSlot;

    Signal(const Signal& org);
    Signal& operator=(const Signal& rhs);

public:
    Signal() : lastSlot(0) { }

    ~Signal() {
        disconnect_all_slots();
    }

    Connection connect(const slot_function_type& func){

        SlotHolderType* slot = new SlotHolderType(func);

        if(!firstSlot){
            firstSlot = slot;
            lastSlot = slot;
        } else {
            lastSlot->next = slot;
            slot->prev = lastSlot;
            lastSlot = slot;
        }
        slot->owner = this;

        return Connection(slot);
    }

    void remove(SlotHolderPtr slot){
        if(slot->owner == this){
            SlotHolderType* next = slot->next;
            SlotHolderType* prev = slot->prev;
            if(next){
                next->prev = prev;
            } else {
                lastSlot = prev;
            }
            if(prev){
                prev->next = next;
            } else {
                firstSlot = next;
            }
            slot->prev = 0;
            slot->next = 0;
            slot->owner = 0;
        }
    }

    void changeOrder(SlotHolderPtr slot, int orderId){
        if(slot->owner == this){
            if(orderId == Connection::FIRST){
                if(firstSlot != slot){
                    remove(slot);
                    slot->owner = this;
                    if(firstSlot){
                        slot->next = firstSlot;
                        slot->next->prev = slot;
                    }
                    firstSlot = slot;
                }
            } else if(orderId == Connection::LAST){
                if(lastSlot != slot){
                    remove(slot);
                    slot->owner = this;
                    if(lastSlot){
                        lastSlot->next = slot;
                        slot->prev = lastSlot;
                    } else {
                        firstSlot = slot;
                    }
                    lastSlot = slot;
                }
            }
        }
    }
                        
    void disconnect_all_slots() {
        while(firstSlot){
            remove(firstSlot);
        }
    }

    bool empty() const {
        return (firstSlot == 0);
    }
    
    template<typename... Args>
    typename result_type operator()(Args... args){
        typedef signal_private::SlotCallIterator<SlotType, Args...> IteratorType;
        Combiner combiner;
        std::tuple<Args...> argset(args...);
        return combiner(IteratorType(firstSlot, argset), IteratorType(0, argset));
    }
};


class LogicalProduct
{
public:
    typedef bool result_type;
    template<typename InputIterator>
    bool operator()(InputIterator first, InputIterator last) const {
        bool result = true;
        while(first != last){
            result &= *first++;
        }
        return result;
    }
};


class LogicalSum
{
public:
    typedef bool result_type;
    template<typename InputIterator>
    bool operator()(InputIterator first, InputIterator last) const {
        bool result = false;
        while(first != last){
            result |= *first++;
        }
        return result;
    }
};


template<
    typename Signature,
    typename Combiner = signal_private::last_value<typename std::function_traits<Signature>::result_type>
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
