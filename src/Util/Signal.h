/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SIGNAL_H
#define CNOID_UTIL_SIGNAL_H

#include "Referenced.h"
#include <functional>
#include <tuple>

namespace cnoid {

namespace signal_private {

template<typename T>
class function_traits
{
    static_assert(sizeof( T ) == 0, "function_traits<T>: T is not a function type");
};

template<typename R, typename... Ts>
struct function_traits<R(Ts...)>
{
    constexpr static const std::size_t arity = sizeof...(Ts);
    using result_type = R;
};

template<typename R, typename... Ts>
struct function_traits<R(Ts...) const> : function_traits<R(Ts...)> {};


template<class F, class... Ts, class... Us>
typename std::enable_if<
    sizeof...(Us) == sizeof...(Ts),
    typename F::result_type>::type
apply_impl(F& func, std::tuple<Ts...>&, Us*... us) 
{
    return func(*us...);
}

template<class F, class... Ts, class... Us>
typename std::enable_if<
    sizeof...(Us) < sizeof...(Ts),
    typename F::result_type>::type
apply_impl(F& func, std::tuple<Ts...>& args, Us*... us) 
{
    return apply_impl(func, args, us..., &std::get<sizeof...(Us)>(args));
}

template<class F, class... Args>
typename F::result_type
apply(F& fun, std::tuple<Args...>& args) 
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


template<typename SlotHolderType, typename... Args>
class SlotCallIterator
{
    typedef typename SlotHolderType::result_type result_type;

    SlotHolderType* currentSlotHolder;
    std::tuple<Args...>& args;

public:
    void seekActiveSlot(){
        while(currentSlotHolder && currentSlotHolder->isBlocked){
            currentSlotHolder = currentSlotHolder->next;
        }
    }
    
    SlotCallIterator(SlotHolderType* firstSlot, std::tuple<Args...>& args)
        : currentSlotHolder(firstSlot), args(args) {
        seekActiveSlot();
    }

    SlotCallIterator(const SlotCallIterator& org)
        : currentSlotHolder(org.currentSlotHolder), args(org.args) {
        seekActiveSlot();
    }

    bool operator==(const SlotCallIterator& rhs) const {
        return (currentSlotHolder == rhs.currentSlotHolder);
    }

    bool operator!=(const SlotCallIterator& rhs) const {
        return (currentSlotHolder != rhs.currentSlotHolder);
    }

    SlotCallIterator operator++(int) {
        SlotCallIterator iter(*this);
        currentSlotHolder = currentSlotHolder->next;
        seekActiveSlot();
        return iter;
    }
    
    result_type operator*() const {
        /**
           The reference to currentSlotHolder should be kept when the slot
           function is called to guarantee the existence of the slot function.
           For example, if the corresponding cnoid::Connection object is holded
           in a Lua script, the connection may be deleted by the garbage collection
           in the Lua interperter when a slot function defined in the Lua script
           is called. In this case, the function does not exist when it is actually
           called unless the reference to currentSlotHolder is kept here.
        */
        ref_ptr<SlotHolderBase> holder = currentSlotHolder;
        
        return apply(currentSlotHolder->func, args);
    }
};

} // namespace signal_private


template<
    typename TSignature,
    typename Combiner = signal_private::last_value<
        typename signal_private::function_traits<TSignature>::result_type>
    >
class Signal;


namespace signal_private {

template<typename Signature, typename Combiner>
class SlotHolder : public SlotHolderBase
{
public:
    typedef std::function<Signature> FuncType;
    FuncType func;
    
    typedef ref_ptr<SlotHolder> SlotHolderPtr;
    SlotHolderPtr next;
    SlotHolder* prev;

    typedef Signal<Signature, Combiner> SignalType;
    SignalType* owner;

    typedef typename signal_private::function_traits<Signature>::result_type result_type;
    
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

    bool connected() const {
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


class ScopedConnection
{
    Connection connection_;
    
public:
    ScopedConnection() { }
    ScopedConnection(const ScopedConnection& org) = delete;
    ScopedConnection(const Connection& org) { connection_ = org; }
    ~ScopedConnection() { connection_.disconnect(); }
    void reset() { connection_.disconnect(); }
    void reset(const Connection& c) { connection_.disconnect(); connection_ = c; }
    ScopedConnection& operator=(const ScopedConnection& rhs) = delete;
    ScopedConnection& operator=(const Connection& rhs) { reset(rhs); return *this; }
    void disconnect() { connection_.disconnect(); }
    bool connected() const { return connection_.connected(); }
    void block() { connection_.block(); }
    void unblock() { connection_.unblock(); }
    ScopedConnection& changeOrder(Connection::Order order) { connection_.changeOrder(order); return *this; }
    Connection& connection(){ return connection_; }
    const Connection& connection() const { return connection_; }
};


template<typename Combiner, typename R, typename... Args>
class Signal<R(Args...), Combiner>
{
public:
    typedef typename signal_private::function_traits<R(Args...)>::result_type result_type;
    typedef std::function<R(Args...)> function_type;
    typedef function_type Function;

private:
    typedef signal_private::SlotHolder<R(Args...), Combiner> SlotHolderType;
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

    Connection connect(const Function& func){

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
            slot->owner = 0;
            slot->isBlocked = true;

            /**
               keep slot->next so that the slot call iteration
               can be continued even if the slot is disconnected
               during the iteration.
            */
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
    
    result_type operator()(Args... args){
        typedef signal_private::SlotCallIterator<SlotHolderType, Args...> IteratorType;
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
    typename Combiner = signal_private::last_value<
        typename signal_private::function_traits<Signature>::result_type>
    >
class SignalProxy
{
public:
    typedef Signal<Signature, Combiner> SignalType;

    SignalProxy() : signal(0) { }
    SignalProxy(SignalType& signal) : signal(&signal) { }
    SignalProxy(const SignalProxy& org) : signal(org.signal) { }

    SignalProxy& operator=(const SignalProxy& rhs) { signal = rhs.signal; }

    Connection connect(typename SignalType::Function f){
        if(signal){
            return signal->connect(f);
        } else {
            return Connection();
        }
    };

private:
    SignalType* signal;
};

} // namespace cnoid;

#endif
