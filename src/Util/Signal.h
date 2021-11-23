/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SIGNAL_H
#define CNOID_UTIL_SIGNAL_H

#include "Referenced.h"
#include <functional>
#include <vector>
#include <tuple>
#include <type_traits>

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
    T operator()(InputIterator iter, InputIterator last) const {
        T value;
        while (iter != last){
            if(iter.isReady()){
                value = *iter;
            }
            ++iter;
        }
        return value;
    }
};

template<>
struct last_value<void> {
  public:
    template<typename InputIterator>
    void operator()(InputIterator iter, InputIterator last) const{
        while (iter != last){
            if(iter.isReady()) *iter;
            ++iter;
        }
    }
};

class SignalBase;

class SlotHolderBase : public Referenced
{
public:
    SlotHolderBase() : prev(nullptr), owner(nullptr), blockCounter(0) { }
    void disconnect();
    bool connected() const;
    void changeOrder(int orderId);

    ref_ptr<SlotHolderBase> next;
    SlotHolderBase* prev;
    SignalBase* owner;
    int blockCounter;
};

class SignalBase
{
protected:
    typedef ref_ptr<SlotHolderBase> SlotHolderPtr;
    SlotHolderPtr firstSlot;
    SlotHolderBase* lastSlot;
    std::vector<SlotHolderPtr>* pSlotsToConnectLater;
    bool isCallingSlots;

    friend class SlotHolderBase;
    
    SignalBase() : lastSlot(nullptr), pSlotsToConnectLater(nullptr), isCallingSlots(false) { }

    ~SignalBase() {
        disconnectAllSlots();
        if(pSlotsToConnectLater){
            delete pSlotsToConnectLater;
        }
    }

    void connectSlotHolder(SlotHolderBase* slot) {

        if(!isCallingSlots){
            if(!firstSlot){
                firstSlot = slot;
                lastSlot = slot;
            } else {
                lastSlot->next = slot;
                slot->prev = lastSlot;
                lastSlot = slot;
            }
            slot->owner = this;
        } else {
            if(!pSlotsToConnectLater){
                pSlotsToConnectLater = new std::vector<SlotHolderPtr>();
            }
            pSlotsToConnectLater->push_back(slot);
        }
    }

    void connectSlotsWithPendingConnection(){
        for(auto& slot : *pSlotsToConnectLater){
            if(!firstSlot){
                firstSlot = slot;
                lastSlot = slot;
            } else {
                lastSlot->next = slot;
                slot->prev = lastSlot;
                lastSlot = slot;
            }
            slot->owner = this;
        }
        pSlotsToConnectLater->clear();
    }

    void remove(SlotHolderPtr slot){
        if(slot->owner == this){
            SlotHolderBase* next = slot->next;
            SlotHolderBase* prev = slot->prev;
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
            slot->prev = nullptr;
            slot->owner = nullptr;
            ++(slot->blockCounter);

            /**
               keep slot->next so that the slot call iteration
               can be continued even if the slot is disconnected
               during the iteration.
            */
        }
    }

    void changeOrder(SlotHolderPtr slot, int orderId);

public:
    bool hasConnections() const {
        return (firstSlot != nullptr);
    }        

    int numConnections() const {
        int n = 0;
        auto slot = firstSlot;
        while(slot){
            ++n;
            slot = slot->next;
        }
        return n;
    }

    void disconnectAllSlots() {
        while(firstSlot){
            remove(firstSlot);
        }
    }

    [[deprecated("Use !hasConnections()")]]
    bool empty() const {
        return (firstSlot == nullptr);
    }
};


inline void SlotHolderBase::disconnect()
{
    if(owner){
        owner->remove(this);
    }
}

inline bool SlotHolderBase::connected() const
{
    return owner != nullptr;
}

inline void SlotHolderBase::changeOrder(int orderId)
{
    if(owner){
        owner->changeOrder(this, orderId);
    }
}


template<typename SlotHolderType, typename... Args>
class SlotCallIterator
{
    typedef typename SlotHolderType::result_type result_type;

    /**
       A smart pointer must be used to store the current slot holder object to
       guarantee the existence of the slot function and the next slot during
       the slot is being called.
       There is a case where the slot is disconnected when it is being called.
       Or when the corresponding cnoid::Connection object is holded in a Lua
       script, the connection may be deleted by the garbage collection in the
       Lua interperter when a slot function defined in the Lua script is called.
       In this case, the function does not exist when it is actually called
       unless the reference to currentSlotHolder is kept here.
    */
    ref_ptr<SlotHolderType> currentSlotHolder;
    
    std::tuple<Args...>& args;

public:
    void seekActiveSlot(){
        while(currentSlotHolder && (currentSlotHolder->blockCounter > 0)){
            currentSlotHolder = currentSlotHolder->next;
        }
    }
    
    SlotCallIterator(SlotHolderBase* firstSlot, std::tuple<Args...>& args)
        : currentSlotHolder(static_cast<SlotHolderType*>(firstSlot)),
          args(args) {
    }

    SlotCallIterator(const SlotCallIterator& org)
        : currentSlotHolder(org.currentSlotHolder), args(org.args) { }

    bool operator==(const SlotCallIterator& rhs) const {
        return (currentSlotHolder == rhs.currentSlotHolder);
    }

    bool operator!=(const SlotCallIterator& rhs) const {
        return (currentSlotHolder != rhs.currentSlotHolder);
    }

    SlotCallIterator& operator++() {
        currentSlotHolder = static_pointer_cast<SlotHolderType>(currentSlotHolder->next);
        return *this;
    }

    bool isReady() const {
        return currentSlotHolder->blockCounter == 0;
    }
    
    result_type operator*() const {
        return apply(currentSlotHolder->func, args);
    }
};


template<typename Signature, typename Combiner>
class SlotHolder : public SlotHolderBase
{
public:
    typedef std::function<Signature> FuncType;
    typedef typename signal_private::function_traits<Signature>::result_type result_type;
    FuncType func;

    SlotHolder(const FuncType& func) : func(func) { }
};
    
} // namespace signal_private


class Connection
{
    ref_ptr<signal_private::SlotHolderBase> slot;

public:
    Connection() { }
    Connection(signal_private::SlotHolderBase* slot) : slot(slot) { }
    Connection(const Connection& org) : slot(org.slot) { }
    Connection(Connection&&) = default;

    Connection& operator=(const Connection& rhs) {
        slot = rhs.slot;
        return *this;
    }

    void disconnect() {
        if(slot) {
            slot->disconnect();
            slot.reset();
        }
    }

    bool connected() const {
        return slot && slot->connected();
    }

    void block() {
        if(slot){
            ++(slot->blockCounter);
        }
    }

    void unblock() {
        if(slot && slot->blockCounter > 0){
            --(slot->blockCounter);
        }
    }

    bool isBlocked() const {
        return slot && (slot->blockCounter > 0);
    }

    enum Order { FIRST = 0, LAST };
    
    Connection& changeOrder(Order order) {
        if(slot){
            slot->changeOrder(order);
        }
        return *this;
    }

    class ScopedBlock {
        Connection* pConnection;
    public:
        ScopedBlock(Connection& connection)
            : pConnection(&connection) {
            connection.block();
        }
        ScopedBlock(ScopedBlock&& org) : pConnection(org.pConnection){
            org.pConnection = nullptr;
        }
        ScopedBlock(const ScopedBlock&) = delete;
        ScopedBlock& operator=(const ScopedBlock&) = delete;
        ~ScopedBlock(){
            if(pConnection){
                pConnection->unblock();
            }
        }
    };
    ScopedBlock scopedBlock(){ return ScopedBlock(*this); }
};


class ScopedConnection
{
    Connection connection_;
    
public:
    ScopedConnection() { }
    ScopedConnection(ScopedConnection&&) = default;
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
    bool isBlocked() const { return connection_.isBlocked(); }
    ScopedConnection& changeOrder(Connection::Order order) { connection_.changeOrder(order); return *this; }
    Connection& connection(){ return connection_; }
    const Connection& connection() const { return connection_; }
    Connection::ScopedBlock scopedBlock(){ return connection_.scopedBlock(); }
};


namespace signal_private {

inline void SignalBase::changeOrder(SlotHolderPtr slot, int orderId)
{
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

}

template<
    typename TSignature,
    typename Combiner = signal_private::last_value<
        typename signal_private::function_traits<TSignature>::result_type>
    >
class Signal;

template<typename Combiner, typename R, typename... Args>
class Signal<R(Args...), Combiner> : public signal_private::SignalBase
{
public:
    typedef typename signal_private::function_traits<R(Args...)>::result_type result_type;
    typedef std::function<R(Args...)> function_type;
    typedef function_type Function;

private:
    typedef signal_private::SlotHolder<R(Args...), Combiner> SlotHolderType;

public:
    Signal() { }
    Signal(Signal&&) = default;
    Signal(const Signal& org) = delete;
    Signal& operator=(const Signal& rhs) = delete;

    Connection connect(const Function& func){
        auto slot = new SlotHolderType(func);
        connectSlotHolder(slot);
        return Connection(slot);
    }

    result_type operator()(Args... args){
        return invoke(std::is_void<result_type>{}, std::forward<Args>(args)...);
    }

private:
    void invoke(std::true_type, Args&&... args){
        if(firstSlot){
            typedef signal_private::SlotCallIterator<SlotHolderType, Args...> IteratorType;
            Combiner combiner;
            std::tuple<Args...> argset(args...);
            isCallingSlots = true;
            combiner(IteratorType(firstSlot, argset), IteratorType(nullptr, argset));
            isCallingSlots = false;
            if(pSlotsToConnectLater){
                connectSlotsWithPendingConnection();
            }
        }
    }

    /**
       The combiner operation must always be executed even if there is no slot connected
       to the signal to return the correct default value that is determined by the combiner.
    */
    result_type invoke(std::false_type, Args&&... args){
        typedef signal_private::SlotCallIterator<SlotHolderType, Args...> IteratorType;
        Combiner combiner;
        std::tuple<Args...> argset(args...);
        isCallingSlots = true;
        auto result = combiner(IteratorType(firstSlot, argset), IteratorType(nullptr, argset));
        isCallingSlots = false;
        if(pSlotsToConnectLater){
            connectSlotsWithPendingConnection();
        }
        return result;
    }
};


class LogicalProduct
{
public:
    typedef bool result_type;
    template<typename InputIterator>
    bool operator()(InputIterator iter, InputIterator last) const {
        bool result = true;
        while(iter != last){
            if(iter.isReady()){
                result &= *iter;
            }
            ++iter;
        }
        return result;
    }
};


class LogicalSum
{
public:
    typedef bool result_type;
    template<typename InputIterator>
    bool operator()(InputIterator iter, InputIterator last) const {
        bool result = false;
        while(iter != last){
            if(iter.isReady()){
                result |= *iter;
            }
            ++iter;
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

    SignalProxy() : signal(nullptr) { }
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

    bool hasConnections() const { return signal->hasConnections(); }

private:
    SignalType* signal;
};

} // namespace cnoid;

#endif
