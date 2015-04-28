/**
   @file Implementation of Signal template classes
   @note The classes are written by using the boost.signals library as a reference.
   @author Shin'ichiro Nakaoka
*/

#define CNOID_SIGNAL_FUNCTION_N_HEADER CNOID_SIGNAL_CONCAT(<boost/function/function,CNOID_SIGNAL_NUM_ARGS.hpp>)
#include CNOID_SIGNAL_FUNCTION_N_HEADER

#if CNOID_SIGNAL_NUM_ARGS == 0
#  define CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS
#else
#  define CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS ,
#endif

#define CNOID_SIGNAL_SIGNAL CNOID_SIGNAL_CONCAT(Signal,CNOID_SIGNAL_NUM_ARGS)
#define CNOID_SIGNAL_FUNCTION CNOID_SIGNAL_CONCAT(boost::function,CNOID_SIGNAL_NUM_ARGS)
#define CNOID_SIGNAL_SLOT_HOLDER CNOID_SIGNAL_CONCAT(SlotHolder,CNOID_SIGNAL_NUM_ARGS)
#define CNOID_SIGNAL_ARGSET CNOID_SIGNAL_CONCAT(ArgSet,CNOID_SIGNAL_NUM_ARGS)


namespace cnoid {

template<
    typename R,
    CNOID_SIGNAL_TEMPLATE_PARMS
    CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS
    typename Combiner = signal_private::last_value<R>
    >
class CNOID_SIGNAL_SIGNAL;


namespace signal_private {

template<
    typename R,
    CNOID_SIGNAL_TEMPLATE_PARMS
    CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS
    typename Combiner
    >
struct CNOID_SIGNAL_SLOT_HOLDER : public SlotHolderBase
{
    typedef CNOID_SIGNAL_FUNCTION<R CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS CNOID_SIGNAL_TEMPLATE_ARGS> FuncType;
    FuncType func;

    typedef CNOID_SIGNAL_SLOT_HOLDER SlotHolder;
    typedef ref_ptr<SlotHolder> SlotHolderPtr;
    SlotHolderPtr next;
    SlotHolder* prev;
    typedef CNOID_SIGNAL_SIGNAL<R, CNOID_SIGNAL_TEMPLATE_ARGS CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS Combiner> SignalType;
    SignalType* owner;
    
public:
    typedef R result_type;
    
    CNOID_SIGNAL_SLOT_HOLDER(const FuncType& func)
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


template<
    typename R,
    CNOID_SIGNAL_TEMPLATE_PARMS
    CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS
    typename Combiner
    >
struct CNOID_SIGNAL_ARGSET
{
    CNOID_SIGNAL_ARGS_AS_MEMBERS
    
    CNOID_SIGNAL_ARGSET(CNOID_SIGNAL_COPY_PARMS)
        CNOID_SIGNAL_INIT_ARGS { }

    template<typename SlotHolderType>
    R call(SlotHolderType* slot) {
        return slot->func(CNOID_SIGNAL_ARGS);
    }
};


template<
    typename R,
    CNOID_SIGNAL_TEMPLATE_PARMS
    CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS
    typename Combiner
    >
class CNOID_SIGNAL_SIGNAL
{
public:
    typedef R result_type;
    typedef CNOID_SIGNAL_FUNCTION<R CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS CNOID_SIGNAL_TEMPLATE_ARGS> slot_function_type;
    typedef slot_function_type slot_type;
    typedef slot_function_type Slot;

    typedef Combiner combiner_type;       
    typedef struct { } group_type;          
    typedef struct { } group_compare_type;  

private:
    typedef signal_private::CNOID_SIGNAL_SLOT_HOLDER<R, CNOID_SIGNAL_TEMPLATE_ARGS CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS Combiner> SlotHolderType;
    typedef ref_ptr<SlotHolderType> SlotHolderPtr;
    typedef CNOID_SIGNAL_ARGSET<R, CNOID_SIGNAL_TEMPLATE_ARGS CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS Combiner> ArgSetType;
    typedef signal_private::SlotCallIterator<SlotHolderType, ArgSetType> IteratorType;

    SlotHolderPtr firstSlot;
    SlotHolderType* lastSlot;

    CNOID_SIGNAL_SIGNAL(const CNOID_SIGNAL_SIGNAL& org);
    CNOID_SIGNAL_SIGNAL& operator=(const CNOID_SIGNAL_SIGNAL& rhs);

public:
    CNOID_SIGNAL_SIGNAL() : lastSlot(0) { }

    ~CNOID_SIGNAL_SIGNAL() {
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

    result_type operator()(CNOID_SIGNAL_PARMS) {

#if CNOID_SIGNAL_NUM_ARGS == 0
        ArgSetType args;
#else
        ArgSetType args(CNOID_SIGNAL_ARGS);
#endif
        
        Combiner combiner;
        return combiner(IteratorType(firstSlot, args), IteratorType(0, args));
    }
};

} // namespace cnoid


#undef CNOID_SIGNAL_FUNCTION_N_HEADER
#undef CNOID_SIGNAL_COMMA_IF_NONZERO_ARGS
#undef CNOID_SIGNAL_SIGNAL
#undef CNOID_SIGNAL_FUNCTION
#undef CNOID_SIGNAL_SLOT_HOLDER
#undef CNOID_SIGNAL_ARGSET
