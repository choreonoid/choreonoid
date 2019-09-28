/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_REFERENCED_H
#define CNOID_UTIL_REFERENCED_H

#include <atomic>
#include <cassert>
#include <iosfwd>
#include <functional>

#ifdef _WIN32
#include <memory>
#endif

#include "exportdecl.h"

namespace cnoid {

class Referenced;

/**
   \todo Make this thread safe
*/
class WeakCounter
{
public:
    WeakCounter(){
        isObjectAlive_ = true;
        weakCount = 0;
    }

    void add() { ++weakCount; }

    void release() {
        if(--weakCount == 0){
            if(!isObjectAlive_){
                delete this;
            }
        }
    }

    void setDestructed() {
        isObjectAlive_ = false;
        if(weakCount == 0){
            delete this;
        }
    }

    bool isObjectAlive(){
        return isObjectAlive_;
    }

private:
    bool isObjectAlive_;
    int weakCount;
};

    
class CNOID_EXPORT Referenced
{
    friend class WeakCounter;
    template<class Y> friend class weak_ref_ptr;
    template<class Y> friend class ref_ptr;

    mutable std::atomic<int> refCount_;
    WeakCounter* weakCounter_;
            
    void addRef() const {
        refCount_.fetch_add(1, std::memory_order_relaxed);
    }

    void releaseRef() const {
        if(refCount_.fetch_sub(1, std::memory_order_release) == 1) {
            std::atomic_thread_fence(std::memory_order_acquire);
            delete this;
        }
    }

    void decrementRef() const {
        refCount_.fetch_sub(1, std::memory_order_release);
    }
    
    WeakCounter* weakCounter(){
        if(!weakCounter_){
            weakCounter_ = new WeakCounter();
        }
        return weakCounter_;
    }

protected:
    Referenced() : refCount_(0), weakCounter_(0) { }
    Referenced(const Referenced&) : refCount_(0), weakCounter_(0) { }

    int refCount() const { return refCount_.load(std::memory_order_relaxed); }
    
public:
    virtual ~Referenced();
};

    
template<class T> class ref_ptr
{
public:
    typedef T element_type;
    
    ref_ptr() : px(0) { }

    ref_ptr(T* p) : px(p){
        if(px != nullptr){
            px->addRef();
        }
    }

    template<class U>
    ref_ptr(ref_ptr<U> const & rhs) : px(rhs.get()){
        if(px != nullptr){
            px->addRef();
        }
    }

    ref_ptr(ref_ptr const & rhs) : px(rhs.px){
        if(px != nullptr){
            px->addRef();
        }
    }

    ~ref_ptr(){
        if(px != nullptr){
            px->releaseRef();
        }
    }

    template<class U> ref_ptr& operator=(ref_ptr<U> const & rhs){
        ref_ptr(rhs).swap(*this);
        return *this;
    }

    ref_ptr(ref_ptr&& rhs) noexcept : px(rhs.px){
        rhs.px = nullptr;
    }

    ref_ptr& operator=(ref_ptr&& rhs) {
        ref_ptr(static_cast<ref_ptr &&>(rhs)).swap(*this);
        return *this;
    }

    ref_ptr& operator=(ref_ptr const & rhs){
        ref_ptr(rhs).swap(*this);
        return *this;
    }

    ref_ptr& operator=(T* rhs){
        ref_ptr(rhs).swap(*this);
        return *this;
    }

    void reset(){
        ref_ptr().swap(*this);
    }

    void reset(T* rhs){
        ref_ptr(rhs).swap(*this);
    }

    // explicit conversion to the raw pointer
    T* get() const {
        return px;
    }

    T* retn(){
        T* p = px;
        if(px != nullptr){
            px->decrementRef();
            px = nullptr;
        }
        return p;
    }

    // implict conversion to the raw pointer
    operator T*() const {
        return px;
    }        

    T& operator*() const {
        assert(px != nullptr);
        return *px;
    }

    T* operator->() const {
        assert(px != nullptr);
        return px;
    }

    void swap(ref_ptr& rhs){
        T* tmp = px;
        px = rhs.px;
        rhs.px = tmp;
    }

private:
    T* px;

    template<class Y> friend class ref_ptr;
    template<class Y> friend class weak_ref_ptr;
    friend struct std::hash<ref_ptr<T>>;
};


template<class T, class U> inline bool operator==(ref_ptr<T> const & a, ref_ptr<U> const & b)
{
    return a.get() == b.get();
}

template<class T, class U> inline bool operator!=(ref_ptr<T> const & a, ref_ptr<U> const & b)
{
    return a.get() != b.get();
}

template<class T, class U> inline bool operator==(ref_ptr<T> const & a, U * b)
{
    return a.get() == b;
}

template<class T, class U> inline bool operator!=(ref_ptr<T> const & a, U * b)
{
    return a.get() != b;
}

template<class T, class U> inline bool operator==(T * a, ref_ptr<U> const & b)
{
    return a == b.get();
}

template<class T, class U> inline bool operator!=(T * a, ref_ptr<U> const & b)
{
    return a != b.get();
}

template<class T> inline bool operator<(ref_ptr<T> const & a, ref_ptr<T> const & b)
{
    return a.get() < b.get();
}

template<class T> void swap(ref_ptr<T> & lhs, ref_ptr<T> & rhs)
{
    lhs.swap(rhs);
}

template<class T, class U> ref_ptr<T> static_pointer_cast(ref_ptr<U> const & p)
{
    return static_cast<T*>(p.get());
}
    
template<class T, class U> ref_ptr<T> const_pointer_cast(ref_ptr<U> const & p)
{
    return const_cast<T*>(p.get());
}
    
template<class T, class U> ref_ptr<T> dynamic_pointer_cast(ref_ptr<U> const & p)
{
    return dynamic_cast<T*>(p.get());
}

template<class Y> std::ostream & operator<< (std::ostream & os, ref_ptr<Y> const & p)
{
    os << p.get();
    return os;
}
    
    
typedef ref_ptr<Referenced> ReferencedPtr;


template<class T> class weak_ref_ptr
{
    typedef void (weak_ref_ptr<T>::*bool_type)() const;

    void bool_type_func() const { }
    
    void setCounter(){
        if(px){
            counter = px->weakCounter();
            counter->add();
        } else {
            counter = 0;
        }
    }
            
public:
    typedef T element_type;
    
    weak_ref_ptr() : px(0), counter(0) { }

    template<class Y>
    weak_ref_ptr(weak_ref_ptr<Y> const & rhs) : px(rhs.lock().get()){
        setCounter();
    }

    weak_ref_ptr(weak_ref_ptr const & rhs) : px(rhs.lock().get()){
        setCounter();
    }
    
    template<class Y>
    weak_ref_ptr& operator=(weak_ref_ptr<Y> const & rhs){
        px = rhs.lock().get();
        setCounter();
        return *this;
    }

    weak_ref_ptr(weak_ref_ptr&& rhs) : px(rhs.px), counter(rhs.counter){
        rhs.px = nullptr;
        rhs.counter = 0;
    }

    weak_ref_ptr& operator=(weak_ref_ptr&& rhs){
        weak_ref_ptr(static_cast<weak_ref_ptr&&>(rhs)).swap(*this);
        rhs.px = nullptr;
        rhs.counter = 0;
        return rhs;
    }

    template<class Y>
    weak_ref_ptr(ref_ptr<Y> const & rhs) : px(rhs.px){
        setCounter();
    }

    template<class Y>
    weak_ref_ptr(Y* const & rhs) : px(rhs){
        setCounter();
    }
        
    template<class Y>
    weak_ref_ptr& operator=(ref_ptr<Y> const & rhs){
        px = rhs.px;
        setCounter();
        return *this;
    }

    operator bool_type() const { return px ? &weak_ref_ptr<T>::bool_type_func : 0; }

    ref_ptr<T> lock() const {
        if(counter && counter->isObjectAlive()){
            return ref_ptr<T>(px);
        } else {
            return ref_ptr<T>();
        }
    }

    bool expired() const {
        return !counter || !counter->isObjectAlive();
    }

    void reset(){
        weak_ref_ptr().swap(*this);
    }

    void swap(weak_ref_ptr& other){
        T* px_ = px;
        px = other.px;
        other.px = px_;
        WeakCounter* counter_ = counter;
        counter = other.counter;
        other.counter = counter_;
    }

    template<class Y> bool _internal_less(weak_ref_ptr<Y> const & rhs) const {
        return counter < rhs.counter;
    }
        
private:
    T* px;
    WeakCounter* counter;

    template<class Y> friend class weak_ref_ptr;
    template<class Y> friend class ref_ptr;
    friend struct std::hash<weak_ref_ptr<T>>;
};

template<class T, class U> inline bool operator<(weak_ref_ptr<T> const & a, weak_ref_ptr<U> const & b)
{
    return a._internal_less(b);
}

template<class T> void swap(weak_ref_ptr<T> & a, weak_ref_ptr<T> & b)
{
    a.swap(b);
}

}

namespace std {

template<class T>
struct hash<cnoid::ref_ptr<T>>
{
public:
    size_t operator()(const cnoid::ref_ptr<T>& p) const
    {
        return hash<T*>()(p.px);
    }
};

template<class T>
struct hash<cnoid::weak_ref_ptr<T>>
{
public:
    size_t operator()(const cnoid::weak_ref_ptr<T>& p) const
    {
        return hash<cnoid::WeakCounter*>()(p.counter);
    }
};

}

#endif
