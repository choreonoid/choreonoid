#ifndef CNOID_UTIL_REFERENCED_H
#define CNOID_UTIL_REFERENCED_H

#include <atomic>
#include <mutex>
#include <cstdint>
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
   This is the control block used by weak_ref_ptr. Its mutex serializes
   weak_ref_ptr::lock() against the final release of the referenced object.
*/
class WeakCounter
{
public:
    WeakCounter(){
        isObjectAlive_ = true;
        weakCount = 1; // count for the Referenced object
    }

    void add() { ++weakCount; }

    void release() {
        if(weakCount.fetch_sub(1) == 1){
            delete this;
        }
    }

    bool isObjectAlive(){
        return isObjectAlive_;
    }

private:
    std::atomic<int> weakCount;
    std::atomic<bool> isObjectAlive_;
    std::mutex mutex; // serializes weak_ref_ptr::lock() against the object's final release

    friend class Referenced;
    template<class Y> friend class weak_ref_ptr;
};


/*
   Lifetime management across a binding language (Python via nanobind, etc.)

   Referenced supports being exposed to a binding language (Python through
   nanobind, and potentially others) while keeping its lifetime correct whether
   an object originates in C++ or in the binding language. The reference count
   field packedRefCount_ operates in one of two modes (see the field comment for
   the bit layout):

     - C++ mode: the field holds the C++ reference count. addRef()/releaseRef()
       update it and the object is deleted when the count reaches 0, exactly as a
       plain intrusively reference-counted C++ object would behave. The binding
       language is never touched in this mode.

     - Binding-language mode: ownership has been handed over to the binding
       language. The field holds a pointer to the bound wrapper object (a
       PyObject* for Python, kept as a void* so this header stays free of any
       binding-language dependency). addRef()/releaseRef() now delegate to the
       wrapper's own reference count via the function pointers in
       ReferencedPythonInterface, and the C++ side never deletes the object - the
       binding language frees both the wrapper and the embedded C++ object when
       the wrapper is garbage-collected.

   setSelfPython() performs the one-way C++ -> binding-language transition: it is
   called once when the object is first exposed to the binding language, moves
   any outstanding C++ reference count into the wrapper's reference count, and
   stores the wrapper pointer. This is the same scheme as nanobind's built-in
   intrusive_counter; we register it through nb::intrusive_ptr so that the
   transition hook is inherited by every Referenced-derived type (see
   src/Util/python/PyReferenced.cpp).

   Why hand memory ownership to the binding language at all? Because an object
   constructed on the Python side is embedded inside the nanobind wrapper's
   allocation (nanobind placement-news it there); the C++ side cannot
   operator-delete such memory. Letting the binding language own and free the
   memory is the only consistent rule that works for both C++-created and
   Python-created objects. The cost is that an object still referenced from C++
   at interpreter shutdown is leaked rather than destroyed (see the is-alive
   guard below); this is accepted - leaking just before process exit is harmless,
   whereas the alternatives crash.

   Exit-time safety (the reason we do NOT use intrusive_counter as-is): when the
   interpreter has shut down, decref must not touch the dead binding-language
   runtime. A C++ static singleton (e.g. MessageOut::master()) released by its
   static destructor after interpreter finalization would otherwise call
   Py_DECREF on a dead runtime and crash. The injected decref therefore checks
   nb::is_alive() and becomes a no-op once the runtime is gone (so the object is
   neither freed nor its destructor run - it simply leaks, as noted above).
   nanobind's stock intrusive_counter lacks this guard; adding it is the one
   change we make on top of that scheme. The is-alive check lives in the injected
   decref (src/Util/python/PyReferenced.cpp) where nb::is_alive() is directly
   available; Referenced itself only decides which mode it is in.

   Note on reference cycles: as with std::shared_ptr or any intrusive count, a
   cycle of strong references among binding-language-exposed objects (e.g. two
   objects holding ref_ptr to each other) will not be collected and leaks. This
   is a property of strong reference counting, not of this scheme; it is avoided
   the usual way (break cycles with weak_ref_ptr on the C++ side, or weakref on
   the Python side). It does not occur from merely exposing an object.
*/

/*
   Function pointers that delegate reference counting of a binding-language-owned
   Referenced object to the reference count of its bound wrapper object. The
   binding layer installs these once at start-up via
   initReferencedPythonInterface(). Referenced does not depend on Python/nanobind
   headers; the wrapper is passed as a void*. The injected decref is responsible
   for the nb::is_alive() guard described above.
*/
struct ReferencedPythonInterface
{
    void (*incref)(void* wrapper) noexcept;
    void (*decref)(void* wrapper) noexcept;
};

CNOID_EXPORT void initReferencedPythonInterface(const ReferencedPythonInterface& iface);
class CNOID_EXPORT Referenced
{
    friend class WeakCounter;
    template<class Y> friend class weak_ref_ptr;
    template<class Y> friend class ref_ptr;

    /*
       packedRefCount_ packs either the C++ reference count or a pointer to the
       bound binding-language wrapper into a single word, distinguished by bit 0:

         bit 0 == 1 (C++ mode):     packedRefCount_ == (count << 1) | 1. The
                                    initial value 1 encodes a count of 0. addRef()
                                    /releaseRef() add/subtract 2 to leave bit 0 set.
         bit 0 == 0 (binding mode): packedRefCount_ is the wrapper pointer held as
                                    a uintptr_t. Wrapper (PyObject) pointers are
                                    aligned so bit 0 is always 0, disambiguating
                                    the two modes.

       In C++ mode the object is deleted when the count reaches 0 (value goes
       3 -> 1), preserving plain Referenced semantics. In binding mode reference
       counting is delegated to the wrapper and the object is destroyed by the
       binding language, so releaseRef() does not delete it here.
    */
    mutable std::atomic<std::uintptr_t> packedRefCount_;
    std::atomic<WeakCounter*> weakCounter_;

    void addRef() const {
        std::uintptr_t v = packedRefCount_.load(std::memory_order_relaxed);
        while(true){
            if(v & 1){ // C++ mode
                if(packedRefCount_.compare_exchange_weak(
                       v, v + 2, std::memory_order_relaxed)){
                    break;
                }
            } else { // binding-language mode: delegate to the wrapper
                pythonInterface.incref(reinterpret_cast<void*>(v));
                break;
            }
        }
    }

    void releaseRef() const {
        std::uintptr_t v = packedRefCount_.load(std::memory_order_acquire);
        if(!(v & 1)){
            // Binding-language mode: delegate to the wrapper's reference count.
            // The injected decref guards against a finalized runtime; the C++
            // side never deletes the object in this mode.
            pythonInterface.decref(reinterpret_cast<void*>(v));
            return;
        }
        WeakCounter* wc = weakCounter_.load(std::memory_order_acquire);
        if(!wc){
            // Fast path: no weak reference exists for this object.
            while(true){
                if(!(v & 1)){
                    // setSelfPython() switched us to binding mode concurrently.
                    pythonInterface.decref(reinterpret_cast<void*>(v));
                    return;
                }
                if(packedRefCount_.compare_exchange_weak(
                       v, v - 2, std::memory_order_acq_rel)){
                    if(v == 3){ // count went 1 -> 0
                        delete this;
                    }
                    return;
                }
            }
        } else {
            // A weak_ref_ptr may concurrently try to revive this object in
            // lock(). Serialize the final release against it via the mutex.
            std::unique_lock<std::mutex> guard(wc->mutex);
            while(true){
                if(!(v & 1)){
                    pythonInterface.decref(reinterpret_cast<void*>(v));
                    return;
                }
                if(packedRefCount_.compare_exchange_weak(
                       v, v - 2, std::memory_order_acq_rel)){
                    if(v == 3){ // count went 1 -> 0
                        wc->isObjectAlive_ = false;
                        guard.unlock();
                        delete this;
                    }
                    return;
                }
            }
        }
    }

    void decrementRef() const {
        // Used by ref_ptr::retn(): drop one C++ reference without deleting, since
        // ownership is being transferred to the caller.
        std::uintptr_t v = packedRefCount_.load(std::memory_order_relaxed);
        while(true){
            if(v & 1){ // C++ mode
                if(packedRefCount_.compare_exchange_weak(
                       v, v - 2, std::memory_order_release)){
                    break;
                }
            } else { // binding-language mode
                pythonInterface.decref(reinterpret_cast<void*>(v));
                break;
            }
        }
    }

    WeakCounter* weakCounter(){
        WeakCounter* wc = weakCounter_.load(std::memory_order_acquire);
        if(!wc){
            WeakCounter* nwc = new WeakCounter();
            if(weakCounter_.compare_exchange_strong(wc, nwc)){
                wc = nwc;
            } else {
                delete nwc; // another thread installed it first
            }
        }
        return wc;
    }

    static ReferencedPythonInterface pythonInterface;
    friend CNOID_EXPORT void initReferencedPythonInterface(const ReferencedPythonInterface& iface);

protected:
    // The initial value 1 encodes a C++ reference count of 0 (C++ mode, bit 0 set).
    Referenced() : packedRefCount_(1), weakCounter_(nullptr) { }
    Referenced(const Referenced&) : packedRefCount_(1), weakCounter_(nullptr) { }

    int refCount() const {
        std::uintptr_t v = packedRefCount_.load();
        if(v & 1){
            return static_cast<int>(v >> 1); // C++ mode
        }
        return 0; // binding-language mode: the count lives in the wrapper
    }

public:
    virtual ~Referenced();

    /**
       Hand ownership of this object over to the given binding-language wrapper.
       After this call the object is in binding-language mode: any outstanding C++
       reference count is moved into the wrapper's reference count, and subsequent
       reference counting is delegated to it. Called once by the binding layer
       when the object is first exposed (see src/Util/python/PyReferenced.cpp).
       The argument is the wrapper (a PyObject*) passed as a void*.

       May be called when the C++ count is 0 (an object created by the binding
       language's constructor, owned by no ref_ptr yet); the wrapper alone then
       keeps it alive. No-op if already in binding-language mode.
    */
    void setSelfPython(void* wrapper) noexcept {
        std::uintptr_t v = packedRefCount_.load(std::memory_order_acquire);
        if(v & 1){ // C++ mode
            std::uintptr_t count = v >> 1;
            for(std::uintptr_t i = 0; i < count; ++i){
                pythonInterface.incref(wrapper);
            }
            packedRefCount_.store(
                reinterpret_cast<std::uintptr_t>(wrapper), std::memory_order_release);
        }
    }

    /**
       Return the binding-language wrapper bound to this object (as a void*), or
       nullptr if it is still in C++ mode.
    */
    void* selfPython() const noexcept {
        std::uintptr_t v = packedRefCount_.load(std::memory_order_acquire);
        if(v & 1){
            return nullptr;
        }
        return reinterpret_cast<void*>(v);
    }
};

    
template<class T> class ref_ptr
{
public:
    typedef T element_type;
    
    ref_ptr() : px(nullptr) { }

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
    void setCounter(){
        if(px){
            counter = px->weakCounter();
            counter->add();
        } else {
            counter = nullptr;
        }
    }
            
public:
    typedef T element_type;
    
    weak_ref_ptr() : px(nullptr), counter(nullptr) { }

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

    weak_ref_ptr& operator=(weak_ref_ptr const & rhs){
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
        return *this;
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

    explicit operator bool() const { return px != nullptr; }

    ref_ptr<T> lock() const {
        if(counter){
            std::lock_guard<std::mutex> guard(counter->mutex);
            if(counter->isObjectAlive()){
                return ref_ptr<T>(px);
            }
        }
        return ref_ptr<T>();
    }

    bool expired() const {
        return counter && !counter->isObjectAlive();
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

    template<class Y> bool _internal_equal(weak_ref_ptr<Y> const & rhs) const {
        return counter == rhs.counter;
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


template<class T, class U> inline bool operator==(weak_ref_ptr<T> const & a, weak_ref_ptr<U> const & b)
{
    return a._internal_equal(b);
}

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
