#ifndef CNOID_BASE_PYTHON_PYQTTRAMPOLINE_H
#define CNOID_BASE_PYTHON_PYQTTRAMPOLINE_H

#include <nanobind/nanobind.h>
#include <new>
#include <utility>

namespace cnoid {
namespace python {

namespace nb = nanobind;

// Create a heap-allocated trampoline (alias) instance so that BOTH Python-side
// virtual overrides and C++ (Qt) parent ownership/deletion work for the same
// object.
//
// Background on why neither stock construction mode is sufficient on its own:
//
//  - nb::init embeds the C++ object inside the PyObject (internal storage). When
//    the object is reparented to a Qt parent and Qt later calls operator delete
//    on the child, it operates on an address inside the PyObject allocation and
//    corrupts the heap.
//
//  - nb::new_ heap-allocates a standalone object (so Qt can delete it safely),
//    but it constructs the object inside the user lambda, i.e. before nanobind
//    has registered the instance in its C++ -> Python map. A trampoline's
//    constructor looks the instance up by its own C++ pointer to install the
//    glue, so the lookup fails and virtual overrides are not possible.
//
// This routine registers an externally-owned instance BEFORE running the
// constructor: the raw memory is allocated, the nanobind wrapper is created
// around it (entering the C++ -> Python map), and only then is the trampoline
// placement-constructed, so its constructor finds the already-registered Python
// object. The wrapper owns the object (destruct + cpp_delete) just like an
// nb::new_ heap object, so it is released with operator delete -- the same
// mechanism Qt uses for a parent-owned child. Ownership can later be handed to
// Qt with the usual releaseOwnership()/OwnershipReleased<T> machinery.
//
// 'cls' is the (possibly Python-derived) type object passed to __new__, so the
// returned wrapper has exactly the requested Python type and no second wrapper
// is created for the same C++ object.
// Make a bound type non-constructible from Python. A bound C++ subclass that is
// only ever created on the C++ side (e.g. a singleton obtained through an
// "instance" accessor) still inherits its base class's __new__, which would
// construct an object of the *base* C++ type while claiming to be the subclass.
// Binding the result of this function as the subclass's __new__ shadows the
// inherited one and raises a clear error instead of silently building a
// mismatched object.
//
// Usage:  cls.def_static("__new__", python::forbidConstruction("TimeBar"));
inline auto forbidConstruction(const char* typeName)
{
    // Accept any arguments (including none) so that this body always runs and
    // raises the explanatory error, rather than nanobind rejecting the call
    // with a generic "incompatible function arguments" message.
    return [typeName](nb::args, nb::kwargs) -> nb::object {
        nb::raise_type_error("%s cannot be instantiated from Python", typeName);
        return nb::object(); // unreachable; raise_type_error always throws
    };
}

template<typename Alias, typename... Args>
nb::object createQtTrampoline(nb::handle cls, Args&&... args)
{
    void* mem = ::operator new(sizeof(Alias));
    nb::object self;
    try {
        // Registers 'mem' in the C++ -> Python instance map and marks the
        // wrapper ready/owning. Disarm destruction until the object is actually
        // constructed so that an exception below cannot destruct raw memory.
        self = nb::inst_take_ownership(cls, mem);
        nb::inst_set_state(self, /* ready = */ false, /* destruct = */ false);
        new (mem) Alias(std::forward<Args>(args)...);
        nb::inst_set_state(self, /* ready = */ true, /* destruct = */ true);
    } catch(...) {
        if(self.is_valid()){
            // The object was not constructed: keep destruction disarmed so the
            // wrapper just unregisters itself, then free the raw memory here.
            nb::inst_set_state(self, false, false);
            self = nb::object();
        }
        ::operator delete(mem);
        throw;
    }
    return self;
}

}
}

#endif
