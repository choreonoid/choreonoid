#include "PyUtil.h"
#include "PyReferenced.h"

namespace nb = nanobind;
using namespace cnoid;

namespace {

// incref/decref for a binding-language-owned Referenced object, delegating to
// the bound Python wrapper's reference count. They acquire the GIL because
// addRef()/releaseRef() may be called from any C++ thread.
//
// The decref guards against a finalized interpreter with nb::is_alive(): once the
// Python runtime has shut down, touching it (Py_DECREF) would crash. In that case
// we skip the decref entirely; the object then leaks (its memory and destructor
// are left to process teardown), which is the accepted trade-off for objects
// still referenced from C++ at interpreter exit. See the long comment in
// Referenced.h.
void referencedIncref(void* wrapper) noexcept
{
    nb::gil_scoped_acquire guard;
    Py_INCREF(static_cast<PyObject*>(wrapper));
}

void referencedDecref(void* wrapper) noexcept
{
    if(!nb::is_alive()){
        return;
    }
    nb::gil_scoped_acquire guard;
    Py_DECREF(static_cast<PyObject*>(wrapper));
}

}

namespace cnoid {

void exportPyReferenced(nb::module_& m)
{
    // Install the reference-counting bridge used once an object is in
    // binding-language mode.
    initReferencedPythonInterface({ referencedIncref, referencedDecref });

    // Register Referenced with nanobind's intrusive ownership. The set_self_py
    // hook (calling setSelfPython) is inherited by every Referenced-derived type
    // and hands ownership to the Python wrapper on first exposure.
    nb::class_<Referenced>(
        m, "Referenced",
        nb::intrusive_ptr<Referenced>(
            [](Referenced* p, PyObject* self) noexcept {
                p->setSelfPython(self);
            }));
}

}
