#include "Referenced.h"

using namespace cnoid;

// Function pointers delegating reference counting to the bound wrapper object.
// They are only dereferenced after the object has entered binding-language mode
// (via setSelfPython), which the binding layer does only after installing them
// through initReferencedPythonInterface(). In a pure C++ program they stay null
// and are never called.
ReferencedPythonInterface Referenced::pythonInterface = { nullptr, nullptr };

namespace cnoid {

void initReferencedPythonInterface(const ReferencedPythonInterface& iface)
{
    Referenced::pythonInterface = iface;
}

}

Referenced::~Referenced()
{
    WeakCounter* wc = weakCounter_.load(std::memory_order_acquire);
    if(wc){
        wc->isObjectAlive_ = false;
        wc->release();
    }
}
