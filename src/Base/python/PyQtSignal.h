#ifndef CNOID_BASE_PYTHON_PYQTSIGNAL_H
#define CNOID_BASE_PYTHON_PYQTSIGNAL_H

#include <cnoid/PySignal>
#include <QObject>

/*
   The std::function type caster used for connecting a Python callable as a Qt
   slot is the exception-safe one defined in the Util binding's PySignal.h
   (included via <cnoid/PySignal>). It reports an error raised on the Python
   side through sys.unraisablehook instead of letting the C++ exception escape
   into the Qt event loop, where it would crash the process.
*/

namespace cnoid {

template<typename SignalFuncPointer, typename SlotSignature>
class QtSignal
{
    typedef typename QtPrivate::FunctionPointer<SignalFuncPointer>::Object Sender;
    Sender* sender;
    SignalFuncPointer signal;

public:
    typedef SlotSignature Slot;

    QtSignal(Sender* sender, SignalFuncPointer signal)
        : sender(sender), signal(signal)
    {
        QObject::connect(
            sender, &QObject::destroyed,
            [this](QObject* /* obj */){
                this->sender = nullptr;
            });
    }
    QMetaObject::Connection connect(std::function<SlotSignature> slot){
        if(sender){
            return QObject::connect(sender, signal, slot);
        }
        return QMetaObject::Connection();
    }
};

template<typename QtSignalType, typename Parent>
void PyQtSignal(Parent& parent, const std::string& name)
{
    nanobind::class_<QtSignalType>(parent, name.c_str())
        .def("connect",
             [](QtSignalType& self, std::function<typename QtSignalType::Slot> slot){
                 return self.connect(slot);
             });
}

}

#endif
