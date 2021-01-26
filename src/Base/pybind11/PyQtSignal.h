#ifndef CNOID_BASE_PYQTSIGNAL_H
#define CNOID_BASE_PYQTSIGNAL_H

#include <cnoid/PyUtil>
#include <QObject>

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

template<typename QtSignal, typename Module>
void PyQtSignal(Module& m, const std::string& name)
{
    pybind11::class_<QtSignal>(m, name.c_str())
        .def("connect",
             [](QtSignal& self, std::function<typename QtSignal::Slot> slot){
                 return self.connect(slot);
             });
}

}

#endif

