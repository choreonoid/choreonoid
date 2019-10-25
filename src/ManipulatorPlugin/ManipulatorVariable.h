#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_VARIBLE_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_VARIBLE_H

#include <cnoid/GeneralId>
#include <cnoid/CloneMappableReferenced>
#include <cnoid/stdx/variant>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ManipulatorVariableSet;

class CNOID_EXPORT ManipulatorVariable : public CloneMappableReferenced
{
public:
    ManipulatorVariable();
    ManipulatorVariable(const GeneralId& id);
    ManipulatorVariable(const ManipulatorVariable& org);

    typedef stdx::variant<bool, int, double, std::string> Value;
    enum ValueType { Bool, Int, Double, String };

    ManipulatorVariable& operator=(const ManipulatorVariable& rhs);

    ManipulatorVariable& operator=(const ManipulatorVariable::Value& rhs){
        value_ = rhs; return *this;
    }

    /*
    ManipulatorVariable& operator=(bool value){
        value_ = value; return *this;
    }
    ManipulatorVariable& operator=(int value){
        value_ = value; return *this;
    }
    ManipulatorVariable& operator=(double value){
        value_ = value; return *this;
    }
    ManipulatorVariable& operator=(const std::string& value){
        value_ = value; return *this;
    }
    */

    template<class T> T get() const { return stdx::get<T>(value_); }

    bool toBool() const { return stdx::get<bool>(value_); }
    int toInt() const { return stdx::get<int>(value_); }
    double toDouble() const { return stdx::get<double>(value_); }
    const std::string& toString() const { return stdx::get<std::string>(value_); }

    const GeneralId& id() const { return id_; }

    const std::string& note() const { return note_; }
    void setNote(const std::string& note) { note_ = note; }

    ManipulatorVariableSet* ownerVariableSet() const;

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Value value_;
    GeneralId id_;
    std::string note_;
    weak_ref_ptr<ManipulatorVariableSet> ownerVariableSet_;

    friend class ManipulatorVariableSet;
};

typedef ref_ptr<ManipulatorVariable> ManipulatorVariablePtr;

}

#endif
