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

    typedef stdx::variant<int, double, bool, std::string> Value;
    enum ValueTypeId { Int, Double, Bool, String };

    ManipulatorVariable& operator=(const ManipulatorVariable& rhs);

    ManipulatorVariable& operator=(const ManipulatorVariable::Value& rhs){
        value_ = rhs; return *this;
    }

    template<class T> T value() const { return stdx::get<T>(value_); }
    template<class T> void setValue(const T& value){ value_ = value; }

    int valueTypeId() const { return stdx::get_variant_index(value_); }

    bool isInt() const { return valueTypeId() == Int; }
    bool isDouble() const { return valueTypeId() == Double; }
    bool isBool() const { return valueTypeId() == Bool; }
    bool isString() const { return valueTypeId() == String; }

    int toInt() const { return stdx::get<int>(value_); }
    double toDouble() const { return stdx::get<double>(value_); }
    bool toBool() const { return stdx::get<bool>(value_); }
    const std::string& toString() const { return stdx::get<std::string>(value_); }

    void changeValueType(int typeId);

    std::string valueString() const;

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
