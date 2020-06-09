#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_VARIBLE_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_VARIBLE_H

#include <cnoid/Referenced>
#include <cnoid/GeneralId>
#include <cnoid/Signal>
#include <cnoid/stdx/variant>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class MprVariableList;

class CNOID_EXPORT MprVariable : public Referenced
{
public:
    typedef stdx::variant<int, double, bool, std::string> Value;
    enum TypeId { Int, Double, Bool, String };

    MprVariable();
    MprVariable(const GeneralId& id, Value value = 0);
    MprVariable(const MprVariable& org);
    MprVariable& operator=(const MprVariable&) = delete;

    static int valueType(const Value& value) { return stdx::get_variant_index(value); }
    static int intValue(const Value& value) { return stdx::get<int>(value); }
    static double doubleValue(const Value& value) { return stdx::get<double>(value); }
    static bool boolValue(const Value& value) { return stdx::get<bool>(value); }
    static const std::string& stringValue(const Value& value) { return stdx::get<std::string>(value); }

    const GeneralId& id() const { return id_; }
    bool resetId(const GeneralId& id);

    int valueType() const { return stdx::get_variant_index(value_); }
    bool changeValueType(TypeId type);
    
    bool isInt() const { return valueType() == Int; }
    bool isDouble() const { return valueType() == Double; }
    bool isBool() const { return valueType() == Bool; }
    bool isString() const { return valueType() == String; }

    template<class T> T value() const { return stdx::get<T>(value_); }
    Value value() const { return value_; }
    int intValue() const { return stdx::get<int>(value_); }
    double doubleValue() const { return stdx::get<double>(value_); }
    bool boolValue() const { return stdx::get<bool>(value_); }
    const std::string& stringValue() const { return stdx::get<std::string>(value_); }

    bool setValue(int value);
    bool setValue(double value);
    bool setValue(bool value);
    bool setValue(const std::string& value);
    bool setValue(const Value& value);

    std::string toString() const;

    const std::string& note() const { return note_; }
    void setNote(const std::string& note) { note_ = note; }

    MprVariableList* ownerVariableList() const;

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;

    enum UpdateFlag {
        IdUpdate = 1 << 0,
        ValueUpdate = 1 << 1,
        NoteUpdate = 1 << 2
    };
    SignalProxy<void(int flags)> sigUpdated() { return sigUpdated_; }
    void notifyUpdate(int flags = ValueUpdate);

private:
    Value value_;
    GeneralId id_;
    std::string note_;
    weak_ref_ptr<MprVariableList> ownerVariableList_;
    Signal<void(int flags)> sigUpdated_;

    friend class MprVariableList;

    template<class ValueType>
    bool setScalarValue(ValueType value);
};

typedef ref_ptr<MprVariable> MprVariablePtr;

}

#endif
