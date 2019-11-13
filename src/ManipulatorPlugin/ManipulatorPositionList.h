#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_POSITION_LIST_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_POSITION_LIST_H

#include <cnoid/CloneableReferenced>
#include "exportdecl.h"

namespace cnoid {

class ManipulatorPosition;
class GeneralId;
class Mapping;

class CNOID_EXPORT ManipulatorPositionList : public CloneableReferenced
{
public:
    ManipulatorPositionList();
    ~ManipulatorPositionList();

    ManipulatorPositionList* clone() const {
        return static_cast<ManipulatorPositionList*>(doClone(nullptr));
    }
    ManipulatorPositionList* clone(CloneMap& cloneMap) const {
        return static_cast<ManipulatorPositionList*>(doClone(&cloneMap));
    }

    void setStringIdEnabled(bool on);
    bool isStringIdEnabled() const;

    void clear();
    int numPositions() const;
    ManipulatorPosition* positionAt(int index) const;
    int indexOf(ManipulatorPosition* position);

    ManipulatorPosition* findPosition(const GeneralId& id) const;
    
    bool insert(int index, ManipulatorPosition* position);
    bool append(ManipulatorPosition* position);
    void removeAt(int index);

    /**
       @return true if the id is successfully changed. false if the id is not
       changed because anther coordinate frame with the same id is exists.
    */
    bool resetId(ManipulatorPosition* position, const GeneralId& newId);

    int removeUnreferencedPositions(std::function<bool(ManipulatorPosition* position)> isReferenced);
    
    void resetIdCounter();
    GeneralId createNextId(int prevId = -1);

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;
    
    class Impl;

protected:
    ManipulatorPositionList(const ManipulatorPositionList& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Impl* impl;

    friend class ManipulatorPosition;
};

typedef ref_ptr<ManipulatorPositionList> ManipulatorPositionListPtr;

}

#endif
