#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LIST_H

#include <cnoid/ClonableReferenced>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class MprPosition;
class GeneralId;
class Mapping;

class CNOID_EXPORT MprPositionList : public ClonableReferenced
{
public:
    MprPositionList();
    ~MprPositionList();

    MprPositionList* clone() const {
        return static_cast<MprPositionList*>(doClone(nullptr));
    }
    MprPositionList* clone(CloneMap& cloneMap) const {
        return static_cast<MprPositionList*>(doClone(&cloneMap));
    }

    void setStringIdEnabled(bool on);
    bool isStringIdEnabled() const;

    void clear();
    int numPositions() const;
    const MprPosition* positionAt(int index) const;
    MprPosition* positionAt(int index);
    int indexOf(MprPosition* position);

    MprPosition* findPosition(const GeneralId& id) const;
    
    bool insert(int index, MprPosition* position);
    bool append(MprPosition* position);
    bool replace(int index, MprPosition* position);
    void removeAt(int index);

    SignalProxy<void(int index)> sigPositionAdded();
    SignalProxy<void(int index, MprPosition* position)> sigPositionRemoved();
    SignalProxy<void(int index, int flags)> sigPositionUpdated();

    /**
       @return true if the id is successfully changed. false if the id is not
       changed because anther coordinate frame with the same id is exists.
    */
    bool resetId(MprPosition* position, const GeneralId& newId);

    int removeUnreferencedPositions(std::function<bool(MprPosition* position)> isReferenced);
    
    void resetIdCounter();
    GeneralId createNextId(int prevId = -1);

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;
    
    class Impl;

protected:
    MprPositionList(const MprPositionList& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    // Called from the CoordinateFrame implementation
    void notifyPositionUpdate(MprPosition* position, int flags);
    
    Impl* impl;

    friend class MprPosition;
};

typedef ref_ptr<MprPositionList> MprPositionListPtr;

}

#endif
