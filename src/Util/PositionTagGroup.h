#ifndef CNOID_UTIL_POSITION_TAG_GROUP_H
#define CNOID_UTIL_POSITION_TAG_GROUP_H

#include "ClonableReferenced.h"
#include "PositionTag.h"
#include "Signal.h"
#include "EigenTypes.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class PositionTag;
class Mapping;

class CNOID_EXPORT PositionTagGroup : public ClonableReferenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PositionTagGroup();
    PositionTagGroup(const PositionTagGroup& org);
    virtual ~PositionTagGroup();

    PositionTagGroup* clone() const {
        return static_cast<PositionTagGroup*>(doClone(nullptr));
    }
    PositionTagGroup* clone(CloneMap& cloneMap) const {
        return static_cast<PositionTagGroup*>(doClone(&cloneMap));
    }

    const std::string& name() const;
    void setName(const std::string& name);

    void clearTags();

    bool empty() const { return tags_.empty(); }
    int numTags() const { return tags_.size(); }
    
    const PositionTag* tagAt(int index) const {
        return tags_[index];
    }
    PositionTag* tagAt(int index) {
        return tags_[index];
    }

    typedef std::vector<PositionTagPtr> Container;
    Container::const_iterator begin() const { return tags_.begin(); }
    Container::const_iterator end() const { return tags_.end(); }
    
    void insert(int index, PositionTag* tag);
    void insert(int index, PositionTagGroup* group);
    void append(PositionTag* tag);
    bool removeAt(int index);
    SignalProxy<void(int index)> sigTagAdded();
    SignalProxy<void(int index, PositionTag* tag)> sigTagRemoved();
    SignalProxy<void(int index)> sigTagPositionChanged();
    SignalProxy<void(int index)> sigTagPositionUpdated();
    void notifyTagPositionChange(int index);
    void notifyTagPositionUpdate(int index, bool doNotifyPositionChange = true);

    bool read(const Mapping* archive);
    bool write(Mapping* archive) const;

    enum CsvFormat { XYZMMRPYDEG = 0, XYZMM = 1 };
    bool loadCsvFile(const std::string& filename, CsvFormat csvFormat, std::ostream& os);

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    Container tags_;
    
    class Impl;
    Impl* impl;
};

typedef ref_ptr<PositionTagGroup> PositionTagGroupPtr;

}

#endif
