#ifndef CNOID_UTIL_POSITION_TAG_LIST_H
#define CNOID_UTIL_POSITION_TAG_LIST_H

#include "PositionTag.h"
#include "Signal.h"
#include "EigenTypes.h"
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class PositionTag;
class Mapping;

class CNOID_EXPORT PositionTagList : public Referenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    PositionTagList();
    PositionTagList(const PositionTagList& org);
    virtual ~PositionTagList();

    void clearTags();

    int numTags() const {
        return tags_.size();
    }
    
    const PositionTag* tagAt(int index) const {
        return tags_[index];
    }
    PositionTag* tagAt(int index) {
        return tags_[index];
    }

    const Position& basePosition() const {
        return T_base_;
    }
    
    template<class Scalar, int Mode, int Options>
    void setBasePosition(const Eigen::Transform<Scalar, 3, Mode, Options>& T, bool doNotify){
        T_base_ = T.template cast<Position::Scalar>();
        if(doNotify){
            notifyBasePositionUpdate();
        }
    }

    typedef std::vector<PositionTagPtr> Container;
    Container::const_iterator begin() const { return tags_.begin(); }
    Container::const_iterator end() const { return tags_.end(); }
    
    void insert(int index, PositionTag* point);
    void append(PositionTag* point);
    bool removeAt(int index);
    SignalProxy<void(int index)> sigTagAdded();
    SignalProxy<void(int index, PositionTag* tag)> sigTagRemoved();
    SignalProxy<void(int index)> sigTagUpdated();
    SignalProxy<void(const Position& T)> sigBasePositionChanged();

    void notifyTagUpdate(int index);
    void notifyBasePositionUpdate();
    
    bool read(const Mapping* archive);
    void write(Mapping* archive) const;

private:
    Container tags_;
    Position T_base_;
    
    class Impl;
    Impl* impl;
};

typedef ref_ptr<PositionTagList> PositionTagListPtr;

}

#endif
