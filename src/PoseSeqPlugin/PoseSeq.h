/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_H

#include "Pose.h"
#include <set>
#include <list>
#include <cnoid/Body>
#include <cnoid/Signal>
#include <cnoid/ConnectionSet>
#include "exportdecl.h"

namespace cnoid {

class PoseSeq;
typedef ref_ptr<PoseSeq> PoseSeqPtr;

class CNOID_EXPORT PoseRef
{
public:

    inline const std::string& name() const {
        return poseUnit_->name();
    }

    inline const PoseUnitPtr poseUnit() const { return poseUnit_; }
    inline PoseUnitPtr poseUnit() { return poseUnit_; }

    template <class PoseType> inline const ref_ptr<PoseType> get() const {
        return dynamic_pointer_cast<PoseType>(poseUnit_);
    }

    template <class PoseType> inline ref_ptr<PoseType> get() {
        return dynamic_pointer_cast<PoseType>(poseUnit_);
    }

    inline double time() const {
        return time_;
    }

    /**
       This function sets the default transition time.
       When the time length between the previous pose and this pose
       is longer than the default transition time, the sysytem uses this
       time to interpolate the two poses. If the time length between the
       two poses is shorter than the default transition time, the former length
       is used for the interpolation.
       If the default transition time is zero, the interpolation is always done
       using the time length between the two poses.
    */
    inline void setMaxTransitionTime(double time){
        maxTransitionTime_ = time;
    }

    inline double maxTransitionTime() const {
        return maxTransitionTime_;
    }

private:

    friend class PoseSeq;

    PoseRef(PoseSeq* owner, PoseUnitPtr poseUnit, double time);
    //PoseRef(const PoseRef& org, bool doDeepCopy = false);

    PoseSeq* owner;
    PoseUnitPtr poseUnit_;
    double time_;
    double maxTransitionTime_;
};

        
class CNOID_EXPORT PoseSeq : public PoseUnit
{
public:

    typedef std::list<PoseRef>::iterator iterator;
    typedef std::list<PoseRef>::const_iterator const_iterator;

    PoseSeq();
    PoseSeq(const PoseSeq& org);
    ~PoseSeq();

    void setName(const std::string& name);

    void setTargetBodyName(const std::string& name) { targetBodyName_ = name; }
    const std::string& targetBodyName() { return targetBodyName_; }

    iterator copyElement(iterator seekpos, const_iterator org, double offset = 0.0);

    virtual PoseUnit* duplicate();

    bool load(const std::string& filename, const BodyPtr body);
    bool save(const std::string& filename, const BodyPtr body);

    bool exportTalkPluginFile(const std::string& filename);
    bool exportSeqFileForFaceController(const std::string& filename);

    const std::string& errorMessage() { return errorMessage_; }

    virtual bool restore(const Mapping& archive, const BodyPtr body);
    virtual void store(Mapping& archive, const BodyPtr body) const;

    iterator changeTime(iterator it, double time);

    inline bool empty() const {
        return refs.empty();
    }

    inline std::list<PoseRef>::size_type size() const {
        return refs.size();
    }

    inline iterator begin(){
        return refs.begin();
    }

    inline const_iterator begin() const {
        return refs.begin();
    }

    inline iterator end(){
        return refs.end();
    }

    inline const_iterator end() const {
        return refs.end();
    }

    inline PoseRef& front() {
        return refs.front();
    }

    inline PoseRef& back() {
        return refs.back();
    }
            
    PoseUnitPtr find(const std::string& name);

    //iterator seek(double time, bool seekPosToInsert = false);
    iterator seek(iterator current, double time, bool seekPosToInsert = false);

    //iterator insert(double time, PoseUnitPtr pose);
    iterator insert(iterator current, double time, PoseUnitPtr pose);

    iterator insert(iterator current, double time, const std::string& name);
            
    iterator erase(iterator it);

    void rename(iterator it, const std::string newName);

    inline double beginningTime() { return refs.empty() ? 0.0 : refs.front().time(); }
    inline double endingTime() { return refs.empty() ? 0.0 : refs.back().time(); }

    void getDomain(double& out_lower, double& out_upper);

    //SignalProxy<void(PoseUnitPtr, const std::string& oldName)> sigNameChanged();

    /*
      SignalProxy<void(iterator, bool isMoving)> sigPoseInserted() {
      return sigPoseInserted_;
      }

      SignalProxy<void(iterator, bool isMoving)> sigPoseRemoving(){
      return sigPoseRemoving_;
      }

      SignalProxy<void(iterator)> sigPoseModified(){
      return sigPoseModified_;
      }
    */

    /*
      void emitSigPoseModified(iterator it) {
      sigPoseModified_(it);
      }
    */

    void beginPoseModification(iterator it){
        sigPoseModifying_(it);
    }

    void endPoseModification(iterator it){
        sigPoseModified_(it);
    }

    ConnectionSet connectSignalSet(
        const Signal<void(iterator, bool isMoving)>::Function& slotInserted,
        const Signal<void(iterator, bool isMoving)>::Function& slotRemoving,
        const Signal<void(iterator)>::Function& slotModified);

    ConnectionSet connectSignalSet(
        const Signal<void(iterator, bool isMoving)>::Function& slotInserted,
        const Signal<void(iterator, bool isMoving)>::Function& slotRemoving,
        const Signal<void(iterator)>::Function& slotModifying,
        const Signal<void(iterator)>::Function& slotModified);

    /// \todo Implement and use the followings. For example for loading a file.
    void blockSignals();
    void unblockSignals();
            
private:

    typedef std::list<PoseRef> PoseRefList;
    PoseRefList refs;

    typedef std::map<std::string, PoseUnitPtr> PoseUnitMap;
    PoseUnitMap poseUnitMap;
    std::set<std::string> storedNames;

    Signal<void(iterator, bool isMoving)> sigPoseInserted_;
    Signal<void(iterator, bool isMoving)> sigPoseRemoving_;
    Signal<void(iterator)> sigPoseModifying_;
    Signal<void(iterator)> sigPoseModified_;

    std::string targetBodyName_;

    std::string errorMessage_;

    iterator insertSub(PoseSeq::iterator current, double time, PoseUnitPtr poseUnit);
    iterator insert(iterator current, double time, PoseRef& ref);

    friend class PoseRef;
};

}

#endif
