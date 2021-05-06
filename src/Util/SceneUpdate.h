#ifndef CNOID_UTIL_SCENE_UPDATE_H
#define CNOID_UTIL_SCENE_UPDATE_H

#include <vector>

namespace cnoid {

class SgObject;

class SgUpdate
{
public:
    enum Action {
        None = 0,
        Added = 1 << 0,
        Removed = 1 << 1,
        GeometryModified = 1 << 2,
        AppearanceModified = 1 << 3,
        Modified = GeometryModified | AppearanceModified,

        // deprecated
        NONE = None,
        ADDED = Added,
        REMOVED = Removed,
        MODIFIED = Modified,
        BBOX_UPDATED = GeometryModified
    };

    typedef std::vector<SgObject*> Path;

    SgUpdate() : action_(MODIFIED), initialPathCapacity_(0) {  }
    SgUpdate(int action) : action_(action), initialPathCapacity_(0) { }
    SgUpdate(const SgUpdate& org)
        : path_(org.path_), action_(org.action_), initialPathCapacity_(0) { }
    ~SgUpdate() { }
    void setInitialPathCapacity(unsigned char n) { initialPathCapacity_ = n; }
    void reservePathCapacity(int n) { path_.reserve(n); }
    int action() const { return action_; }
    bool hasAction(int act) const { return action_ & act; }
    SgUpdate& withAction(int act) { action_ = act; return *this; }
    void setAction(int act) { action_ = act; }
    void addAction(int act) { action_ |= act; }
    const Path& path() const { return path_; }
    void pushNode(SgObject* node) { path_.push_back(node); }
    void popNode() { path_.pop_back(); }
    void clearPath() {
        if(path_.empty()){
            path_.reserve(initialPathCapacity_);
        } else  {
            path_.clear();
        }
    }

    [[deprecated("Use setAction.")]]
    void resetAction(int act = None) { setAction(act); }
    [[deprecated("Use clearPath()")]]
    void clear() { clearPath(); }

private:
    Path path_;
    char action_;
    unsigned char initialPathCapacity_;
};


class SgTmpUpdate : public SgUpdate
{
public:
    SgTmpUpdate() { setInitialPathCapacity(16); }
};


class SgUpdateRef
{
    SgUpdate* pUpdate;
    bool isTmpUpdate;
    
public:
    SgUpdateRef() : pUpdate(nullptr), isTmpUpdate(false) { }
    SgUpdateRef(SgUpdate& update) : pUpdate(&update), isTmpUpdate(false) { }
    SgUpdateRef(SgUpdate* pUpdate) : pUpdate(pUpdate), isTmpUpdate(false) { }
    SgUpdateRef(const SgUpdateRef& org) : pUpdate(org.pUpdate), isTmpUpdate(false) { }
    SgUpdateRef(bool doCreateTmpUpdate) : isTmpUpdate(doCreateTmpUpdate) {
        if(doCreateTmpUpdate){
            pUpdate = new SgTmpUpdate;
        } else {
            pUpdate = nullptr;
        }
    }
    ~SgUpdateRef() { if(isTmpUpdate) delete pUpdate; }
    operator bool() const { return pUpdate != nullptr; }
    operator SgUpdate*() { return pUpdate; }
    SgUpdate& operator*() { return *pUpdate; }
    SgUpdate* operator->() { return pUpdate; }
};

}

#endif
