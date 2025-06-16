#ifndef CNOID_UTIL_CLONABLE_REFERENCED_H
#define CNOID_UTIL_CLONABLE_REFERENCED_H

#include "Referenced.h"

namespace cnoid {

class CloneMap;

class ClonableReferenced : public Referenced
{
public:
    /**
     * Override this method to implement cloning with optional CloneMap support.
     * 
     * The doClone() method follows the Template Method pattern, where derived
     * classes implement the actual cloning logic. The 'do' prefix indicates
     * this is an implementation method to be overridden.
     * 
     * Derived classes can use covariant return types to return their own type:
     *   class Derived : public ClonableReferenced {
     *       virtual Derived* doClone(CloneMap* cloneMap) const override {
     *           return new Derived(*this);
     *       }
     *   };
     * 
     * For better usability, derived classes may also provide public clone() methods
     * that delegate to doClone(), as demonstrated in the ValueTree classes.
     */
    virtual Referenced* doClone(CloneMap* cloneMap) const = 0;
};

}

#endif
