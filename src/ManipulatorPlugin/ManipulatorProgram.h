#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_H

#include "ManipulatorStatements.h"
#include <cnoid/Referenced>
#include <string>
#include <deque>
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class ManipulatorPosition;
class ManipulatorPositionCloneMap;
class ManipulatorPositionSet;
class ManipulatorProgram;

class CNOID_EXPORT ManipulatorProgramCloneMap
{
public:
    ManipulatorProgramCloneMap();
    ManipulatorProgramCloneMap(const ManipulatorProgramCloneMap& org) = delete;
    ~ManipulatorProgramCloneMap();
    void clear();
    ManipulatorProgram* getClone(ManipulatorProgram* org);
    ManipulatorPosition* getClone(ManipulatorPosition* org);
    ManipulatorPositionCloneMap& manipulatorPositionCloneMap();
    bool isPositionSetIncluded() const;
    void setPositionSetIncluded(bool on);
    
private:
    class Impl;
    Impl* impl;

    friend class ManipulatorProgram;
};

class CNOID_EXPORT ManipulatorProgram : public Referenced
{
public:
    typedef std::deque<ManipulatorStatementPtr> StatementContainer;
    typedef StatementContainer::iterator iterator;
    typedef StatementContainer::const_iterator const_iterator;

    ManipulatorProgram();
    ~ManipulatorProgram();

    ManipulatorProgram* clone() const { return doClone(nullptr); }
    ManipulatorProgram* clone(ManipulatorProgramCloneMap& cloneMap) const { return doClone(&cloneMap); }
    
    const std::string& name() const;
    void setName(const std::string& name);

    bool empty() const { return statements_.empty(); }
    int numStatements() const { return statements_.size(); }

    iterator insert(iterator pos, ManipulatorStatement* statement);
    iterator append(ManipulatorStatement* statement);
    iterator remove(iterator pos);
    bool remove(ManipulatorStatement* statement);

    iterator begin(){ return statements_.begin(); }
    const_iterator begin() const { return statements_.begin(); }
    iterator end(){ return statements_.end(); }
    const_iterator end() const { return statements_.end(); }

    ManipulatorPositionSet* positions();
    const ManipulatorPositionSet* positions() const;
    void removeUnreferencedPositions();
    ManipulatorPositionSet* createPositionSet() const;

    bool load(const std::string& filename, std::ostream& os);
    bool save(const std::string& filename);

protected:
    ManipulatorProgram(const ManipulatorProgram& org, ManipulatorProgramCloneMap* cloneMap);
    virtual ManipulatorProgram* doClone(ManipulatorProgramCloneMap* cloneMap) const;

private:
    bool read(Mapping& archive);

    StatementContainer statements_;

    class Impl;
    Impl* impl;
};

typedef ref_ptr<ManipulatorProgram> ManipulatorProgramPtr;

}

#endif
