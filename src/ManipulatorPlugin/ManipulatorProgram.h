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
    ~ManipulatorProgramCloneMap();
    ManipulatorProgram* getClone(ManipulatorProgram* org);
    ManipulatorPosition* getClone(ManipulatorPosition* org);
    ManipulatorPositionCloneMap& manipulatorPositionCloneMap();
    bool isPositionSetIncluded() const { return isPositionSetIncluded_; }
    void setPositionSetIncluded(bool on){ isPositionSetIncluded_ = on; }
    
private:
    class Impl;
    Impl* impl;
    bool isPositionSetIncluded_;

    friend class ManipulatorProgram;
};

class CNOID_EXPORT ManipulatorProgram : public Referenced
{
public:
    typedef std::deque<ManipulatorStatementPtr> StatementContainer;
    typedef StatementContainer::iterator iterator;

    ManipulatorProgram();
    ~ManipulatorProgram();

    virtual ManipulatorProgram* clone(ManipulatorProgramCloneMap& cloneMap);
    
    const std::string& name() const;
    void setName(const std::string& name);

    iterator insert(const iterator& pos, ManipulatorStatement* statement);
    iterator append(ManipulatorStatement* statement);
    bool remove(ManipulatorStatement* statement);

    iterator begin(){ return statements_.begin(); }
    iterator end(){ return statements_.end(); }

    ManipulatorPositionSet* positions();
    const ManipulatorPositionSet* positions() const;
    void removeUnreferencedPositions();

    bool load(const std::string& filename, std::ostream& os);
    bool save(const std::string& filename);

protected:
    ManipulatorProgram(const ManipulatorProgram& org, ManipulatorProgramCloneMap& cloneMap);

private:
    bool read(Mapping& archive);

    StatementContainer statements_;

    class Impl;
    Impl* impl;
};

typedef ref_ptr<ManipulatorProgram> ManipulatorProgramPtr;

}

#endif
