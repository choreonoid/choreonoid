#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_H

#include "ManipulatorStatements.h"
#include <cnoid/Referenced>
#include <cnoid/Signal>
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
class StructuredStatement;

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

    iterator insert(iterator pos, ManipulatorStatement* statement, bool doNotify = true);
    iterator append(ManipulatorStatement* statement, bool doNotify = true);
    iterator remove(iterator pos, bool doNotify = true);
    bool remove(ManipulatorStatement* statement, bool doNotify = true);

    iterator begin(){ return statements_.begin(); }
    const_iterator begin() const { return statements_.begin(); }
    iterator end(){ return statements_.end(); }
    const_iterator end() const { return statements_.end(); }

    ManipulatorPositionSet* positions();
    const ManipulatorPositionSet* positions() const;
    void removeUnreferencedPositions();
    ManipulatorPositionSet* createPositionSet() const;

    SignalProxy<void(ManipulatorProgram* program, ManipulatorProgram::iterator iter)> sigStatementInserted();
    SignalProxy<void(ManipulatorProgram* program, ManipulatorStatement* statement)> sigStatementRemoved();
    SignalProxy<void(ManipulatorStatement* statement)> sigStatementUpdated();
    
    void notifyStatementUpdate(ManipulatorStatement* statement) const;

    StructuredStatement* holderStatement() const;

    bool load(const std::string& filename, std::ostream& os);
    bool save(const std::string& filename);

protected:
    ManipulatorProgram(const ManipulatorProgram& org, ManipulatorProgramCloneMap* cloneMap);
    virtual ManipulatorProgram* doClone(ManipulatorProgramCloneMap* cloneMap) const;

private:
    StatementContainer statements_;

    class Impl;
    Impl* impl;

    friend class StructuredStatement;

    void setHolderStatement(StructuredStatement* holder);
};

typedef ref_ptr<ManipulatorProgram> ManipulatorProgramPtr;

}

#endif
