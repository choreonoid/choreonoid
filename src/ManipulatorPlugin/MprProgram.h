#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_H

#include "MprStatement.h"
#include <cnoid/ClonableReferenced>
#include <cnoid/Signal>
#include <string>
#include <deque>
#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class MprPosition;
class MprPositionList;
class MprProgram;
class MprStructuredStatement;
class ArchiveSession;


class CNOID_EXPORT MprProgram : public ClonableReferenced
{
public:
    typedef std::deque<MprStatementPtr> StatementContainer;
    typedef StatementContainer::iterator iterator;
    typedef StatementContainer::const_iterator const_iterator;

    MprProgram();
    ~MprProgram();

    MprProgram* clone() const {
        return static_cast<MprProgram*>(doClone(nullptr));
    }
    MprProgram* clone(CloneMap& cloneMap) const {
        return static_cast<MprProgram*>(doClone(&cloneMap));
    }

    const std::string& name() const;
    void setName(const std::string& name);

    bool empty() const { return statements_.empty(); }
    int numStatements() const { return statements_.size(); }

    iterator insert(iterator pos, MprStatement* statement, bool doNotify = true);
    iterator append(MprStatement* statement, bool doNotify = true);
    iterator remove(iterator pos, bool doNotify = true);
    bool remove(MprStatement* statement, bool doNotify = true);
    void clearStatements();

    iterator begin(){ return statements_.begin(); }
    const_iterator begin() const { return statements_.begin(); }
    iterator end(){ return statements_.end(); }
    const_iterator end() const { return statements_.end(); }

    void setLocalPositionListEnabled(bool on);
    bool hasLocalPositionList() const;
    MprPositionList* positionList();
    const MprPositionList* positionList() const;
    void removeUnreferencedPositions();

    SignalProxy<void(MprProgram::iterator iter)> sigStatementInserted();
    SignalProxy<void(MprProgram* program, MprStatement* statement)> sigStatementRemoved();
    SignalProxy<void(MprStatement* statement)> sigStatementUpdated();
    
    void notifyStatementUpdate(MprStatement* statement) const;

    MprStructuredStatement* holderStatement() const;
    bool isTopLevelProgram() const;
    bool isSubProgram() const;
    MprProgram* topLevelProgram() const;

    bool traverseAllStatements(std::function<bool(MprStatement* statement)> callback);

    void renumberPositionIds();

    bool load(const std::string& filename, std::ostream& os);
    bool save(const std::string& filename);

    bool read(const Mapping& archive);
    bool write(Mapping& archive) const;

    // Temporary
    ArchiveSession* archiveSession();
    void setArchiveSession(ArchiveSession* session);

protected:
    MprProgram(const MprProgram& org, CloneMap* cloneMap = nullptr);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    StatementContainer statements_;

    class Impl;
    Impl* impl;

    friend class MprStructuredStatement;

    void setHolderStatement(MprStructuredStatement* holder);
};

typedef ref_ptr<MprProgram> MprProgramPtr;

}

#endif
