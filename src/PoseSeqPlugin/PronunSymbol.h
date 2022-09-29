#ifndef CNOID_POSE_SEQ_PLUGIN_PRONUN_SYMBOL_H
#define CNOID_POSE_SEQ_PLUGIN_PRONUN_SYMBOL_H

#include "AbstractPose.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PronunSymbol : public AbstractPose
{
public:
    PronunSymbol();
    PronunSymbol(const PronunSymbol& org);
    virtual ~PronunSymbol();

    virtual bool restore(const Mapping& archive, const Body* body);
    virtual void store(Mapping& archive, const Body* body) const;

protected:
    virtual Referenced* doClone(CloneMap*) const override;    
};

typedef ref_ptr<PronunSymbol> PronunSymbolPtr;

}

#endif
