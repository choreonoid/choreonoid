#ifndef CNOID_POSE_SEQ_PLUGIN_PRONUN_SYMBOL_H
#define CNOID_POSE_SEQ_PLUGIN_PRONUN_SYMBOL_H

#include "AbstractPose.h"
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PronunSymbol : public AbstractPose
{
public:
    PronunSymbol();
    PronunSymbol(const PronunSymbol& org);
    virtual ~PronunSymbol();

    const std::string& symbol() const {
        return symbol_;
    }
    void setSymbol(const std::string& symbol){
        symbol_ = symbol;
    }
    
    virtual bool restore(const Mapping& archive, const Body* body) override;
    virtual void store(Mapping& archive, const Body* body) const override;

protected:
    virtual Referenced* doClone(CloneMap*) const override;

private:
    std::string symbol_;
};

typedef ref_ptr<PronunSymbol> PronunSymbolPtr;

}

#endif
