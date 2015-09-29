/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PARAMETRIC_PATH_PROCESSOR_H
#define CNOID_BASE_PARAMETRIC_PATH_PROCESSOR_H

#include <string>
#include <boost/optional.hpp>
#include "exportdecl.h"

namespace cnoid {

class Mapping;
class ParametricPathProcessorImpl;

class CNOID_EXPORT ParametricPathProcessor
{
public:
    static ParametricPathProcessor* instance();
    
    ParametricPathProcessor();
    ~ParametricPathProcessor();

    void setVariables(Mapping* variables);
    void setBaseDirectory(const std::string& directory);
    std::string parameterize(const std::string& path) const;
    boost::optional<std::string> expand(const std::string& path) const;
    const std::string& errorMessage() const;

private:
    ParametricPathProcessorImpl* impl;
};

}

#endif
