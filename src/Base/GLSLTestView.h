/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_GLSL_TEST_VIEW_H
#define CNOID_BASE_GLSL_TEST_VIEW_H

#include "View.h"

namespace cnoid {

class GLSLTestViewImpl;

class GLSLTestView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    GLSLTestView();
    ~GLSLTestView();
        
private:
    GLSLTestViewImpl* impl;
};

}

#endif
