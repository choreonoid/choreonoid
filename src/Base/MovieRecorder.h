/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_MOVIE_RECORDER_H
#define CNOID_BASE_MOVIE_RECORDER_H

#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class MovieRecorderImpl;

class CNOID_EXPORT MovieRecorder
{
  public:
    static void initialize(ExtensionManager* ext);
    static MovieRecorder* instance();
    ~MovieRecorder();

  private:
    MovieRecorder(ExtensionManager* ext);

    MovieRecorderImpl* impl;
};

}

#endif
