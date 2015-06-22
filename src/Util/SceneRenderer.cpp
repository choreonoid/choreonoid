/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneRenderer.h"

using namespace std;
using namespace cnoid;


bool SceneRenderer::getSimplifiedCameraPathStrings(int index, std::vector<std::string>& out_pathStrings)
{
    out_pathStrings.clear();
    
    if(index < numCameras()){
        const SgNodePath& path = cameraPath(index);
        const string& name = path.back()->name();
        if(!name.empty()){
            size_t n = path.size() - 1;
            for(size_t i=0; i < n; ++i){
                const string& element = path[i]->name();
                if(!element.empty()){
                    out_pathStrings.push_back(element);
                    break;
                }
            }
            out_pathStrings.push_back(name);
        }
    }
    return !out_pathStrings.empty();

}


/**
   @return Camera index, or -1 if the path is not found.
*/
int SceneRenderer::findCameraPath(const std::vector<std::string>& simplifiedPathStrings)
{
    int index = -1;
    
    if(!simplifiedPathStrings.empty()){
        vector<int> candidates;
        const string& name = simplifiedPathStrings.back();
        const int n = numCameras();
        for(size_t i=0; i < n; ++i){
            const vector<SgNode*>& path = cameraPath(i);
            if(path.back()->name() == name){
                candidates.push_back(i);
            }
        }
        if(candidates.size() == 1){
            index = candidates.front();

        } else if(candidates.size() >= 2 && simplifiedPathStrings.size() >= 2){
            const string& owner = simplifiedPathStrings.front();
            for(size_t i=0; i < candidates.size(); ++i){
                const vector<SgNode*>& path = cameraPath(i);
                if(path.front()->name() == owner){
                    index = i;
                    break;
                }
            }
        }
    }

    return index;
}


bool SceneRenderer::setCurrentCameraPath(const std::vector<std::string>& simplifiedPathStrings)
{
    int index = findCameraPath(simplifiedPathStrings);
    if(index >= 0){
        setCurrentCamera(index);
        return true;
    }
    return false;
}
