/*!
  @author Shin'ichiro Nakaoka
*/

#include "StdSceneLoader.h"
#include "StdSceneReader.h"
#include "SceneLoader.h"
#include "YAMLReader.h"
#include "ValueTree.h"
#include "NullOut.h"
#include "UTF8.h"
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

struct Registration {
    Registration(){
        SceneLoader::registerLoader(
            "scen",
            []() -> shared_ptr<AbstractSceneLoader> { return std::make_shared<StdSceneLoader>(); });
    }
} registration;

}

namespace cnoid {

class StdSceneLoader::Impl
{
public:
    StdSceneReader sceneReader;
    ostream* os_;

    Impl();
    ostream& os() { return *os_; }
    SgNode* load(const std::string& filename);
};

}
    

StdSceneLoader::StdSceneLoader()
{
    impl = new Impl;
}


StdSceneLoader::Impl::Impl()
{
    os_ = &nullout();
}


StdSceneLoader::~StdSceneLoader()
{
    delete impl;
}


void StdSceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneReader.setMessageSink(os);
}
    

void StdSceneLoader::setDefaultDivisionNumber(int n)
{
    impl->sceneReader.setDefaultDivisionNumber(n);
}


int StdSceneLoader::defaultDivisionNumber() const
{
    return impl->sceneReader.defaultDivisionNumber();
}


SgNode* StdSceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


SgNode* StdSceneLoader::Impl::load(const std::string& filename)
{
    SgNodePtr scene;
    MappingPtr topNode;
    
    try {
        YAMLReader reader;
        topNode = reader.loadDocument(filename)->toMapping();
        if(topNode){
            stdx::filesystem::path filepath(fromUTF8(filename));
            sceneReader.setBaseDirectory(toUTF8(filepath.parent_path().string()));
            sceneReader.readHeader(topNode);
            auto sceneSrc = topNode->find("scene");
            if(!sceneSrc->isValid()){
                os() << format(_("Scene file \"{}\" does not have the \"scene\" node."), filename) << endl;
            } else {
                scene = sceneReader.readScene(sceneSrc);
                if(!scene){
                    os() << format(_("Scene file \"{}\" is an empty scene."), filename) << endl;
                    scene = new SgNode;
                }
            }
        }
    } catch(const ValueNode::Exception& ex){
        os() << ex.message();
    }

    os().flush();

    sceneReader.clear();
    
    return scene.retn();
}
