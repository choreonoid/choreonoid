/*!
  @author Shin'ichiro Nakaoka
*/

#include "YAMLSceneLoader.h"
#include "YAMLSceneReader.h"
#include "SceneLoader.h"
#include "YAMLReader.h"
#include "ValueTree.h"
#include "NullOut.h"
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
            []() -> shared_ptr<AbstractSceneLoader> { return std::make_shared<YAMLSceneLoader>(); });
    }
} registration;

}

namespace cnoid {

class YAMLSceneLoaderImpl
{
public:
    YAMLSceneReader sceneReader;
    ostream* os_;

    YAMLSceneLoaderImpl();
    ostream& os() { return *os_; }
    SgNode* load(const std::string& filename);
};

}
    

YAMLSceneLoader::YAMLSceneLoader()
{
    impl = new YAMLSceneLoaderImpl;
}


YAMLSceneLoaderImpl::YAMLSceneLoaderImpl()
{
    os_ = &nullout();
}


YAMLSceneLoader::~YAMLSceneLoader()
{
    delete impl;
}


void YAMLSceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneReader.setMessageSink(os);
}
    

void YAMLSceneLoader::setDefaultDivisionNumber(int n)
{
    impl->sceneReader.setDefaultDivisionNumber(n);
}


int YAMLSceneLoader::defaultDivisionNumber() const
{
    return impl->sceneReader.defaultDivisionNumber();
}


SgNode* YAMLSceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


SgNode* YAMLSceneLoaderImpl::load(const std::string& filename)
{
    SgNodePtr scene;
    MappingPtr topNode;
    
    try {
        YAMLReader reader;
        topNode = reader.loadDocument(filename)->toMapping();
        if(topNode){
            stdx::filesystem::path filepath(filename);
            sceneReader.setBaseDirectory(filepath.parent_path().string());
            sceneReader.readHeader(*topNode);
            ValueNodePtr sceneElements = topNode->findMapping("scene");
            if(!sceneElements->isValid()){
                os() << format(_("Scene file \"{}\" does not have the \"scene\" node."), filename) << endl;
            } else {
                scene = sceneReader.readNodeList(*sceneElements);
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
