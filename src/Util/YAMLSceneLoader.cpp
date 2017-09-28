/*!
  @author Shin'ichiro Nakaoka
*/

#include "YAMLSceneLoader.h"
#include "YAMLSceneReader.h"
#include "SceneLoader.h"
#include "YAMLReader.h"
#include "ValueTree.h"
#include "NullOut.h"
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;

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
            boost::filesystem::path filepath(filename);
            sceneReader.setBaseDirectory(filepath.parent_path().string());
            sceneReader.readHeader(*topNode);
            ValueNodePtr sceneElements = topNode->findMapping("scene");
            if(!sceneElements->isValid()){
                os() << (format(_("Scene file \"%1%\" does not have the \"scene\" node.")) % filename) << endl;
            } else {
                scene = sceneReader.readNodeList(*sceneElements);
                if(!scene){
                    os() << (format(_("Scene file \"%1%\" is an empty scene.")) % filename) << endl;
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
