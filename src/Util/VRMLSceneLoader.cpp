#include "VRMLSceneLoader.h"
#include "SceneLoader.h"
#include "VRMLParser.h"
#include "VRMLToSGConverter.h"
#include "EasyScanner.h"
#include "NullOut.h"
#include "Format.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

struct Registration {
    Registration(){
        SceneLoader::registerLoader(
            "wrl",
            []() -> shared_ptr<AbstractSceneLoader> { return std::make_shared<VRMLSceneLoader>(); });
    }
} registration;

}

namespace cnoid {

class VRMLSceneLoader::Impl
{
public:
    VRMLParser parser;
    VRMLToSGConverter converter;
    ostream* os_;

    Impl();
    ostream& os() { return *os_; }
    SgNode* load(const std::string& filename);
};

}
    

VRMLSceneLoader::VRMLSceneLoader()
{
    impl = new Impl;
}


VRMLSceneLoader::Impl::Impl()
{
    os_ = &nullout();
}


VRMLSceneLoader::~VRMLSceneLoader()
{
    delete impl;
}


void VRMLSceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->parser.setMessageSink(os);
    impl->converter.setMessageSink(os);
}
    

SgNode* VRMLSceneLoader::load(const std::string& filename)
{
    return insertTransformNodesToAdjustLengthUnitAndUpperAxis(impl->load(filename));
}


SgNode* VRMLSceneLoader::Impl::load(const std::string& filename)
{
    converter.clearConvertedNodeMap();

    SgGroupPtr group = new SgGroup;

    try {
        parser.load(filename);
        
        while(VRMLNodePtr vrml = parser.readNode()){
            SgNodePtr node = converter.convert(vrml);
            if(node){
                group->addChild(node);
            }
        }
        parser.checkEOF();

    } catch(EasyScanner::Exception& ex){
        os() << ex.getFullMessage() << endl;
        return 0;
    }

    if(group->empty()){
        os() << formatR(_("VRML file \"{}\" does not have any valid entity."), filename) << endl;
        return nullptr;
    }

    SgNodePtr node = group;
    if(group->numChildren() == 1){
        node = group->child(0);
        group->removeChildAt(0);
    }
    group.reset();

    node->setUriWithFilePathAndCurrentDirectory(filename);

    return node.retn();
}
