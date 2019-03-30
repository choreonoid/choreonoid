/*!
  @file
  @author Shizuko Hattori
*/

#include "SceneInfo_impl.h"
#include "BodyInfo_impl.h"
#include <cnoid/BodyLoader>
#include <cnoid/Body>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;


SceneInfo_impl::SceneInfo_impl(PortableServer::POA_ptr poa) :
    ShapeSetInfo_impl(poa)
{
    os = &cout;
}


SceneInfo_impl::~SceneInfo_impl()
{
    
}


char* SceneInfo_impl::url()
{
    return CORBA::string_dup(url_.c_str());
}


TransformedShapeIndexSequence* SceneInfo_impl::shapeIndices()
{
    return new TransformedShapeIndexSequence(shapeIndices_);
}


void SceneInfo_impl::load(const std::string& filename)
{
    url_ = filename;
    BodyLoader loader;
    loader.setMessageSink(*os);

    BodyPtr body = loader.load(filename);
    if(!body){
        throw ModelLoader::ModelLoaderException("The model file cannot be loaded.");
        return ;
    }

    setShapeIndices(body->rootLink()->visualShape(), shapeIndices_);

}
