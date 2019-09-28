/**
   @author Shizuko Hattori
   This is implemented based on ItemPropertyView.cpp
*/

#include "SceneGraphPropertyView.h"
#include "SceneGraphView.h"
#include <cnoid/SelectionListEditor>
#include <cnoid/EigenUtil>
#include <cnoid/AppConfig>
#include <cnoid/ViewManager>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneLights>
#include <cnoid/SceneCameras>
#include <cnoid/SceneEffects>
#include <cnoid/stdx/variant>
#include <QBoxLayout>
#include <QTableWidget>
#include <QHeaderView>
#include <QDoubleSpinBox>
#include <QStyledItemDelegate>
#include <QItemEditorFactory>
#include <QStandardItemEditorCreator>
#include <QKeyEvent>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

struct Int {
    int value;
    int min, max;
    Int(int v, int min=std::numeric_limits<int>::min(), int max= std::numeric_limits<int>::max()) {
        value = v;
        this->min = min;
        this->max = max;
    };
};

struct Double {
    double value;
    int decimals;
    double min, max;
    Double(double v, int d=2, double min= -std::numeric_limits<double>::max(), double max=std::numeric_limits<double>::max()) {
        value = v;
        decimals = d;
        this->min = min;
        this->max = max;
    };
};

typedef stdx::variant<bool, Int, Double, string> ValueVariant;

typedef stdx::variant<std::function<bool(bool)>, std::function<bool(int)>, std::function<bool(double)>,
                      std::function<bool(const string&)> > FunctionVariant;

enum TypeId { TYPE_BOOL, TYPE_INT, TYPE_DOUBLE, TYPE_STRING };

class PropertyItem : public QTableWidgetItem
{
public:
    PropertyItem(SceneGraphPropertyViewImpl* viewImpl, ValueVariant value);
    PropertyItem(SceneGraphPropertyViewImpl* viewImpl, ValueVariant value, FunctionVariant func);

    virtual QVariant data(int role) const;
    virtual void setData(int role, const QVariant& qvalue);

    SceneGraphPropertyViewImpl* sceneGraphPropertyViewImpl;
    ValueVariant value;
    FunctionVariant func;
};

class CustomizedTableWidget : public QTableWidget
{
public:
    CustomizedTableWidget(QWidget* parent) : QTableWidget(parent) { }
        
    PropertyItem* itemFromIndex(const QModelIndex& index) const {
        return dynamic_cast<PropertyItem*>(QTableWidget::itemFromIndex(index));
    }
};

class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    CustomizedTableWidget* tableWidget;
    int decimals;
        
    CustomizedItemDelegate(CustomizedTableWidget* tableWidget)
        : tableWidget(tableWidget) { decimals = 2; }
        
    virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const {

        QWidget* editor = QStyledItemDelegate::createEditor(parent, option, index);
        PropertyItem* item = tableWidget->itemFromIndex(index);
        if(item){
            if(QSpinBox* spinBox = dynamic_cast<QSpinBox*>(editor)){
                ValueVariant& value = item->value;
                if(stdx::get_variant_index(value) == TYPE_INT){
                    Int& v = stdx::get<Int>(value);
                    spinBox->setRange(v.min, v.max);
                }
            } else if(QDoubleSpinBox* doubleSpinBox = dynamic_cast<QDoubleSpinBox*>(editor)){
                ValueVariant& value = item->value;
                if(stdx::get_variant_index(value) == TYPE_DOUBLE){
                    Double& v = stdx::get<Double>(value);
                    if(v.decimals >= 0){
                        doubleSpinBox->setDecimals(v.decimals);
                        doubleSpinBox->setSingleStep(pow(10.0, -v.decimals));
                    }
                    doubleSpinBox->setRange(v.min, v.max);
                }
            }
        }
        return editor;
    }

    virtual QString displayText(const QVariant& value, const QLocale& locale) const {
        if(value.type() == QVariant::Double){
            return QString::number(value.toDouble(), 'f', decimals);
        }
        return QStyledItemDelegate::displayText(value, locale);
    }

    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const {
            
        PropertyItem* item = tableWidget->itemFromIndex(index);
        if(item && (stdx::get_variant_index(item->value) == TYPE_DOUBLE)){
            int& d = const_cast<int&>(decimals);
            d = stdx::get<Double>(item->value).decimals;
        }
        QStyledItemDelegate::paint(painter, option, index);
    }

};
}


namespace cnoid {

class SceneGraphPropertyViewImpl 
{
public:
    SceneGraphPropertyViewImpl(SceneGraphPropertyView* self);
    ~SceneGraphPropertyViewImpl();

    SceneGraphPropertyView* self;
    CustomizedTableWidget* tableWidget;
    int fontPointSizeDiff;
    SgObject* currentObject;

    Connection selectionChangedConnection;
    Connection connectionOfsceneUpdated;

    void setProperty(SgObject* obj);
    bool setName(const string& name);
    void setProperty(SgNode* node);
    void setProperty(SgGroup* group);
    void setProperty(SgInvariantGroup* invGroup);
    void setProperty(SgTransform* trans);
    void setProperty(SgPosTransform* posTrans);
    void setProperty(SgScaleTransform* scaleTrans);
    void setProperty(SgMaterial* material);
    void setProperty(SgTextureTransform* textureTrans);
    void setProperty(SgTexture* texture);
    void setProperty(SgMeshBase* meshBase);
    void setProperty(SgMesh* mesh);
    void setProperty(SgPolygonMesh* pMesh);
    void setProperty(SgShape* shape);
    void setProperty(SgPlot* plot);
    void setProperty(SgPointSet* pointSet);
    void setProperty(SgLineSet* lineSet);
    void setProperty(SgPreprocessed* preprocessed);
    void setProperty(SgLight* light);
    void setProperty(SgDirectionalLight* dlight);
    void setProperty(SgPointLight* plight);
    void setProperty(SgSpotLight* slight);
    void setProperty(SgCamera* camera);
    void setProperty(SgPerspectiveCamera* pcamera);
    void setProperty(SgOrthographicCamera* ocamera);
    void setProperty(SgFog* fog);

    void addProperty(const std::string& name, PropertyItem* propertyItem);
    void updateProperties();
    void onActivated(bool on);
    void onItemSelectionChanged(const SgObject* object);
    void onSceneGraphUpdated(const SgUpdate& update);
    void zoomFontSize(int pointSizeDiff);
};
}


PropertyItem::PropertyItem(SceneGraphPropertyViewImpl* viewImpl, ValueVariant value)
    : sceneGraphPropertyViewImpl(viewImpl),
      value(value)
{
    setFlags(Qt::ItemIsEnabled);
}


PropertyItem::PropertyItem(SceneGraphPropertyViewImpl* viewImpl, ValueVariant value, FunctionVariant func)
    : sceneGraphPropertyViewImpl(viewImpl),
      value(value),
      func(func)
{
    setFlags(Qt::ItemIsEnabled|Qt::ItemIsEditable);
}


QVariant PropertyItem::data(int role) const
{
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        switch(stdx::get_variant_index(value)){
        case TYPE_BOOL:      return stdx::get<bool>(value);
        case TYPE_INT:       return stdx::get<Int>(value).value;
        case TYPE_DOUBLE:    return stdx::get<Double>(value).value;
        case TYPE_STRING:    return stdx::get<string>(value).c_str();
        }
    }
    return QTableWidgetItem::data(role);
}


void PropertyItem::setData(int role, const QVariant& qvalue)
{
    bool accepted = false;
    if(role == Qt::EditRole){
        switch(qvalue.type()){
                
        case QVariant::Bool:
            accepted = stdx::get<std::function<bool(bool)>>(func)(qvalue.toBool());
            break;
                
        case QVariant::String:
            accepted = stdx::get<std::function<bool(const string&)>>(func)(qvalue.toString().toStdString());
            break;
                
        case QVariant::Int:
            accepted = stdx::get<std::function<bool(int)>>(func)(qvalue.toInt());
            break;
                
        case QVariant::Double:
            accepted = stdx::get<std::function<bool(double)>>(func)(qvalue.toDouble());
            break;
        default:
            break;
        }
    }
}


void SceneGraphPropertyView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<SceneGraphPropertyView>(
        "SceneGraphPropertyView", N_("Scene Graph Property"), ViewManager::SINGLE_OPTIONAL);
}


SceneGraphPropertyView::SceneGraphPropertyView()
{
    setDefaultLayoutArea(View::LEFT_BOTTOM);
    
    impl = new SceneGraphPropertyViewImpl(this);
}


SceneGraphPropertyView::~SceneGraphPropertyView()
{
    delete impl;
}


SceneGraphPropertyViewImpl::SceneGraphPropertyViewImpl(SceneGraphPropertyView* self)
    : self(self)
{
    tableWidget = new CustomizedTableWidget(self);
    tableWidget->setFrameShape(QFrame::NoFrame);
    tableWidget->setColumnCount(2);
    tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableWidget->setSelectionMode(QAbstractItemView::NoSelection);

    tableWidget->horizontalHeader()->hide();
    tableWidget->horizontalHeader()->setStretchLastSection(true);

    tableWidget->verticalHeader()->hide();
    tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

    QStyledItemDelegate* delegate = new CustomizedItemDelegate(tableWidget);
    QItemEditorFactory* factory = new QItemEditorFactory;
    
    QItemEditorCreatorBase* selectionListCreator =
        new QStandardItemEditorCreator<SelectionListEditor>();
    factory->registerEditor(QVariant::StringList, selectionListCreator);
    
    delegate->setItemEditorFactory(factory);

    tableWidget->setItemDelegate(delegate);

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(tableWidget);
    self->setLayout(layout);
    
    self->sigActivated().connect([&](){ onActivated(true); });
    self->sigDeactivated().connect([&](){ onActivated(false); });

    currentObject = 0;

    fontPointSizeDiff = 0;
    MappingPtr config = AppConfig::archive()->openMapping("SceneGraphPropertyView");
    int storedFontPointSizeDiff;
    if(config->read("fontZoom", storedFontPointSizeDiff)){
        zoomFontSize(storedFontPointSizeDiff);
    }
}


SceneGraphPropertyViewImpl::~SceneGraphPropertyViewImpl()
{
    selectionChangedConnection.disconnect();
    connectionOfsceneUpdated.disconnect();
}


void SceneGraphPropertyViewImpl::setProperty(SgObject* obj)
{
//    std::function<bool(const string&)> f = std::bind(&SceneGraphPropertyViewImpl::setName, this, _1);
//    addProperty("name", new PropertyItem(this, obj->name(), (FunctionVariant)f));
    addProperty(_("name"), new PropertyItem(this, obj->name()));
}


bool SceneGraphPropertyViewImpl::setName(const string& name)
{
    currentObject->setName(name);
//    currentObject->notifyUpdate();
    return true;
}


void SceneGraphPropertyViewImpl::setProperty(SgNode* node)
{

}


void SceneGraphPropertyViewImpl::setProperty(SgGroup* group)
{
    addProperty(_("num of children"), new PropertyItem(this, Int(group->numChildren())));
}


void SceneGraphPropertyViewImpl::setProperty(SgInvariantGroup* invGroup)
{

}


void SceneGraphPropertyViewImpl::setProperty(SgTransform* trans)
{

}


void SceneGraphPropertyViewImpl::setProperty(SgPosTransform* posTrans)
{
    addProperty(_("translation"), new PropertyItem(this, str(Vector3(posTrans->translation()))));
    AngleAxis angleAxis(posTrans->rotation());
    addProperty(_("rotation"), new PropertyItem(this, str(angleAxis)));
}


void SceneGraphPropertyViewImpl::setProperty(SgScaleTransform* scaleTrans)
{
    addProperty(_("scale"), new PropertyItem(this, str(scaleTrans->scale())));
}


void SceneGraphPropertyViewImpl::setProperty(SgMaterial* material)
{
    addProperty(_("ambientIntensity"), new PropertyItem(this, Double(material->ambientIntensity(), 1, 0.0, 1.0)));
    addProperty(_("diffuseColor"), new PropertyItem(this,str(material->diffuseColor())));
    addProperty(_("emissiveColor"), new PropertyItem(this,str(material->emissiveColor())));
    addProperty(_("shininess"), new PropertyItem(this, Double(material->shininess(), 1, 0.0, 1.0)));
    addProperty(_("specularColor"), new PropertyItem(this,str(material->specularColor())));
    addProperty(_("transparency"), new PropertyItem(this, Double(material->transparency(), 1, 0.0, 1.0)));
}


void SceneGraphPropertyViewImpl::setProperty(SgTextureTransform* textureTrans)
{
    addProperty(_("center"), new PropertyItem(this, str(textureTrans->center())));
    addProperty(_("rotation"), new PropertyItem(this, Double(textureTrans->rotation(), 3)));
    addProperty(_("scale"), new PropertyItem(this, str(textureTrans->scale())));
    addProperty(_("translation"), new PropertyItem(this, str(textureTrans->translation())));
}


void SceneGraphPropertyViewImpl::setProperty(SgTexture* texture)
{
    SgImage* image = texture->image();
    if(image){
        addProperty(_("width"), new PropertyItem(this, Int(image->width())));
        addProperty(_("height"), new PropertyItem(this, Int(image->height())));
        addProperty(_("num of components"), new PropertyItem(this, Int(image->numComponents())));
    }
    addProperty(_("repeatS"), new PropertyItem(this, texture->repeatS()));
    addProperty(_("repeatT"), new PropertyItem(this, texture->repeatT()));
}


void SceneGraphPropertyViewImpl::setProperty(SgMeshBase* meshBase)
{
    if(meshBase->hasVertices())
        addProperty(_("num of vertices"), new PropertyItem(this, Int(meshBase->vertices()->size())));
    if(meshBase->hasNormals())
        addProperty(_("num of normals"), new PropertyItem(this, Int(meshBase->normals()->size())));
    if(meshBase->hasColors())
        addProperty(_("num of colors"), new PropertyItem(this, Int(meshBase->colors()->size())));
    if(meshBase->hasTexCoords())
        addProperty(_("num of texCoord"), new PropertyItem(this, Int(meshBase->texCoords()->size())));
    addProperty(_("num of normalIndices"), new PropertyItem(this, Int(meshBase->normalIndices().size())));
    addProperty(_("num of colorIndices"), new PropertyItem(this, Int(meshBase->colorIndices().size())));
    addProperty(_("num of texCoordIndices"), new PropertyItem(this, Int(meshBase->texCoordIndices().size())));
    addProperty(_("is Solid"), new PropertyItem(this, meshBase->isSolid()));
}


void SceneGraphPropertyViewImpl::setProperty(SgMesh* mesh)
{
    addProperty(_("num of triangles"), new PropertyItem(this, Int(mesh->numTriangles())));
    switch(mesh->primitiveType())
        {
        case SgMesh::BOX : {
            addProperty(_("primitive type"), new PropertyItem(this, string("Box")));
            const Vector3& size = mesh->primitive<SgMesh::Box>().size;
            addProperty(_("size"), new PropertyItem(this,str(size)));
            break; }
        case SgMesh::SPHERE : {
            double radius = mesh->primitive<SgMesh::Sphere>().radius;
            addProperty(_("primitive type"), new PropertyItem(this, string("Sphere")));
            addProperty(_("radius"), new PropertyItem(this,Double(radius,3)));
            break; }
        case SgMesh::CYLINDER : {
            addProperty(_("primitive type"), new PropertyItem(this, string("Cylinder")));
            SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
            addProperty(_("radius"), new PropertyItem(this,Double(cylinder.radius,3)));
            addProperty(_("height"), new PropertyItem(this,Double(cylinder.height,3)));
            addProperty(_("bottom"), new PropertyItem(this,cylinder.bottom));
            addProperty(_("side"), new PropertyItem(this,cylinder.side));
            addProperty(_("top"), new PropertyItem(this,cylinder.top));
            break; }
        case SgMesh::CONE :{
            addProperty(_("primitive type"), new PropertyItem(this, string("Cone")));
            SgMesh::Cone cone = mesh->primitive<SgMesh::Cone>();
            addProperty(_("radius"), new PropertyItem(this,Double(cone.radius,3)));
            addProperty(_("height"), new PropertyItem(this,Double(cone.height,3)));
            addProperty(_("bottom"), new PropertyItem(this,cone.bottom));
            addProperty(_("side"), new PropertyItem(this,cone.side));
            break; }
        case SgMesh::CAPSULE :{
            addProperty(_("primitive type"), new PropertyItem(this, string("Capsule")));
            SgMesh::Capsule capsule = mesh->primitive<SgMesh::Capsule>();
            addProperty(_("radius"), new PropertyItem(this,Double(capsule.radius,3)));
            addProperty(_("height"), new PropertyItem(this,Double(capsule.height,3)));
            break; }
        }
}


void SceneGraphPropertyViewImpl::setProperty(SgPolygonMesh* pMesh)
{
    addProperty(_("num of polygonVertices"), new PropertyItem(this, Int(pMesh->polygonVertices().size())));
}


void SceneGraphPropertyViewImpl::setProperty(SgShape* shape)
{

}


void SceneGraphPropertyViewImpl::setProperty(SgPlot* plot)
{
    if(plot->hasVertices())
        addProperty(_("num of vertices"), new PropertyItem(this, Int(plot->vertices()->size())));
    if(plot->hasNormals())
        addProperty(_("num of normals"), new PropertyItem(this, Int(plot->normals()->size())));
    if(plot->hasColors())
        addProperty(_("num of colors"), new PropertyItem(this, Int(plot->colors()->size())));
}


void SceneGraphPropertyViewImpl::setProperty(SgPointSet* pointSet)
{
    addProperty(_("point size"), new PropertyItem(this, Double(pointSet->pointSize())));
}


void SceneGraphPropertyViewImpl::setProperty(SgLineSet* lineSet)
{
    addProperty(_("line width"), new PropertyItem(this, Double(lineSet->lineWidth())));
    addProperty(_("num of colorIndices"), new PropertyItem(this, Int(lineSet->colorIndices().size())));
}


void SceneGraphPropertyViewImpl::setProperty(SgPreprocessed* preprocessed)
{

}


void SceneGraphPropertyViewImpl::setProperty(SgLight* light)
{
    addProperty(_("on/off"), new PropertyItem(this, light->on()? string("ON") : string("OFF") ));
    addProperty(_("color"), new PropertyItem(this,str(light->color())));
    addProperty(_("intensity"), new PropertyItem(this,Double(light->intensity(), 2, 0.0, 1.0)));
    addProperty(_("ambientIntensity"), new PropertyItem(this,Double(light->ambientIntensity(), 2, 0.0, 1.0)));
}


void SceneGraphPropertyViewImpl::setProperty(SgDirectionalLight* dlight)
{
    addProperty(_("direction"), new PropertyItem(this,str(dlight->direction())));
}


void SceneGraphPropertyViewImpl::setProperty(SgPointLight* plight)
{
    addProperty(_("constant attenuation"), new PropertyItem(this, Double(plight->constantAttenuation(), 2, 0.0)));
    addProperty(_("linear attenuation"), new PropertyItem(this, Double(plight->linearAttenuation(), 2, 0.0)));
    addProperty(_("quadratic attenuation"), new PropertyItem(this, Double(plight->quadraticAttenuation(), 2, 0.0)));
}


void SceneGraphPropertyViewImpl::setProperty(SgSpotLight* slight)
{
    addProperty(_("direction"), new PropertyItem(this,str(slight->direction())));
    addProperty(_("beam width"), new PropertyItem(this,Double(slight->beamWidth(), 3)));
    addProperty(_("cut off Angle"), new PropertyItem(this,Double(slight->cutOffAngle(), 3)));
}


void SceneGraphPropertyViewImpl::setProperty(SgCamera* camera)
{
    addProperty(_("near distance"), new PropertyItem(this,Double(camera->nearClipDistance(), 3)));
    addProperty(_("far distance"), new PropertyItem(this,Double(camera->farClipDistance(), 3)));
}


void SceneGraphPropertyViewImpl::setProperty(SgPerspectiveCamera* pcamera)
{
    addProperty(_("field of view"), new PropertyItem(this,Double(pcamera->fieldOfView(), 3)));
}


void SceneGraphPropertyViewImpl::setProperty(SgOrthographicCamera* ocamera)
{
    addProperty(_("height"), new PropertyItem(this,Double(ocamera->height(), 3)));
}


void SceneGraphPropertyViewImpl::setProperty(SgFog* fog)
{

}


void SceneGraphPropertyViewImpl::updateProperties()
{
    tableWidget->clear();
    tableWidget->setRowCount(0);
    if(currentObject){
        setProperty(currentObject);
        SgNode* node = dynamic_cast<SgNode*>(currentObject);
        if(node){
            SgGroup* group = dynamic_cast<SgGroup*>(node);
            if(group){
                setProperty(group);
                SgInvariantGroup* invGroup = dynamic_cast<SgInvariantGroup*>(group);
                if(invGroup){
                    setProperty(invGroup);
                    return;
                }
                SgTransform* trans = dynamic_cast<SgTransform*>(group);
                if(trans){
                    setProperty(trans);
                    SgPosTransform* posTrans = dynamic_cast<SgPosTransform*>(trans);
                    if(posTrans){
                        setProperty(posTrans);
                        return;
                    }
                    SgScaleTransform* scaleTrans = dynamic_cast<SgScaleTransform*>(trans);
                    if(scaleTrans){
                        setProperty(scaleTrans);
                        return;
                    }
                    return;
                }
                return;
            }
            SgShape* shape = dynamic_cast<SgShape*>(node);
            if(shape){
                setProperty(shape);
                return;
            }
            SgPlot* plot = dynamic_cast<SgPlot*>(node);
            if(plot){
                setProperty(plot);
                SgPointSet* pointSet = dynamic_cast<SgPointSet*>(plot);
                if(pointSet){
                    setProperty(pointSet);
                    return;
                }
                SgLineSet* lineSet = dynamic_cast<SgLineSet*>(plot);
                if(lineSet){
                    setProperty(lineSet);
                    return;
                }
                return;
            }
            SgPreprocessed* preprocessed = dynamic_cast<SgPreprocessed*>(node);
            if(preprocessed){
                setProperty(preprocessed);
                SgLight* light = dynamic_cast<SgLight*>(preprocessed);
                if(light){
                    setProperty(light);
                    SgDirectionalLight* dlight = dynamic_cast<SgDirectionalLight*>(light);
                    if(dlight){
                        setProperty(dlight);
                        return;
                    }
                    SgPointLight* plight = dynamic_cast<SgPointLight*>(light);
                    if(plight){
                        setProperty(plight);
                        return;
                    }
                    SgSpotLight* slight = dynamic_cast<SgSpotLight*>(light);
                    if(slight){
                        setProperty(slight);
                        return;
                    }
                    return;
                }
                SgCamera* camera = dynamic_cast<SgCamera*>(preprocessed);
                if(camera){
                    setProperty(camera);
                    SgPerspectiveCamera* pcamera = dynamic_cast<SgPerspectiveCamera*>(camera);
                    if(pcamera){
                        setProperty(pcamera);
                        return;
                    }
                    SgOrthographicCamera* ocamera = dynamic_cast<SgOrthographicCamera*>(camera);
                    if(ocamera){
                        setProperty(ocamera);
                        return;
                    }
                    return;
                }
                SgFog* fog = dynamic_cast<SgFog*>(preprocessed);
                if(fog){
                    setProperty(fog);
                    return;
                }
                return;
            }
            return;
        }
        SgMaterial* material = dynamic_cast<SgMaterial*>(currentObject);
        if(material){
            setProperty(material);
            return;
        }
        SgTextureTransform* textureTrans = dynamic_cast<SgTextureTransform*>(currentObject);
        if(textureTrans){
            setProperty(textureTrans);
            return;
        }
        SgTexture* texture = dynamic_cast<SgTexture*>(currentObject);
        if(texture){
            setProperty(texture);
            return;
        }
        SgMeshBase* meshBase = dynamic_cast<SgMeshBase*>(currentObject);
        if(meshBase){
            setProperty(meshBase);
            SgMesh* mesh = dynamic_cast<SgMesh*>(meshBase);
            if(mesh){
                setProperty(mesh);
                return;
            }
            SgPolygonMesh* polygonMesh = dynamic_cast<SgPolygonMesh*>(meshBase);
            if(polygonMesh){
                setProperty(polygonMesh);
                return;
            }
            return;
        }
        return;
    }
}


void SceneGraphPropertyViewImpl::addProperty(const std::string& name, PropertyItem* propertyItem)
{
    int row = tableWidget->rowCount();
    tableWidget->setRowCount(row + 1);

    QTableWidgetItem* nameItem = new QTableWidgetItem(name.c_str());
    nameItem->setFlags(Qt::ItemIsEnabled);
    tableWidget->setItem(row, 0, nameItem);
    
    tableWidget->setItem(row, 1, propertyItem);
}


void SceneGraphPropertyViewImpl::onActivated(bool on)
{
    selectionChangedConnection.disconnect();
    
    if(on){
        SceneGraphView* sceneGraphView = ViewManager::findView<SceneGraphView>();
        if(sceneGraphView){
            onItemSelectionChanged(sceneGraphView->selectedObject());
            selectionChangedConnection = sceneGraphView->sigSelectionChanged().connect(
                [&](const SgObject* object){ onItemSelectionChanged(object); });
        }
    } else {
        connectionOfsceneUpdated.disconnect();
        currentObject = 0;
    }
}


void SceneGraphPropertyViewImpl::onItemSelectionChanged(const SgObject* object)
{
    if(!object)
        return;

    if(object != currentObject){
        connectionOfsceneUpdated.disconnect();
        currentObject = (SgObject*)object;
        if(currentObject)
            connectionOfsceneUpdated =
                currentObject->sigUpdated().connect(
                    [&](const SgUpdate& update){ onSceneGraphUpdated(update); });
        updateProperties();
    }
}


void SceneGraphPropertyViewImpl::onSceneGraphUpdated(const SgUpdate& update)
{
    if(update.action() & (SgUpdate::MODIFIED))
        updateProperties();
}


void SceneGraphPropertyView::keyPressEvent(QKeyEvent* event)
{
    if(event->modifiers() & Qt::ControlModifier){
        switch(event->key()){
        case Qt::Key_Plus:
        case Qt::Key_Semicolon:
            impl->zoomFontSize(1);
            return;
        case Qt::Key_Minus:
            impl->zoomFontSize(-1);
            return;
        defaut:
            break;
        }
    }
    View::keyPressEvent(event);
}


void SceneGraphPropertyViewImpl::zoomFontSize(int pointSizeDiff)
{
    QFont font = tableWidget->font();
    font.setPointSize(font.pointSize() + pointSizeDiff);
    tableWidget->setFont(font);
    fontPointSizeDiff += pointSizeDiff;
    AppConfig::archive()->openMapping("SceneGraphPropertyView")->write("fontZoom", fontPointSizeDiff);
}
