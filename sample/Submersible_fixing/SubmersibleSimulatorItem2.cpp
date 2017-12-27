/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SubmersibleSimulatorItem2.h"

#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Body>
#include <cnoid/Light>
#include <cnoid/Joystick>
#include <vector>
#include <cnoid/SceneGraph>
#include <cnoid/BoundingBox>
#include <cnoid/Link>
#include "ThrusterDevice.h"
#include <iostream>

using namespace std;
using namespace cnoid;
using boost::format;

namespace {

vector<Vector3, Eigen::aligned_allocator<Vector3> > resistancePoints;
vector<Vector3, Eigen::aligned_allocator<Vector3> > thrustPoints;

std::shared_ptr<Joystick> joystick;
vector<SimulationBodyPtr> allSimBodies;
vector<SimulationBody*> simBodies;

bool flag;
int num;

}

void SubmersibleSimulatorItem2::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<SubmersibleSimulatorItem2>("SubmersibleSimulatorItem2");
    im.addCreationPanel<SubmersibleSimulatorItem2>();

}


SubmersibleSimulatorItem2::SubmersibleSimulatorItem2()
{
    initialize();
}


SubmersibleSimulatorItem2::SubmersibleSimulatorItem2(const SubmersibleSimulatorItem2& org)
    : SubSimulatorItem(org)
{
    initialize();
}


void SubmersibleSimulatorItem2::initialize()
{
    simulatorItem = 0;
    flag = true;

}


SubmersibleSimulatorItem2::~SubmersibleSimulatorItem2()
{

}


Item* SubmersibleSimulatorItem2::doDuplicate() const
{
    return new SubmersibleSimulatorItem2(*this);
}

bool SubmersibleSimulatorItem2::initializeSimulation(SimulatorItem* simulatorItem)
{
	simBodies.clear();

    vector<SimulationBody*> simBody = simulatorItem->simulationBodies();
    for(int i = 0; i < (int)simBody.size(); ++i){
    	simBodies.push_back(simBody[i]);

    }

    body.resize(simBody.size());

    if(simBody.size()){
    	for(int i = 0; i < (int)simBody.size(); ++i){
    		body[i] = simBody[i]->body();

    	}

        MessageView::instance()->putln("A submersible model has been detected.");
        simulatorItem->addPreDynamicsFunction(
            std::bind(&SubmersibleSimulatorItem2::applyResistanceForce, this));
        joystick.reset(new Joystick());
        joystickIntervalCounter = 0;
    }
    
    return true;
}


void SubmersibleSimulatorItem2::applyResistanceForce()
{
	for(int j = 0; j < (int)simBodies.size(); ++j){
		root = body[j]->rootLink();
		SgNode* collisionShape = root->collisionShape();

		DeviceList<ThrusterDevice> thruster(body[j]->devices());
		double thrust_pos[3];

		if(thruster.size()){
			if(collisionShape && flag){
				max_bbox[0] = collisionShape->boundingBox().max().x();
				max_bbox[1] = collisionShape->boundingBox().max().y();
				max_bbox[2] = collisionShape->boundingBox().max().z();
				min_bbox[0] = collisionShape->boundingBox().min().x();
				min_bbox[1] = collisionShape->boundingBox().min().y();
				min_bbox[2] = collisionShape->boundingBox().min().z();

				resistancePoints.push_back(Vector3( max_bbox[0],  max_bbox[1],  max_bbox[2]));
				resistancePoints.push_back(Vector3( max_bbox[0],  min_bbox[1],  max_bbox[2]));
				resistancePoints.push_back(Vector3( max_bbox[0],  min_bbox[1],  min_bbox[2]));
				resistancePoints.push_back(Vector3( max_bbox[0],  max_bbox[1],  min_bbox[2]));
				resistancePoints.push_back(Vector3( min_bbox[0],  max_bbox[1],  max_bbox[2]));
				resistancePoints.push_back(Vector3( min_bbox[0],  min_bbox[1],  max_bbox[2]));
				resistancePoints.push_back(Vector3( min_bbox[0],  min_bbox[1],  min_bbox[2]));
				resistancePoints.push_back(Vector3( min_bbox[0],  max_bbox[1],  min_bbox[2]));

				for(size_t k = 0; k < thruster.size(); ++k){
					thrust_pos[0] = thruster[k]->T_local().translation().x();
					thrust_pos[1] = thruster[k]->T_local().translation().y();
					thrust_pos[2] = thruster[k]->T_local().translation().z();
					thrustPoints.push_back(Vector3(thrust_pos[2], thrust_pos[1], thrust_pos[0]));

				}

				num = j;
				flag = false;

			}

			root = body[num]->rootLink();
			// buoyancy
			Vector3 b(0, 0, body[num]->mass() * 9.80665);
			root->f_ext() += b;
			Vector3 cb = root->T() * Vector3(0.0, 0.0, 0.05);
			root->tau_ext() += cb.cross(b);

			for(size_t i=0; i < resistancePoints.size(); ++i){
				Vector3 a = root->R() * resistancePoints[i];
				Vector3 v = root->v() + root->w().cross(a);
				Vector3 f = -2.0 * v;
				double l = f.norm();
				if(l > 100.0){
					f /= l;
				}
				root->f_ext() += f;
				Vector3 p = a + root->p();
				root->tau_ext() += p.cross(f);
			}

			for(int i=0; i < 2; ++i){
				Vector3 f = root->R() * Vector3(15.0 * thruster[i+1]->hydraulic(), 0.0, 0.0);
				Vector3 p = root->T() * thrustPoints[i+1];
				root->f_ext() += f;
				root->tau_ext() += p.cross(f);
			}

			Vector3 fz(0.0, 0.0, -5.0 * thruster[0]->hydraulic());
			root->f_ext() += fz;
			Vector3 cz = root->T() * Vector3(0.0, 0.0, -1.5);
			root->tau_ext() += cz.cross(fz);

		}
	}
}
        

void SubmersibleSimulatorItem2::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);
}


bool SubmersibleSimulatorItem2::store(Archive& archive)
{
    SubSimulatorItem::store(archive);
    return true;
}


bool SubmersibleSimulatorItem2::restore(const Archive& archive)
{
    SubSimulatorItem::restore(archive);
    return true;
}
