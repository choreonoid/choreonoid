/*
 * SubmersibleController.cpp
 *
 *  Created on: 2017/11/02
 *      Author: anazawa
 */

#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/Joystick>
#include <iostream>
#include "SubmersibleSimulatorItem2.h"
#include "ThrusterDevice.h"

using namespace std;
using namespace cnoid;

class SubmersibleController : public SimpleController
{
	Link* root;
	LightPtr light;
	SpotLightPtr spotLight;
	bool prevLightButtonState;
	Joystick joystick;
	int joystickIntervalCounter;
	DeviceList<ThrusterDevice> thrusts;
	ThrusterDevicePtr thruster;

public:
	virtual bool initialize(SimpleControllerIO* io) override
	{
		ostream& os = io->os();
		Body* body = io->body();
		root = io->body()->rootLink();

		thrusts << body->devices();

		DeviceList<Light> lights(body->devices());

		if(!lights.empty()){
			light = lights.front();

		}

		prevLightButtonState = false;

		if(!joystick.isReady()){
			os << "Joystick is not ready: " << joystick.errorMessage() << endl;

		}

		if(joystick.numAxes() < 5){
			os << "The number of the joystick axes is not sufficient for controlling the robot." << endl;

		}

		if(joystick.numButtons() < 1){
			os << "The number of the joystick buttons is not sufficient for controlling the robot." << endl;

		}

		return true;

	}

	virtual bool control() override
	{
		if(joystickIntervalCounter++ > 40){
	        joystickIntervalCounter = 0;
	        joystick.readCurrentState();
	    }

		double thrust[2];
		double thrust_updown;
	    double hyd = 0.0;

	    thrust[0]  = -joystick.getPosition(3); // right
	    thrust[1]  = -joystick.getPosition(1); // left
	    thrust_updown = joystick.getPosition(5); //up、down

	    ThrusterDevice* thrusterDevice[3];

	    for(size_t i = 0; i < thrusts.size(); ++i){
	    	thrusterDevice[i] = thrusts[i];

	    	if(thrusts[i]->id()){
	    		// Set hydraulic power.
	    		if(thrust[i - 1]){
	    			hyd = thrust[i - 1] * 0.75;

	    		}else{
	    			hyd = 0.0;

	    		}
	    	}else{
	    	   	if(thrust_updown){
	    	   		hyd = thrust_updown * 0.75;

	    	   	}else{
	    			hyd = 0.0;

	    		}
	       }
	    	thrusterDevice[i]->setHydraulic(hyd);
	    	thrusterDevice[i]->notifyStateChange();
	    }

	    if(light){
	    	bool changed = false;
	        bool lightButtonState = joystick.getButtonState(0);

	        if(lightButtonState){
	            if(!prevLightButtonState){
	                light->on(!light->on());
	                changed = true;

	            }
	        }

	        prevLightButtonState = lightButtonState;

	        if(changed){
	        	light->notifyStateChange();

	        }
	    }

		return true;

	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SubmersibleController)
