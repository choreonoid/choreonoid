/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include <iostream>

#include <QtCore/QtCore>

#include <boost/property_tree/ini_parser.hpp>

#include <cnoid/LazyCaller>
#include <cnoid/MeshExtractor>
#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <cnoid/ViewManager>
#include <cnoid/ItemManager>
#include <cnoid/BodyBar>
#include <cnoid/LinkSelectionView>
#include <cnoid/RootItem>
#include <cnoid/SimulatorItem>
#include <cnoid/SubSimulatorItem>
#include <cnoid/MeshGenerator>
#include <cnoid/SceneView>
#include <cnoid/AISTCollisionDetector>
#include <cnoid/YAMLBodyLoader>
#include <cnoid/EigenArchive>
#include <cnoid/Tokenizer>

#include "gettext.h"
#include "exportdecl.h"

#include "GlobalImpl.h"

#include "SmartPointer.h"
#include "Array.h"

#include "Triangle3.h"
#include "Box.h"

#include "MulticopterPluginConfig.h"

#include "FFCalc_Common.h"
#include "FFCalc_GaussQuadratureTriangle.h"
#include "FFCalc_GaussTriangle3d.h"
#include "FFCalc_INormalizedFunction.h"
#include "FFCalc_CutoffCoef.h"
#include "FFCalc_CutoffCoefImpl.h"

#include "LinkAttribute.h"
#include "LinkTriangleAttribute.h"
#include "FluidEnvironment.h"

#include "FFCalc_LinkForce.h"
#include "FFCalc_LinkState.h"
#include "FFCalc_FFCalculator.h"
#include "FFCalc_calcFluidForce.h"

#include "RotorDevice.h"

#include "EventManager.h"
#include "LinkManager.h"
#include "SimulationManager.h"

#include "UtilityImpl.h"

#include "MonitorForm.h"
#include "MonitorView.h"
#include "MulticopterMonitorView.h"

#include "MulticopterSimulatorItem.h"
