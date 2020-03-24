/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 *
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file AppSUPERball.cpp
 * @brief Contains the definition function main() for the Super Ball applicaiton
 * application.
 * $Id$
 */

// This application
#include "T6Model.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// Sensors
#include "sensors/tgDataLogger2.h"
#include "sensors/tgRodSensorInfo.h"
#include "sensors/tgSphereSensorInfo.h"
#include "sensors/tgSpringCableActuatorSensorInfo.h"
// The C++ Standard Library
#include <iostream>
#include <string>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */

int stiffnessouter = 1921;  // stiffness of outer muscles (N/dm)
int stiffnessinner = 2483;  // stiffness of inner muscles (N/dm)
int pretensionouter = 82.69; // pretension of outer muscles (N)
int pretensioninner = 55.45; // pretension of inner muscles (N)
int dampingouter = 192.1; //damping of outer muscles (kg/s)
int dampinginner = 248.3; //damping of inner muscles (kg/s)
int main(int argc, char** argv)
{
    std::cout << "AppSUPERball" << std::endl;

    // stiffnessbooga = 90;
    // First create the ground and world

    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    const double pitch = 0.0; //M_PI/15.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    const double titan_gravity = 98.10;
    const tgWorld::Config config(titan_gravity); // gravity, cm/sec^2  Use this to adjust length scale of world.
        // Note, by changing the setting below from 981 to 98.1, we've
        // scaled the world length scale to decimeters not cm.
    tgWorld world(config, ground);

    // Second create the view
    const double timestep_physics = 0.0001; // Seconds
    const double timestep_graphics = 1.f/240.f; // Seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    T6Model* const myModel = new T6Model();

    simulation.addModel(myModel);


    std::stringstream sstm;
    sstm << "/media/sf_Simulation_Data/datalogs_" << stiffnessouter << "_" << stiffnessinner << "_" << pretensionouter << "_" << pretensioninner << "_" << dampingouter << "_" << dampinginner;
    std::string log_filename = sstm.str();


    double samplingTimeInterval = 0.001;
    tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, samplingTimeInterval);

    myDataLogger->addSenseable(myModel);

    tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    tgSphereSensorInfo* mySphereSensorInfo = new tgSphereSensorInfo();
    tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();
    myDataLogger->addSensorInfo(myRodSensorInfo);
    myDataLogger->addSensorInfo(mySCASensorInfo);
    myDataLogger->addSensorInfo(mySphereSensorInfo);
    simulation.addDataManager(myDataLogger);

    // Run until the user stops
    simulation.run();

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
