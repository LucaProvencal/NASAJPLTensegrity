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
 * @file T6Model.cpp
 * @brief Contains the implementation of class T6Model.
 * $Id$
 */

// This module
#include "T6Model.h"
// This library
#include "core/tgSpringCableActuator.h"
#include "core/tgRod.h"
#include "core/tgSphere.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <iostream>
#include <math.h>

#define PI 3.14159265359
using std::cout;
namespace
{
    // see tgBaseString.h for a descripton of some of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double stiffness_in;
        double damping;
        double damping_in;
        double rod_length;
        double rod_space;
        double payload_h;
        double density_pay;
        double radius_pay;
        double density_cyl;
        double radius_cyl;
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        double pretension_inner;
        // add pretension inner
        bool   history;
        double maxTens;
        double targetVelocity;
    } c =
   {
     4.5,    // density (kg / length^3)
     0.015875,     // radius (length)
     stiffnessouter,   // stiffness of outer muscles (kg / sec^2 = N/m = .1N/dm)
     stiffnessinner,    // stiffness of inner muscles (kg/sec^2)
     192,    // damping of outer muscles (kg / sec)
     248,     //damping of inner muscles (kg/sec)
     2.96418,     // rod_length (length) THIS IS THE ACTUAL ROD LENGTH
     .635,      // rod_space (length) THE ACTUAL SPACE IS 2X THE ENTERED VALUE

     .3302,         // half payload height (length)
     1.212205,        //payload density (kg/length^3)
     .3302,        //payload radius (length) // ACTUAL DIAMETER OF //.3302

     .40,        //payload density (kg/length^3)
     2,

     5,      // friction (unitless)
     0.1,     // rollFriction (unitless)
     0.0,       // restitution (?)
     pretensionouter,        // pretension OUTER (N)
     pretensioninner,       // pretension INNER (N)
     false,     // history
     1000000,   // maxTens
     10000    // targetVelocity
#if (0)
     20000     // maxAcc
#endif // removed 12/10/14
  };
} // namespace

T6Model::T6Model() : tgModel()
{
    //data observer
    // m_dataObserver("Data_test");
}

T6Model::~T6Model()
{
}

void T6Model::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;

    // Coordinates for parallel rod landing
    s.addNode(-c.rod_space,    -half_length, 0); // 4
    s.addNode(-c.rod_space,     half_length, 0); // 5
    s.addNode( c.rod_space,    -half_length, 0); // 6
    s.addNode( c.rod_space,     half_length, 0); // 7
    s.addNode(0,           -c.rod_space,   -half_length); // 4
    s.addNode(0,           -c.rod_space,    half_length); // 5
    s.addNode(0,            c.rod_space,   -half_length); // 6
    s.addNode(0,            c.rod_space,    half_length); // 7
    s.addNode(-half_length, 0,            c.rod_space);   // 8
    s.addNode( half_length, 0,            c.rod_space);   // 9
    s.addNode(-half_length, 0,           -c.rod_space);   // 10
    s.addNode( half_length, 0,           -c.rod_space);   // 11
           // 0

     // Coordinates for triangle landing
     // s.addNode((-5.001*.254),  (-2.021*.254), (1.237*.254));            // 0
     // s.addNode((3.572*.254),  (4.042*.254),  (1.237*.254));            // 1
     // s.addNode((-3.572*.254),  (-4.042*.254), (-1.237*.254));            // 2
     // s.addNode( (5.001*.254),   (2.021*.254), (-1.237*.254));            // 3
     //
     // s.addNode((-3.572*.254), (2.021*.254), (-3.712*.254));   // 8
     // s.addNode((.7144*.254), (-4.042*.254), (3.712*.254));   // 9
     // s.addNode((-.7144*.254), (4.042*.254), (-3.712*.254));   // 10
     // s.addNode((3.572*.254), (-2.021*.254),  (3.712*.254));   // 11
     //
     // s.addNode((-1.429*.254),  (2.021*.254),  (4.950*.254)); // 6
     // s.addNode((2.858*.254),  (-4.042*.254),  (-2.475*.254)); // 7
     // s.addNode((-2.858*.254),   (4.042*.254),  (2.475*.254)); // 4
     // s.addNode((1.429*.254), (-2.021*.254),    (-4.950*.254)); // 5


    // Node SPHERE for payload
     s.addNode(0,0,0, "payload_sphere"); // 12

    // Nodes for cylinder squishing
     // s.addNode(0,1.5,.02);
     // s.addNode(0,2.0,.02);

    // s.addNode(0,  c.payload_h, 0);  // 12 cylinder payload
    // s.addNode(0,  -c.payload_h, 0);   // 13 cylinder payload
}

void T6Model::addRods(tgStructure& s)
{
    // Struts
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod");
    s.addPair( 4,  5, "rod");
    s.addPair( 6,  7, "rod");
    s.addPair( 8,  9, "rod");
    s.addPair(10, 11, "rod");
    // s.addPair( 13,  3, "rod");  // additional rods for testing what rodspace is. this one has length=rod_space
    // s.addPair( 1,  3, "rod"); // this one has length=2*rod_space

    // // Payload
    // s.addPair(12, 13, "payload_rod"); // FOR JUST CYL

}

void T6Model::addMuscles(tgStructure& s)
{
    // Outer Cables
    s.addPair(0, 4,  "muscle");
    s.addPair(0, 5,  "muscle");
    s.addPair(0, 8,  "muscle");
    s.addPair(0, 10, "muscle");

    s.addPair(1, 6,  "muscle");
    s.addPair(1, 7,  "muscle");
    s.addPair(1, 8,  "muscle");
    s.addPair(1, 10, "muscle");

    s.addPair(2, 4,  "muscle");
    s.addPair(2, 5,  "muscle");
    s.addPair(2, 9,  "muscle");
    s.addPair(2, 11, "muscle");

    s.addPair(3, 7,  "muscle");
    s.addPair(3, 6,  "muscle");
    s.addPair(3, 9,  "muscle");
    s.addPair(3, 11, "muscle");
    //
    // s.addPair(4, 2,  "muscle"); // is this extra?
    s.addPair(4, 10, "muscle");
    s.addPair(4, 11, "muscle");

    s.addPair(5, 8,  "muscle");
    s.addPair(5, 9,  "muscle");

    s.addPair(6, 10, "muscle");
    s.addPair(6, 11, "muscle");

    s.addPair(7, 8,  "muscle");
    s.addPair(7, 9,  "muscle");
    //
    // // Payload Muscles sphere
    s.addPair(0, 12, "muscle_in");
    s.addPair(1, 12, "muscle_in");
    s.addPair(2, 12, "muscle_in");
    s.addPair(3, 12, "muscle_in");
    s.addPair(4, 12, "muscle_in");
    s.addPair(5, 12, "muscle_in");
    s.addPair(6, 12, "muscle_in");
    s.addPair(7, 12, "muscle_in");
    s.addPair(8, 12, "muscle_in");
    s.addPair(9, 12, "muscle_in");
    s.addPair(10, 12, "muscle_in");
    s.addPair(11, 12, "muscle_in");

    // Payload Muscles cylinder
    // s.addPair(0, 13, "muscle_in");
    // s.addPair(1, 12, "muscle_in");
    // s.addPair(2, 13, "muscle_in");
    // s.addPair(3, 12, "muscle_in");
    // s.addPair(4, 13, "muscle_in");
    // s.addPair(5, 12, "muscle_in");
    // s.addPair(6, 13, "muscle_in");
    // s.addPair(7, 12, "muscle_in");
    // s.addPair(8, 13, "muscle_in");
    // s.addPair(9, 12, "muscle_in");
    // s.addPair(10, 13, "muscle_in");
    // s.addPair(11, 12, "muscle_in");

}

void T6Model::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction,
				c.rollFriction, c.restitution);

    const tgRod::Config payConfig(c.radius_cyl, c.density_cyl, c.friction,
                c.rollFriction, c.restitution);

    const tgSphere::Config sphereConfig(c.radius_pay, c.density_pay, c.friction,
                c.rollFriction, c.restitution);

    /// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
    tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension * c.stiffness / c.stiffness_in, c.history,
					    c.maxTens, c.targetVelocity);

    tgSpringCableActuator::Config muscleInConfig(c.stiffness_in, c.damping_in, c.pretension_inner, c.history,
                        c.maxTens, c.targetVelocity);

    // Start creating the structure
    tgStructure s;
    // tgStructure y;
    addNodes(s);
    addRods(s);
    addMuscles(s);
    // addNodes(y);
    // addRods(y);
    // addMuscles(y);
    s.move(btVector3(0, 50.7, 0));
    // y.move(btVector3(0, 10, 0));



    // Add a rotation to land the struture on a V.
    btVector3 rotationPoint1 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis1 = btVector3(0, 0, 0);  // x-axis
    double rotationAngle1 = 0; //M_PI/2; 0.4636;
    s.addRotation(rotationPoint1, rotationAxis1, rotationAngle1);
    // Add a rotation to move structure towards triangle.
    btVector3 rotationPoint2 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis2 = btVector3(0, 0, 0);  // z-axis
    double rotationAngle2 = 0; // 1.991;
    s.addRotation(rotationPoint2, rotationAxis2, rotationAngle2);
    // Add a rotation to land the struture on a triangle.
    btVector3 rotationPoint3 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis3 = btVector3(0, 0, 0);  // x-axis
    double rotationAngle3 = 0; // 0.58895;
    s.addRotation(rotationPoint3, rotationAxis3, rotationAngle3);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("payload_rod", new tgRodInfo(payConfig));
    spec.addBuilder("payload_sphere", new tgSphereInfo(sphereConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("muscle_in", new tgBasicActuatorInfo(muscleInConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control.
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    btVector3 location(0,0,0); // DONT CHANGE THIS. USE S.MOVE() to change initial location
    btVector3 rotation(0,0,0);
  	btVector3 speed(0,0,0);  // X*, Z, Y*
    this->moveModel(location,rotation,speed);

    // Actually setup the children
    tgModel::setup(world);
}

void T6Model::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void T6Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& T6Model::getAllMuscles() const
{
    return allMuscles;
}

void T6Model::teardown()
{
    tgModel::teardown();
}

void T6Model::moveModel(btVector3 positionVector,btVector3 rotationVector,btVector3 speedVector)
{
    std::vector<tgRod *> rods=find<tgRod>("rod");
    // std:cout << "booga";
	btQuaternion initialRotationQuat;
	initialRotationQuat.setEuler(rotationVector[0],rotationVector[1],rotationVector[2]);
	btTransform initialTransform;
	initialTransform.setIdentity();
	initialTransform.setRotation(initialRotationQuat);
	initialTransform.setOrigin(positionVector);
	for(int i=0;i<rods.size();i++)
	{
			rods[i]->getPRigidBody()->setLinearVelocity(speedVector);
			rods[i]->getPRigidBody()->setWorldTransform(initialTransform * rods[i]->getPRigidBody()->getWorldTransform());
	}
}
