/*
* Copyright (2006 as c)-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions: dynamic 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

part of rockdot_physics;

/**
 * Friction joint defintion
 * @see b2FrictionJoint
 */
class b2FrictionJointDef extends b2JointDef {
  b2FrictionJointDef() {
    type = b2Joint.e_frictionJoint;
    maxForce = 0.0;
    maxTorque = 0.0;
  }

  /**
	 * Initialize the bodies, anchors, axis, and reference angle using the world
	 * anchor and world axis.
	 */
  void Initialize(b2Body bA, b2Body bB, b2Vec2 anchor) {
    bodyA = bA;
    bodyB = bB;
    localAnchorA.SetV(bodyA.GetLocalPoint(anchor));
    localAnchorB.SetV(bodyB.GetLocalPoint(anchor));
  }

  /**
	* The local anchor point relative to bodyA's origin.
	*/
  b2Vec2 localAnchorA = new b2Vec2();

  /**
	* The local anchor point relative to bodyB's origin.
	*/
  b2Vec2 localAnchorB = new b2Vec2();

  /**
	 * The maximun force in N.
	 */
  double maxForce = 0.0;

  /**
	 * The maximun friction torque in N-m
	 */
  double maxTorque = 0.0;
}
