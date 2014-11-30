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

 part of stagexl_box2d;
	



	

/**
* Prismatic joint definition. This requires defining a line of
* motion using an axis and an anchor point. The definition uses local
* anchor points and a local axis so that the initial configuration
* can violate the constraint slightly. The joint translation is zero
* when the local anchor points coincide in world space. Using local
* anchors and a local axis helps when saving and loading a game.
* @see b2PrismaticJoint
*/
 class b2PrismaticJointDef extends b2JointDef
{
	 b2PrismaticJointDef()
	{
		type = b2Joint.e_prismaticJoint;
		//localAnchor1.SetZero();
		//localAnchor2.SetZero();
		localAxisA.Set(1.0, 0.0);
		referenceAngle = 0.0;
		enableLimit = false;
		lowerTranslation = 0.0;
		upperTranslation = 0.0;
		enableMotor = false;
		maxMotorForce = 0.0;
		motorSpeed = 0.0;
	}
	
	  void Initialize(b2Body bA,b2Body bB,b2Vec2 anchor,b2Vec2 axis)
	{
		bodyA = bA;
		bodyB = bB;
		localAnchorA = bodyA.GetLocalPoint(anchor);
		localAnchorB = bodyB.GetLocalPoint(anchor);
		localAxisA = bodyA.GetLocalVector(axis);
		referenceAngle = bodyB.GetAngle() - bodyA.GetAngle();
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
	* The local translation axis in body1.
	*/
	 b2Vec2 localAxisA = new b2Vec2();

	/**
	* The constrained angle between the bodies: bodyB_angle - bodyA_angle.
	*/
	 double referenceAngle = 0.0;

	/**
	* Enable/disable the joint limit.
	*/
	 bool enableLimit;

	/**
	* The lower translation limit, usually in meters.
	*/
	 double lowerTranslation = 0.0;

	/**
	* The upper translation limit, usually in meters.
	*/
	 double upperTranslation = 0.0;

	/**
	* Enable/disable the joint motor.
	*/
	 bool enableMotor;

	/**
	* The maximum motor torque, usually in N-m.
	*/
	 double maxMotorForce = 0.0;

	/**
	* The desired motor speed in radians per second.
	*/
	 double motorSpeed = 0.0;
}

