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

 part of rockdot_box2d;
	
	





/**
* Pulley joint definition. This requires two ground anchors,
* two dynamic body anchor points, max lengths for side,
* and a pulley ratio.
* @see b2PulleyJoint
*/

 class b2PulleyJointDef extends b2JointDef
{
	 b2PulleyJointDef()
	{
		type = b2Joint.e_pulleyJoint;
		groundAnchorA.Set(-1.0, 1.0);
		groundAnchorB.Set(1.0, 1.0);
		localAnchorA.Set(-1.0, 0.0);
		localAnchorB.Set(1.0, 0.0);
		lengthA = 0.0;
		maxLengthA = 0.0;
		lengthB = 0.0;
		maxLengthB = 0.0;
		ratio = 1.0;
		collideConnected = true;
	}
	
	  void Initialize(b2Body bA,b2Body bB,b2Vec2 gaA,b2Vec2 gaB,b2Vec2 anchorA,b2Vec2 anchorB,double r)
	{
		bodyA = bA;
		bodyB = bB;
		groundAnchorA.SetV( gaA );
		groundAnchorB.SetV( gaB );
		localAnchorA = bodyA.GetLocalPoint(anchorA);
		localAnchorB = bodyB.GetLocalPoint(anchorB);
		//b2Vec2 d1 = anchorA - gaA;
		double d1X = anchorA.x - gaA.x;
		double d1Y = anchorA.y - gaA.y;
		//length1 = d1.Length();
		lengthA = sqrt(d1X*d1X + d1Y*d1Y);
		
		//b2Vec2 d2 = anchor2 - ga2;
		double d2X = anchorB.x - gaB.x;
		double d2Y = anchorB.y - gaB.y;
		//length2 = d2.Length();
		lengthB = sqrt(d2X*d2X + d2Y*d2Y);
		
		ratio = r;
		//b2Settings.b2Assert(ratio > double.MIN_VALUE);
		double C = lengthA + ratio * lengthB;
		maxLengthA = C - ratio * b2PulleyJoint.b2_minPulleyLength;
		maxLengthB = (C - b2PulleyJoint.b2_minPulleyLength) / ratio;
	}

	/**
	* The first ground anchor in world coordinates. This point never moves.
	*/
	 b2Vec2 groundAnchorA = new b2Vec2();
	
	/**
	* The second ground anchor in world coordinates. This point never moves.
	*/
	 b2Vec2 groundAnchorB = new b2Vec2();
	
	/**
	* The local anchor point relative to bodyA's origin.
	*/
	 b2Vec2 localAnchorA = new b2Vec2();
	
	/**
	* The local anchor point relative to bodyB's origin.
	*/
	 b2Vec2 localAnchorB = new b2Vec2();
	
	/**
	* The a reference length for the segment attached to bodyA.
	*/
	 double lengthA = 0.0;
	
	/**
	* The maximum length of the segment attached to bodyA.
	*/
	 double maxLengthA = 0.0;
	
	/**
	* The a reference length for the segment attached to bodyB.
	*/
	 double lengthB = 0.0;
	
	/**
	* The maximum length of the segment attached to bodyB.
	*/
	 double maxLengthB = 0.0;
	
	/**
	* The pulley ratio, used to simulate a block-and-tackle.
	*/
	 double ratio = 0.0;
	
}

