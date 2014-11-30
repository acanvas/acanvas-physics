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
* Joint definitions are used to construct joints.
* @see b2Joint
*/
 class b2JointDef
{
	 b2JointDef()
	{
		type = b2Joint.e_unknownJoint;
		userData = null;
		bodyA = null;
		bodyB = null;
		collideConnected = false;
	}

	/**
	* The joint type is set automatically for concrete joint types.
	*/
	 int type = 0;
	/**
	* Use this to attach application specific data to your joints.
	*/
	 dynamic userData;
	/**
	* The first attached body.
	*/
	 b2Body bodyA;
	/**
	* The second attached body.
	*/
	 b2Body bodyB;
	/**
	* Set this flag to true if the attached bodies should collide.
	*/
	 bool collideConnected;
	
}

