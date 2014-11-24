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
* @
*/
 class b2ContactConstraint
{
	 b2ContactConstraint(){
		points = new List<b2ContactConstraintPoint>(b2Settings.b2_maxManifoldPoints);
		for (int i = 0; i < b2Settings.b2_maxManifoldPoints; i++){
			points[i] = new b2ContactConstraintPoint();
		}
		
		
	}
	 List<b2ContactConstraintPoint> points;
	 b2Vec2 localPlaneNormal = new b2Vec2();
	 b2Vec2 localPoint = new b2Vec2();
	 b2Vec2 normal = new b2Vec2();
	 b2Mat22 normalMass = new b2Mat22();
	 b2Mat22 K = new b2Mat22();
	 b2Body bodyA;
	 b2Body bodyB;
	 int type;//b2Manifold::Type
	 double radius;
	 double friction;
	 double restitution;
	 int pointCount;
	 b2Manifold manifold;
}


