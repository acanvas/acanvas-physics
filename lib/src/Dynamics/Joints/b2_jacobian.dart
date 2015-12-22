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
* @
*/
 class b2Jacobian
{
	 b2Vec2 linearA = new b2Vec2();
	 double angularA = 0.0;
	 b2Vec2 linearB = new b2Vec2();
	 double angularB = 0.0;

	  void SetZero(){
		linearA.SetZero(); angularA = 0.0;
		linearB.SetZero(); angularB = 0.0;
	}
	  void Set(b2Vec2 x1,double a1,b2Vec2 x2,double a2){
		linearA.SetV(x1); angularA = a1;
		linearB.SetV(x2); angularB = a2;
	}
	  double Compute(b2Vec2 x1,double a1,b2Vec2 x2,double a2){
		
		//return b2Math.b2Dot(linearA, x1) + angularA * a1 + b2Math.b2Dot(linearV, x2) + angularV * a2;
		return (linearA.x*x1.x + linearA.y*x1.y) + angularA * a1 + (linearB.x*x2.x + linearB.y*x2.y) + angularB * a2;
	}
}


