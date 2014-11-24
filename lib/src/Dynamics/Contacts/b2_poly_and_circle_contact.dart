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
 class b2PolyAndCircleContact extends b2Contact{
	
	static   b2Contact Create(dynamic allocator){
		return new b2PolyAndCircleContact();
	}
	static   void Destroy(b2Contact contact,dynamic allocator){
	}

	  void Reset([b2Fixture fixtureA=null, b2Fixture fixtureB=null]){
		super.Reset(fixtureA, fixtureB);
		b2Settings.b2Assert(fixtureA.GetType() == b2Shape.e_polygonShape);
		b2Settings.b2Assert(fixtureB.GetType() == b2Shape.e_circleShape);
	}
	//~b2PolyAndCircleContact() {}

	@override 
		 void Evaluate(){
		b2Body bA = m_fixtureA.m_body;
		b2Body bB = m_fixtureB.m_body;
		
		b2Collision.CollidePolygonAndCircle(m_manifold, 
					m_fixtureA.GetShape() as b2PolygonShape, bA.m_xf, 
					m_fixtureB.GetShape() as b2CircleShape, bB.m_xf);
	}
}

