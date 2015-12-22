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
 class b2EdgeAndCircleContact extends b2Contact
{
	static   b2Contact Create(dynamic allocator){
		return new b2EdgeAndCircleContact();
	}
	static   void Destroy(b2Contact contact,dynamic allocator){
		//
	}

	  void Reset([b2Fixture fixtureA=null, b2Fixture fixtureB=null]){
		super.Reset(fixtureA, fixtureB);
		//b2Settings.b2Assert(m_shape1.m_type == b2Shape.e_circleShape);
		//b2Settings.b2Assert(m_shape2.m_type == b2Shape.e_circleShape);
	}
	//~b2EdgeAndCircleContact() {}
	
	@override 
		 void Evaluate(){
		b2Body bA = m_fixtureA.GetBody();
		b2Body bB = m_fixtureB.GetBody();
		b2CollideEdgeAndCircle(m_manifold,
					m_fixtureA.GetShape() as b2EdgeShape, bA.m_xf,
					m_fixtureB.GetShape() as b2CircleShape, bB.m_xf);
	}
	
	  void b2CollideEdgeAndCircle(b2Manifold manifold,b2EdgeShape edge,b2Transform xf1,b2CircleShape circle,b2Transform xf2)
	{
		//TODO_BORIS
		/*
		manifold.m_pointCount = 0;
		b2Mat22 tMat;
		b2Vec2 tVec;
		double dX = 0.0;
		double dY = 0.0;
		double tX = 0.0;
		double tY = 0.0;
		b2ManifoldPoint tPoint = 0;
		
		//b2Vec2 c = b2Mul(xf2, circle->GetLocalPosition());
		tMat = xf2.R;
		tVec = circle.m_r;
		double cX = xf2.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
		double cY = xf2.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
		
		//b2Vec2 cLocal = b2MulT(xf1, c);
		tMat = xf1.R;
		tX = cX - xf1.position.x;
		tY = cY - xf1.position.y;
		double cLocalX = (tX * tMat.col1.x + tY * tMat.col1.y );
		double cLocalY = (tX * tMat.col2.x + tY * tMat.col2.y );
		
		b2Vec2 n = edge.m_normal;
		b2Vec2 v1 = edge.m_v1;
		b2Vec2 v2 = edge.m_v2;
		double radius = circle.m_radius;
		double separation = 0.0;
		
		double dirDist = (cLocalX - v1.x) * edge.m_direction.x +
		                      (cLocalY - v1.y) * edge.m_direction.y;
		
		bool normalCalculated = false;
		
		if (dirDist <= 0) {
			dX = cLocalX - v1.x;
			dY = cLocalY - v1.y;
			if (dX * edge.m_cornerDir1.x + dY * edge.m_cornerDir1.y < 0) {
				return;
			}
			dX = cX - (xf1.position.x + (tMat.col1.x * v1.x + tMat.col2.x * v1.y));
			dY = cY - (xf1.position.y + (tMat.col1.y * v1.x + tMat.col2.y * v1.y));
		} else if (dirDist >= edge.m_length) {
			dX = cLocalX - v2.x;
			dY = cLocalY - v2.y;
			if (dX * edge.m_cornerDir2.x + dY * edge.m_cornerDir2.y > 0) {
				return;
			}
			dX = cX - (xf1.position.x + (tMat.col1.x * v2.x + tMat.col2.x * v2.y));
			dY = cY - (xf1.position.y + (tMat.col1.y * v2.x + tMat.col2.y * v2.y));
		} else {
			separation = (cLocalX - v1.x) * n.x + (cLocalY - v1.y) * n.y;
			if (separation > radius || separation < -radius) {
				return;
			}
			separation -= radius;
			
			//manifold.normal = b2Mul(xf1.R, n);
			tMat = xf1.R;
			manifold.normal.x = (tMat.col1.x * n.x + tMat.col2.x * n.y);
			manifold.normal.y = (tMat.col1.y * n.x + tMat.col2.y * n.y);
			
			normalCalculated = true;
		}
		
		if( normalCalculated == null || normalCalculated == false) {
			double distSqr = dX * dX + dY * dY;
			if (distSqr > radius * radius)
			{
				return;
			}
			
			if (distSqr < double.MIN_VALUE)
			{
				separation = -radius;
				manifold.normal.x = (tMat.col1.x * n.x + tMat.col2.x * n.y);
				manifold.normal.y = (tMat.col1.y * n.x + tMat.col2.y * n.y);
			}
			else
			{
				distSqr = Math.sqrt(distSqr);
				dX /= distSqr;
				dY /= distSqr;
				separation = distSqr - radius;
				manifold.normal.x = dX;
				manifold.normal.y = dY;
			}
		}
		
		tPoint = manifold.points[0];
		manifold.pointCount = 1;
		tPoint.id.key = 0;
		tPoint.separation = separation;
		cX = cX - radius * manifold.normal.x;
		cY = cY - radius * manifold.normal.y;
		
		tX = cX - xf1.position.x;
		tY = cY - xf1.position.y;
		tPoint.localPoint1.x = (tX * tMat.col1.x + tY * tMat.col1.y );
		tPoint.localPoint1.y = (tX * tMat.col2.x + tY * tMat.col2.y );
		
		tMat = xf2.R;
		tX = cX - xf2.position.x;
		tY = cY - xf2.position.y;
		tPoint.localPoint2.x = (tX * tMat.col1.x + tY * tMat.col1.y );
		tPoint.localPoint2.y = (tX * tMat.col2.x + tY * tMat.col2.y );
		*/
	}
}

