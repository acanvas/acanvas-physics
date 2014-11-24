/*
* Copyright (2006 as c)-2007 Adam Newgas
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
 * Calculates buoyancy forces for fluids in the form of a half plane
 */
 class b2BuoyancyController extends b2Controller
{
	/**
	 * The outer surface normal
	 */
	 b2Vec2 normal = new b2Vec2(0.0,-1.0);
	/**
	 * The height of the fluid surface along the normal
	 */
	 double offset = 0.0;
	/**
	 * The fluid density
	 */
	 double density = 0.0;
	/**
	 * Fluid velocity, for drag calculations
	 */
	 b2Vec2 velocity = new b2Vec2(0.0,0.0);
	/**
	 * Linear drag co-efficient
	 */
	 double linearDrag = 2.0;
	/**
	 * Linear drag co-efficient
	 */
	 double angularDrag = 1.0;
	/**
	 * If false, bodies are assumed to be uniformly dense, otherwise use the shapes densities
	 */
	 bool useDensity = false; //False by default to prevent a gotcha
	/**
	 * If true, gravity is taken from the world instead of the gravity parameter.
	 */
	 bool useWorldGravity = true;
	/**
	 * Gravity vector, if the world's gravity is not used
	 */
	 b2Vec2 gravity = null;
	
		
	 @override 
		 void Step(b2TimeStep step){
		if(m_bodyList == null)
			return;
		if( useWorldGravity){
			gravity = GetWorld().GetGravity().Copy();
		}
		for(b2ControllerEdge i=m_bodyList;i != null;i=i.nextBody){
			b2Body body = i.body;
			if(body.IsAwake() == false){
				//Buoyancy force is just a function of position,
				//so unlike most forces, it is safe to ignore sleeping bodes
				continue;
			}
			b2Vec2 areac = new b2Vec2();
			b2Vec2 massc = new b2Vec2();
			double area = 0.0;
			double mass = 0.0;
			for(b2Fixture fixture=body.GetFixtureList();fixture != null;fixture=fixture.GetNext()){
				b2Vec2 sc = new b2Vec2();
				double sarea = fixture.GetShape().ComputeSubmergedArea(normal, offset, body.GetTransform(), sc);
				area += sarea;
				areac.x += sarea * sc.x;
				areac.y += sarea * sc.y;
				double shapeDensity;
				if( useDensity ) {
					//TODO: Figure out what to do now density is gone
					shapeDensity = 1.0;
				}else{
					shapeDensity = 1.0;
				}
				mass += sarea*shapeDensity;
				massc.x += sarea * sc.x * shapeDensity;
				massc.y += sarea * sc.y * shapeDensity;
			}
			areac.x/=area;
			areac.y/=area;
			massc.x/=mass;
			massc.y/=mass;
			if(area<double.MIN_POSITIVE)
				continue;
			//Buoyancy
			b2Vec2 buoyancyForce = gravity.GetNegative();
			buoyancyForce.Multiply(density*area);
			body.ApplyForce(buoyancyForce,massc);
			//Linear drag
			b2Vec2 dragForce = body.GetLinearVelocityFromWorldPoint(areac);
			dragForce.Subtract(velocity);
			dragForce.Multiply(-linearDrag*area);
			body.ApplyForce(dragForce,areac);
			//Angular drag
			//TODO: Something that makes more physical sense?
			body.ApplyTorque(-body.GetInertia()/body.GetMass()*area*body.GetAngularVelocity()*angularDrag);
			
		}
	}
	
	 @override 
		 void Draw(b2DebugDraw debugDraw)
	{
		double r = 1000.0;
		//Would like to draw a semi-transparent box
		//But debug draw doesn't support that
		b2Vec2 p1 = new b2Vec2();
		b2Vec2 p2 = new b2Vec2();
		p1.x = normal.x * offset + normal.y * r;
		p1.y = normal.y * offset - normal.x * r;
		p2.x = normal.x * offset - normal.y * r;
		p2.y = normal.y * offset + normal.x * r;
		b2Color color = new b2Color(0.0,0.0,1.0);
		debugDraw.DrawSegment(p1,p2,color);
	}
}

