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
	
	 class TestBreakable extends Test{
//===============
  // Member Data 
  //===============
  
   b2Body m_body1;
   b2Vec2 m_velocity = new b2Vec2();
   double m_angularVelocity = 0.0;
   b2PolygonShape m_shape1 = new b2PolygonShape();
   b2PolygonShape m_shape2 = new b2PolygonShape();
   b2Fixture m_piece1;
   b2Fixture m_piece2;
   bool m_broke;
   bool m_break;
	 
	   TestBreakable(Stopwatch w) : super(w) {
			
			// Set Text field
			Main.m_aboutText.text = "Breakable";
			
			m_world.SetContactListener(new BreakableContactListener(this));
			
			b2Body ground = m_world.GetGroundBody();
			
			// Breakable Dynamic Body
			{
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(5.0, 5.0);
				bd.angle = 0.25 * PI;
				m_body1 = m_world.CreateBody(bd);
				
				m_shape1.SetAsOrientedBox(0.5, 0.5, new b2Vec2( -0.5, 0.0));
				m_piece1 = m_body1.CreateFixture2(m_shape1, 1.0);
				
				m_shape2.SetAsOrientedBox(0.5, 0.5, new b2Vec2( 0.5, 0.0));
				m_piece2 = m_body1.CreateFixture2(m_shape2, 1.0);
			}
			
			m_break = false;
			m_broke = false;
		}
		
		  void Break()
		{
			// Apply cached velocity for more realistic break
			m_body1.SetLinearVelocity(m_velocity);
			m_body1.SetAngularVelocity(m_angularVelocity);
			
			// Split body into two pieces
			m_body1.Split((b2Fixture fixture) {
				return fixture != m_piece1;
			});
		}
		
		@override 
		  void Update() 
		{
			super.Update();
			if (m_break)
			{
				Break();
				m_broke = true;
				m_break = false;
			}
			
			// Cache velocities to improve movement on breakage
			if (m_broke == false)
			{
				m_velocity = m_body1.GetLinearVelocity();
				m_angularVelocity = m_body1.GetAngularVelocity();
			}
		}
		
		
	}


class BreakableContactListener extends b2ContactListener
{
	 TestBreakable test;
	 BreakableContactListener(TestBreakable test)
	{
		this.test = test;
	}
	
	@override 
		  void PostSolve(b2Contact contact,b2ContactImpulse impulse) 
	{
		if (test.m_broke)
		{
			// The body already broke
			return;
		}
		
		// Should the body break?
		int count = contact.GetManifold().m_pointCount;
		
		double maxImpulse = 0.0;
		for (int i = 0; i < count; i++)
		{
			maxImpulse = b2Math.Max(maxImpulse, impulse.normalImpulses[i]);
		}
		
		if (maxImpulse > 50)
		{
			test.m_break = true;
		}
	}
	}
