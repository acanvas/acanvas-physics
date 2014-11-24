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
	
	
	
	
	
	
	 class TestCCD extends Test{
	 TestCCD(Stopwatch w) : super(w) {
			
			// Set Text field
			Main.m_aboutText.text = "Continuous Collision Detection";
			
			b2BodyDef bd;
			b2Body body;
			b2FixtureDef fixtureDef = new b2FixtureDef();
			// These values are used for all the parts of the 'basket'
			fixtureDef.density = 4.0; 
			fixtureDef.restitution = 1.4;
			
			// Create 'basket'
			{
				bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.bullet = true;
				bd.position.Set( 150.0/m_physScale, 100.0/m_physScale );
				body = m_world.CreateBody(bd);
				b2PolygonShape sd_bottom = new b2PolygonShape();
				sd_bottom.SetAsBox( 45.0 / m_physScale, 4.5 / m_physScale );
				fixtureDef.shape = sd_bottom;
				body.CreateFixture( fixtureDef );
				
				b2PolygonShape sd_left = new b2PolygonShape();
				sd_left.SetAsOrientedBox(4.5/m_physScale, 81.0/m_physScale, new b2Vec2(-43.5/m_physScale, -70.5/m_physScale), -0.2);
				fixtureDef.shape = sd_left;
				body.CreateFixture( fixtureDef );
				
				b2PolygonShape sd_right = new b2PolygonShape();
				sd_right.SetAsOrientedBox(4.5/m_physScale, 81.0/m_physScale, new b2Vec2(43.5/m_physScale, -70.5/m_physScale), 0.2);
				fixtureDef.shape = sd_right;
				body.CreateFixture( fixtureDef );
			}
			
			// add some small circles for effect
			for (int i = 0; i < 5; i++){
				b2CircleShape cd = new b2CircleShape((new Random().nextDouble() * 10 + 5) / m_physScale);
				fixtureDef.shape = cd;
				fixtureDef.friction = 0.3;
				fixtureDef.density = 1.0;
				fixtureDef.restitution = 1.1;
				bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.bullet = true;
				bd.position.Set( (new Random().nextDouble()*300 + 250)/m_physScale, (new Random().nextDouble()*320 + 20)/m_physScale );
				body = m_world.CreateBody(bd);
				body.CreateFixture(fixtureDef);
			}
			
		}
		
		
		//===============
		// Member Data 
		//===============
		
	}
	
