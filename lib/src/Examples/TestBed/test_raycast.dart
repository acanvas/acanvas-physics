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
	
	
	
	
	
	
	 class TestRaycast extends Test{
		
		 b2Body laser;
	 TestRaycast(){
			// Set Text field
			Main.m_aboutText.text = "Raycast";
			
			m_world.SetGravity(new b2Vec2(0,0));
			
			b2Body ground = m_world.GetGroundBody();
			
			b2PolygonShape box = new b2PolygonShape();
			box.SetAsBox(30 / m_physScale, 4 / m_physScale);
			b2FixtureDef fd = new b2FixtureDef();
			fd.shape = box;
			fd.density = 4;
			fd.friction = 0.4;
			fd.restitution = 0.3;
			fd.userData="laser";
			b2BodyDef bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			bd.position.Set(320 / m_physScale, 150 / m_physScale);
			bd.position.Set(40 / m_physScale, 150 / m_physScale);
			laser = m_world.CreateBody(bd);
			laser.CreateFixture(fd);
			laser.SetAngle(0.5);
			laser.SetAngle(PI);
			
			b2CircleShape circle = new b2CircleShape(30 / m_physScale);
			fd.shape = circle;
			fd.density = 4;
			fd.friction = 0.4;
			fd.restitution = 0.3;
			fd.userData="circle";
			bd.position.Set(100 / m_physScale, 100 / m_physScale);
			b2Body body = m_world.CreateBody(bd);
			body.CreateFixture(fd);
		}
		
		
		//===============
		// Member Data 
		//===============
		
		 @override 
		 void Update(){
			super.Update();
			
			b2Vec2 p1 = laser.GetWorldPoint(new b2Vec2(30.1 / m_physScale, 0));
			b2Vec2 p2 = laser.GetWorldPoint(new b2Vec2(130.1 / m_physScale, 0));
			
			b2Fixture f = m_world.RayCastOne(p1, p2);
			double lambda = 1;
			if( f != null)
			{
				b2RayCastInput input = new b2RayCastInput(p1, p2);
				b2RayCastOutput output = new b2RayCastOutput();
				f.RayCast(output, input);
				lambda = output.fraction;
			}
			m_sprite.graphics.lineStyle(1,0xff0000,1);
			m_sprite.graphics.moveTo(p1.x * m_physScale, p1.y * m_physScale);
			m_sprite.graphics.lineTo( 	(p2.x * lambda + (1 - lambda) * p1.x) * m_physScale,
										(p2.y * lambda + (1 - lambda) * p1.y) * m_physScale);

		}
	}
	
