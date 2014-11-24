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
	
	
	
	
	
	
	 class TestBridge extends Test{
	 TestBridge(){
			
			// Set Text field
			Main.m_aboutText.text = "Bridge";
			
			b2Body ground = m_world.GetGroundBody();
			int i;
			b2Vec2 anchor = new b2Vec2();
			b2Body body;
			
			// Bridge
			{
				b2PolygonShape sd = new b2PolygonShape();
				b2FixtureDef fixtureDef = new b2FixtureDef();
				sd.SetAsBox(24 / m_physScale, 5 / m_physScale);
				fixtureDef.shape = sd;
				fixtureDef.density = 20.0;
				fixtureDef.friction = 0.2;
				
				b2BodyDef bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				
				b2RevoluteJointDef jd = new b2RevoluteJointDef();
				const int doublePlanks = 10;
				jd.lowerAngle = -15 / (180/PI);
				jd.upperAngle = 15 / (180/PI);
				jd.enableLimit = true;
				
				b2Body prevBody = ground;
				for (i = 0; i < doublePlanks; ++i)
				{
					bd.position.Set((100 + 22 + 44 * i) / m_physScale, 250 / m_physScale);
					body = m_world.CreateBody(bd);
					body.CreateFixture(fixtureDef);
					
					anchor.Set((100 + 44 * i) / m_physScale, 250 / m_physScale);
					jd.Initialize(prevBody, body, anchor);
					m_world.CreateJoint(jd);
					
					prevBody = body;
				}
				
				anchor.Set((100 + 44 * doublePlanks) / m_physScale, 250 / m_physScale);
				jd.Initialize(prevBody, ground, anchor);
				m_world.CreateJoint(jd);
			}
			
			
			
			
			
			
			
			
			// Spawn in a bunch of crap
			for (i = 0; i < 5; i++){
				b2BodyDef bodyDef = new b2BodyDef();
				bodyDef.type = b2Body.b2_dynamicBody;
				b2PolygonShape boxShape = new b2PolygonShape();
				fixtureDef.shape = boxShape
				fixtureDef.density = 1.0;
				// Override the default friction.
				fixtureDef.friction = 0.3;
				fixtureDef.restitution = 0.1;
				boxShape.SetAsBox((new Random().nextDouble() * 5 + 10) / m_physScale, (new Random().nextDouble() * 5 + 10) / m_physScale);
				bodyDef.position.Set((new Random().nextDouble() * 400 + 120) / m_physScale, (new Random().nextDouble() * 150 + 50) / m_physScale);
				bodyDef.angle = new Random().nextDouble() * PI;
				body = m_world.CreateBody(bodyDef);
				body.CreateFixture(fixtureDef);
				
			}
			for (i = 0; i < 5; i++){
				b2BodyDef bodyDefC = new b2BodyDef();
				bodyDefC.type = b2Body.b2_dynamicBody;
				b2CircleShape circShape = new b2CircleShape((new Random().nextDouble() * 5 + 10) / m_physScale);
				fixtureDef.shape = circShape
				fixtureDef.density = 1.0;
				// Override the default friction.
				fixtureDef.friction = 0.3;
				fixtureDef.restitution = 0.1;
				bodyDefC.position.Set((new Random().nextDouble() * 400 + 120) / m_physScale, (new Random().nextDouble() * 150 + 50) / m_physScale);
				bodyDefC.angle = new Random().nextDouble() * PI;
				body = m_world.CreateBody(bodyDefC);
				body.CreateFixture(fixtureDef);
				
			}
			int j;
			for (i = 0; i < 15; i++){
				b2BodyDef bodyDefP = new b2BodyDef();
				bodyDefP.type = b2Body.b2_dynamicBody;
				b2PolygonShape polyShape = new b2PolygonShape();
				List vertices = new List();
				int vertexCount;
				if (new Random().nextDouble() > 0.66){
					vertexCount = 4;
					for ( j = 0; j < vertexCount; ++j )
					{
						vertices[j] = new b2Vec2();
					}
					vertices[0].Set((-10 -new Random().nextDouble()*10) / m_physScale, ( 10 +new Random().nextDouble()*10) / m_physScale);
					vertices[1].Set(( -5 -new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale);
					vertices[2].Set((  5 +new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale);
					vertices[3].Set(( 10 +new Random().nextDouble()*10) / m_physScale, ( 10 +new Random().nextDouble()*10) / m_physScale);
				}
				else if (new Random().nextDouble() > 0.5){
					vertexCount = 5;
					for ( j = 0; j < vertexCount; ++j )
					{
						vertices[j] = new b2Vec2();
					}
					vertices[0].Set(0, (10 +new Random().nextDouble()*10) / m_physScale);
					vertices[2].Set((-5 -new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale);
					vertices[3].Set(( 5 +new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale);
					vertices[1].Set((vertices[0].x + vertices[2].x), (vertices[0].y + vertices[2].y));
					vertices[1].Multiply(new Random().nextDouble()/2+0.8);
					vertices[4].Set((vertices[3].x + vertices[0].x), (vertices[3].y + vertices[0].y));
					vertices[4].Multiply(new Random().nextDouble()/2+0.8);
				}
				else{
					vertexCount = 3;
					for ( j = 0; j < vertexCount; ++j )
					{
						vertices[j] = new b2Vec2();
					}
					vertices[0].Set(0, (10 +new Random().nextDouble()*10) / m_physScale);
					vertices[1].Set((-5 -new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale);
					vertices[2].Set(( 5 +new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale);
				}
				polyShape.SetAsList( vertices, vertexCount );
				fixtureDef.shape = polyShape;
				fixtureDef.density = 1.0;
				fixtureDef.friction = 0.3;
				fixtureDef.restitution = 0.1;
				bodyDefP.position.Set((new Random().nextDouble() * 400 + 120) / m_physScale, (new Random().nextDouble() * 150 + 50) / m_physScale);
				bodyDefP.angle = new Random().nextDouble() * PI;
				body = m_world.CreateBody(bodyDefP);
				body.CreateFixture(fixtureDef);
			}
			
		}
		
		
		//===============
		// Member Data 
		//===============
	}
	
