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
	
	
	
	
	
	
	 class TestBuoyancy extends Test{
		
		 List m_bodies = new List();
		 b2Controller m_controller;
	 TestBuoyancy(Stopwatch w) : super(w) {
			b2BuoyancyController bc = new b2BuoyancyController();
			m_controller = bc;
			
			bc.normal.Set(0.0,-1.0);
			bc.offset = -200 / m_physScale;
			bc.density = 2.0;
			bc.linearDrag = 5.0;
			bc.angularDrag = 2.0;
			
			b2Body ground = m_world.GetGroundBody();
			int i = 0;
			b2Vec2 anchor = new b2Vec2();
			b2Body body;
			b2FixtureDef fd;

			b2BodyDef bodyDef;
			b2PolygonShape boxDef;
			b2CircleShape circDef;
			
			// Spawn in a bunch of crap
			for (i = 0; i < 5; i++){
				bodyDef = new b2BodyDef();
				bodyDef.type = b2Body.b2_dynamicBody;
				//bodyDef.isBullet = true;
				boxDef = new b2PolygonShape();
				fd = new b2FixtureDef();
				fd.shape = boxDef;
				fd.density = 1.0;
				// Override the default friction.
				fd.friction = 0.3;
				fd.restitution = 0.1;
				boxDef.SetAsBox((new Random().nextDouble() * 5 + 10) / m_physScale, (new Random().nextDouble() * 5 + 10) / m_physScale);
				bodyDef.position.Set((new Random().nextDouble() * 400 + 120) / m_physScale, (new Random().nextDouble() * 150 + 50) / m_physScale);
				bodyDef.angle = new Random().nextDouble() * PI;
				body = m_world.CreateBody(bodyDef);
				body.CreateFixture(fd);
				m_bodies.add(body);
				
			}
			for (i = 0; i < 5; i++){
				b2BodyDef bodyDefC = new b2BodyDef();
				bodyDefC.type = b2Body.b2_dynamicBody;
				//bodyDefC.isBullet = true;
				circDef = new b2CircleShape((new Random().nextDouble() * 5 + 10) / m_physScale);
				fd = new b2FixtureDef();
				fd.shape = circDef;
				fd.density = 1.0;
				// Override the default friction.
				fd.friction = 0.3;
				fd.restitution = 0.1;
				bodyDefC.position.Set((new Random().nextDouble() * 400 + 120) / m_physScale, (new Random().nextDouble() * 150 + 50) / m_physScale);
				bodyDefC.angle = new Random().nextDouble() * PI;
				body = m_world.CreateBody(bodyDefC);
				body.CreateFixture(fd);
				m_bodies.add(body);
			}
			for (i = 0; i < 15; i++){
				b2BodyDef bodyDefP = new b2BodyDef();
				bodyDefP.type = b2Body.b2_dynamicBody;
				//bodyDefP.isBullet = true;
				b2PolygonShape polyDef = new b2PolygonShape();
				if (new Random().nextDouble() > 0.66) {
					polyDef.SetAsVector([
						new b2Vec2((-10 -new Random().nextDouble()*10) / m_physScale, ( 10 +new Random().nextDouble()*10) / m_physScale),
						new b2Vec2(( -5 -new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale),
						new b2Vec2((  5 +new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale),
						new b2Vec2(( 10 +new Random().nextDouble() * 10) / m_physScale, ( 10 +new Random().nextDouble() * 10) / m_physScale)
						]);
				}
				else if (new Random().nextDouble() > 0.5) 
				{
					List array = [];
					b2Vec2 a0 = new b2Vec2(0.0, (10 +new Random().nextDouble()*10) / m_physScale);
					array.add( a0);
					b2Vec2 a2 = new b2Vec2((-5 -new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale);
					b2Vec2 a3 = new b2Vec2(( 5 +new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale);

					b2Vec2 a1 = new b2Vec2((a0.x + a2.x), (a0.y + a2.y));
					a1.Multiply(new Random().nextDouble()/2+0.8);
					
					b2Vec2 a4 = new b2Vec2((a3.x + a0.x), (a3.y + a0.y));
					a4.Multiply(new Random().nextDouble() / 2 + 0.8);
					polyDef.SetAsVector([a0, a1, a2, a3, a4]);
				}
				else 
				{
					polyDef.SetAsVector([
						new b2Vec2(0.0, (10 +new Random().nextDouble()*10) / m_physScale),
						new b2Vec2((-5 -new Random().nextDouble()*10) / m_physScale, (-10 -new Random().nextDouble()*10) / m_physScale),
						new b2Vec2(( 5 +new Random().nextDouble() * 10) / m_physScale, ( -10 -new Random().nextDouble() * 10) / m_physScale)
					]);
				}
				fd = new b2FixtureDef();
				fd.shape = polyDef;
				fd.density = 1.0;
				fd.friction = 0.3;
				fd.restitution = 0.1;
				bodyDefP.position.Set((new Random().nextDouble() * 400 + 120) / m_physScale, (new Random().nextDouble() * 150 + 50) / m_physScale);
				bodyDefP.angle = new Random().nextDouble() * PI;
				body = m_world.CreateBody(bodyDefP);
				body.CreateFixture(fd);
				m_bodies.add(body);
			}
			
			//Add some exciting bath toys
			boxDef.SetAsBox(40 / m_physScale, 10 / m_physScale);
			fd = new b2FixtureDef();
			fd.shape = boxDef;
			fd.density = 3.0;
			bodyDef.position.Set(50 / m_physScale, 300 / m_physScale);
			bodyDef.angle = 0.0;
			body = m_world.CreateBody(bodyDef);
			body.CreateFixture(fd);
			m_bodies.add(body);
			
			bodyDef.position.Set(300/ m_physScale, 300 / m_physScale);
			body = m_world.CreateBody(bodyDef);
			circDef = new b2CircleShape(7 / m_physScale);
			fd = new b2FixtureDef();
			fd.shape = circDef;
			fd.density =2.0;
			circDef.m_p.Set(30 / m_physScale, 0 / m_physScale);
			body.CreateFixture(fd);
			circDef.m_p.Set(-30 / m_physScale, 0 / m_physScale);
			body.CreateFixture(fd);
			circDef.m_p.Set(0 / m_physScale, 30 / m_physScale);
			body.CreateFixture(fd);
			circDef.m_p.Set(0 / m_physScale, -30 / m_physScale);
			body.CreateFixture(fd);
			
			fd = new b2FixtureDef();
			fd.shape = boxDef;
			fd.density = 2.0;
			boxDef.SetAsBox(30 / m_physScale, 2 / m_physScale);
			body.CreateFixture(fd);
			fd.density = 2.0;
			boxDef.SetAsBox(2 / m_physScale, 30 / m_physScale);
			body.CreateFixture(fd);
			m_bodies.add(body);
			
			for(body in m_bodies)
				m_controller.AddBody(body);
			m_world.AddController(m_controller);
			
			// Set Text field
			Main.m_aboutText.text = "Buoyancy";
			
		}
		
		
		
		//===============
		// Member Data 
		//===============
		
		 @override 
		 void Update(){
			
			super.Update();
			//Draw water line
			m_sprite.graphics.moveTo(5,200);
			m_sprite.graphics.lineTo(635,200);
			m_sprite.graphics.strokeColor(0xff0000ff,1);
			//It's not water without transparency...
			m_sprite.graphics.beginPath();
			m_sprite.graphics.moveTo(5,200);
			m_sprite.graphics.lineTo(635,200);
			m_sprite.graphics.lineTo(635,355);
			m_sprite.graphics.lineTo(5,355);
			m_sprite.graphics.closePath();
			m_sprite.graphics.fillColor(0x440000ff);
			//m_sprite.graphics.endFill() //not supported in StageXL;

		}
	}
	
