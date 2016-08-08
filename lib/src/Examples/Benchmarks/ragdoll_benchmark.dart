/*
* Copyright (2009 as c) Adam Newgas http://www.boristhebrave.com
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

 part of rockdot_physics;

	 class RagdollBenchmark implements IBenchmark
	{
		 double angularVelocity = 1.0;
		  String Name()
		{
			return "Ragdoll";
		}
		
		
		  void MakeRagdoll(b2World world,b2Vec2 pos)
		{
			double m_physScale = 30.0;
			double startX = pos.x * m_physScale;
			double startY = pos.y * m_physScale;
			b2World m_world = world;
				b2CircleShape circ; 
				b2PolygonShape box;
				b2BodyDef bd = new b2BodyDef();
				b2RevoluteJointDef jd = new b2RevoluteJointDef();
				b2FixtureDef fixtureDef = new b2FixtureDef();
			{
				// BODIES
				// Set these to dynamic bodies
				bd.type = b2Body.b2_dynamicBody;
				
				// Head
				circ = new b2CircleShape( 12.5 / m_physScale );
				fixtureDef.shape = circ;
				fixtureDef.density = 1.0;
				fixtureDef.friction = 0.4;
				fixtureDef.restitution = 0.3;
				bd.position.Set(startX / m_physScale, startY / m_physScale);
				b2Body head = m_world.CreateBody(bd);
				head.CreateFixture(fixtureDef);
				//if (i == 0){
					head.ApplyImpulse(new b2Vec2(0.5 * 100 - 50, 0.5 * 100 - 50), head.GetWorldCenter());
				//}
				
				// Torso1
				box = new b2PolygonShape();
				box.SetAsBox(15 / m_physScale, 10 / m_physScale);
				fixtureDef.shape = box;
				fixtureDef.density = 1.0;
				fixtureDef.friction = 0.4;
				fixtureDef.restitution = 0.1;
				bd.position.Set(startX / m_physScale, (startY + 28) / m_physScale);
				b2Body torso1 = m_world.CreateBody(bd);
				torso1.CreateFixture(fixtureDef);
				// Torso2
				box = new b2PolygonShape();
				box.SetAsBox(15 / m_physScale, 10 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set(startX / m_physScale, (startY + 43) / m_physScale);
				b2Body torso2 = m_world.CreateBody(bd);
				torso2.CreateFixture(fixtureDef);
				// Torso3
				box.SetAsBox(15 / m_physScale, 10 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set(startX / m_physScale, (startY + 58) / m_physScale);
				b2Body torso3 = m_world.CreateBody(bd);
				torso3.CreateFixture(fixtureDef);
				
				// UpperArm
				fixtureDef.density = 1.0;
				fixtureDef.friction = 0.4;
				fixtureDef.restitution = 0.1;
				// L
				box = new b2PolygonShape();
				box.SetAsBox(18 / m_physScale, 6.5 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set((startX - 30) / m_physScale, (startY + 20) / m_physScale);
				b2Body upperArmL = m_world.CreateBody(bd);
				upperArmL.CreateFixture(fixtureDef);
				// R
				box = new b2PolygonShape();
				box.SetAsBox(18 / m_physScale, 6.5 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set((startX + 30) / m_physScale, (startY + 20) / m_physScale);
				b2Body upperArmR = m_world.CreateBody(bd);
				upperArmR.CreateFixture(fixtureDef);
				
				// LowerArm
				fixtureDef.density = 1.0;
				fixtureDef.friction = 0.4;
				fixtureDef.restitution = 0.1;
				// L
				box = new b2PolygonShape();
				box.SetAsBox(17 / m_physScale, 6 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set((startX - 57) / m_physScale, (startY + 20) / m_physScale);
				b2Body lowerArmL = m_world.CreateBody(bd);
				lowerArmL.CreateFixture(fixtureDef);
				// R
				box = new b2PolygonShape();
				box.SetAsBox(17 / m_physScale, 6 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set((startX + 57) / m_physScale, (startY + 20) / m_physScale);
				b2Body lowerArmR = m_world.CreateBody(bd);
				lowerArmR.CreateFixture(fixtureDef);
				
				// UpperLeg
				fixtureDef.density = 1.0;
				fixtureDef.friction = 0.4;
				fixtureDef.restitution = 0.1;
				// L
				box = new b2PolygonShape();
				box.SetAsBox(7.5 / m_physScale, 22 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set((startX - 8) / m_physScale, (startY + 85) / m_physScale);
				b2Body upperLegL = m_world.CreateBody(bd);
				upperLegL.CreateFixture(fixtureDef);
				// R
				box = new b2PolygonShape();
				box.SetAsBox(7.5 / m_physScale, 22 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set((startX + 8) / m_physScale, (startY + 85) / m_physScale);
				b2Body upperLegR = m_world.CreateBody(bd);
				upperLegR.CreateFixture(fixtureDef);
				
				// LowerLeg
				fixtureDef.density = 1.0;
				fixtureDef.friction = 0.4;
				fixtureDef.restitution = 0.1;
				// L
				box = new b2PolygonShape();
				box.SetAsBox(6 / m_physScale, 20 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set((startX - 8) / m_physScale, (startY + 120) / m_physScale);
				b2Body lowerLegL = m_world.CreateBody(bd);
				lowerLegL.CreateFixture(fixtureDef);
				// R
				box = new b2PolygonShape();
				box.SetAsBox(6 / m_physScale, 20 / m_physScale);
				fixtureDef.shape = box;
				bd.position.Set((startX + 8) / m_physScale, (startY + 120) / m_physScale);
				b2Body lowerLegR = m_world.CreateBody(bd);
				lowerLegR.CreateFixture(fixtureDef);
				
				
				// JOINTS
				jd.enableLimit = true;
				
				// Head to shoulders
				jd.lowerAngle = -40 / (180/PI);
				jd.upperAngle = 40 / (180/PI);
				jd.Initialize(torso1, head, new b2Vec2(startX / m_physScale, (startY + 15) / m_physScale));
				m_world.CreateJoint(jd);
				
				// Upper arm to shoulders
				// L
				jd.lowerAngle = -85 / (180/PI);
				jd.upperAngle = 130 / (180/PI);
				jd.Initialize(torso1, upperArmL, new b2Vec2((startX - 18) / m_physScale, (startY + 20) / m_physScale));
				m_world.CreateJoint(jd);
				// R
				jd.lowerAngle = -130 / (180/PI);
				jd.upperAngle = 85 / (180/PI);
				jd.Initialize(torso1, upperArmR, new b2Vec2((startX + 18) / m_physScale, (startY + 20) / m_physScale));
				m_world.CreateJoint(jd);
				
				// Lower arm to upper arm
				// L
				jd.lowerAngle = -130 / (180/PI);
				jd.upperAngle = 10 / (180/PI);
				jd.Initialize(upperArmL, lowerArmL, new b2Vec2((startX - 45) / m_physScale, (startY + 20) / m_physScale));
				m_world.CreateJoint(jd);
				// R
				jd.lowerAngle = -10 / (180/PI);
				jd.upperAngle = 130 / (180/PI);
				jd.Initialize(upperArmR, lowerArmR, new b2Vec2((startX + 45) / m_physScale, (startY + 20) / m_physScale));
				m_world.CreateJoint(jd);
				
				// Shoulders/stomach
				jd.lowerAngle = -15 / (180/PI);
				jd.upperAngle = 15 / (180/PI);
				jd.Initialize(torso1, torso2, new b2Vec2(startX / m_physScale, (startY + 35) / m_physScale));
				m_world.CreateJoint(jd);
				// Stomach/hips
				jd.Initialize(torso2, torso3, new b2Vec2(startX / m_physScale, (startY + 50) / m_physScale));
				m_world.CreateJoint(jd);
				
				// Torso to upper leg
				// L
				jd.lowerAngle = -25 / (180/PI);
				jd.upperAngle = 45 / (180/PI);
				jd.Initialize(torso3, upperLegL, new b2Vec2((startX - 8) / m_physScale, (startY + 72) / m_physScale));
				m_world.CreateJoint(jd);
				// R
				jd.lowerAngle = -45 / (180/PI);
				jd.upperAngle = 25 / (180/PI);
				jd.Initialize(torso3, upperLegR, new b2Vec2((startX + 8) / m_physScale, (startY + 72) / m_physScale));
				m_world.CreateJoint(jd);
				
				// Upper leg to lower leg
				// L
				jd.lowerAngle = -25 / (180/PI);
				jd.upperAngle = 115 / (180/PI);
				jd.Initialize(upperLegL, lowerLegL, new b2Vec2((startX - 8) / m_physScale, (startY + 105) / m_physScale));
				m_world.CreateJoint(jd);
				// R
				jd.lowerAngle = -115 / (180/PI);
				jd.upperAngle = 25 / (180/PI);
				jd.Initialize(upperLegR, lowerLegR, new b2Vec2((startX + 8) / m_physScale, (startY + 105) / m_physScale));
				m_world.CreateJoint(jd);
			}
		}
		
		  void Init(b2World world)
		{
			world.SetGravity(new b2Vec2(0.0, 10.0));
			
			b2BodyDef groundDef = new b2BodyDef();
			groundDef.type = b2Body.b2_kinematicBody;
			groundDef.angularVelocity = angularVelocity;
			groundDef.position.Set(5.0, 5.0);
			b2PolygonShape ground = new b2PolygonShape();
			b2Body body;
			body = world.CreateBody(groundDef);
			ground.SetAsOrientedBox(.5, 5.0, new b2Vec2(-4.5, 0.0));
			body.CreateFixture2(ground);
			ground.SetAsOrientedBox(5.0, .5, new b2Vec2(0.0, -4.5));
			body.CreateFixture2(ground);
			ground.SetAsOrientedBox(5.0, .5, new b2Vec2(0.0, 4.5));
			body.CreateFixture2(ground);
			ground.SetAsOrientedBox(.5, 5.0, new b2Vec2(4.5, 0.0));
			body.CreateFixture2(ground);
			
			MakeRagdoll(world, new b2Vec2(3.0, 2.0));
			MakeRagdoll(world, new b2Vec2(5.0, 5.0));
			MakeRagdoll(world, new b2Vec2(7.0, 2.0));
			
		}
		
		  void Update()
		{
			
		}
	}
