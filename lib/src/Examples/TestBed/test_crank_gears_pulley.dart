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
	
	
	
	
	
	
	 class TestCrankGearsPulley extends Test{
	 TestCrankGearsPulley(Stopwatch w) : super(w) {
			
			// Set Text field
			Main.m_aboutText.text = "Joints";
			
			b2Body ground = m_world.GetGroundBody();
			
			b2Body body;
			b2Body circleBody;
			b2PolygonShape sd;
			b2BodyDef bd;
			b2FixtureDef fixtureDef = new b2FixtureDef();
			
			//
			// CRANK
			//
			{
				// Define crank.
				sd = new b2PolygonShape();
				sd.SetAsBox(7.5 / m_physScale, 30.0 / m_physScale);
				fixtureDef.shape = sd;
				fixtureDef.density = 1.0;
				
				b2RevoluteJointDef rjd = new b2RevoluteJointDef();
				
				b2Body prevBody = ground;
				
				bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(100.0/m_physScale, (360.0-105.0)/m_physScale);
				body = m_world.CreateBody(bd);
				body.CreateFixture(fixtureDef);
				
				rjd.Initialize(prevBody, body, new b2Vec2(100.0/m_physScale, (360.0-75.0)/m_physScale));
				rjd.motorSpeed = 1.0 * -PI;
				rjd.maxMotorTorque = 5000.0;
				rjd.enableMotor = true;
				m_joint1 = m_world.CreateJoint(rjd) as b2RevoluteJoint;
				
				prevBody = body;
				
				// Define follower.
				sd = new b2PolygonShape();
				sd.SetAsBox(7.5 / m_physScale, 60.0 / m_physScale);
				fixtureDef.shape = sd;
				bd.position.Set(100.0/m_physScale, (360.0-195.0)/m_physScale);
				body = m_world.CreateBody(bd);
				body.CreateFixture(fixtureDef);
				
				rjd.Initialize(prevBody, body, new b2Vec2(100.0/m_physScale, (360.0-135.0)/m_physScale));
				rjd.enableMotor = false;
				m_world.CreateJoint(rjd);
				
				prevBody = body;
				
				// Define piston
				sd = new b2PolygonShape();
				sd.SetAsBox(22.5 / m_physScale, 22.5 / m_physScale);
				fixtureDef.shape = sd;
				bd.position.Set(100.0/m_physScale, (360.0-255.0)/m_physScale);
				body = m_world.CreateBody(bd);
				body.CreateFixture(fixtureDef);
				
				rjd.Initialize(prevBody, body, new b2Vec2(100.0/m_physScale, (360.0-255.0)/m_physScale));
				m_world.CreateJoint(rjd);
				
				b2PrismaticJointDef pjd = new b2PrismaticJointDef();
				pjd.Initialize(ground, body, new b2Vec2(100.0/m_physScale, (360.0-255.0)/m_physScale), new b2Vec2(0.0, 1.0));
				
				pjd.maxMotorForce = 500.0;
				pjd.enableMotor = true;
				
				m_joint2 = m_world.CreateJoint(pjd) as b2PrismaticJoint;
				
				// Create a payload
				sd = new b2PolygonShape();
				sd.SetAsBox(22.5 / m_physScale, 22.5 / m_physScale);
				fixtureDef.shape = sd;
				fixtureDef.density = 2.0;
				bd.position.Set(100.0/m_physScale, (360.0-345.0)/m_physScale);
				body = m_world.CreateBody(bd);
				body.CreateFixture(fixtureDef);
			}
			
			
			// 
			// GEARS
			//
			//{
				b2CircleShape circle1 = new b2CircleShape(25 / m_physScale);
				fixtureDef.shape = circle1;
				fixtureDef.density = 5.0;
				
				b2BodyDef bd1 = new b2BodyDef();
				bd1.type = b2Body.b2_dynamicBody;
				bd1.position.Set(200 / m_physScale, 360/2 / m_physScale);
				b2Body body1 = m_world.CreateBody(bd1);
				body1.CreateFixture(fixtureDef);
				
				b2RevoluteJointDef jd1 = new b2RevoluteJointDef();
				jd1.Initialize(ground, body1, bd1.position);
				m_gJoint1 = m_world.CreateJoint(jd1) as b2RevoluteJoint;
				
				b2CircleShape circle2 = new b2CircleShape(50 / m_physScale);
				fixtureDef.shape = circle2;
				fixtureDef.density = 5.0;
				
				b2BodyDef bd2 = new b2BodyDef();
				bd2.type = b2Body.b2_dynamicBody;
				bd2.position.Set(275 / m_physScale, 360/2 / m_physScale);
				b2Body body2 = m_world.CreateBody(bd2);
				body2.CreateFixture(fixtureDef);
				
				b2RevoluteJointDef jd2 = new b2RevoluteJointDef();
				jd2.Initialize(ground, body2, bd2.position);
				m_gJoint2 = m_world.CreateJoint(jd2) as b2RevoluteJoint;
				
				b2PolygonShape box = new b2PolygonShape();
				box.SetAsBox(10 / m_physScale, 100 / m_physScale);
				fixtureDef.shape = box;
				fixtureDef.density = 5.0;
				
				b2BodyDef bd3 = new b2BodyDef();
				bd3.type = b2Body.b2_dynamicBody;
				bd3.position.Set(335 / m_physScale, 360/2 / m_physScale);
				b2Body body3 = m_world.CreateBody(bd3);
				body3.CreateFixture(fixtureDef);
				
				b2PrismaticJointDef jd3 = new b2PrismaticJointDef();
				jd3.Initialize(ground, body3, bd3.position, new b2Vec2(0.0,1.0));
				jd3.lowerTranslation = -25.0 / m_physScale;
				jd3.upperTranslation = 100.0 / m_physScale;
				jd3.enableLimit = true;
				
				m_gJoint3 = m_world.CreateJoint(jd3) as b2PrismaticJoint;
				
				b2GearJointDef jd4 = new b2GearJointDef();
				jd4.bodyA = body1;
				jd4.bodyB = body2;
				jd4.joint1 = m_gJoint1;
				jd4.joint2 = m_gJoint2;
				jd4.ratio = circle2.GetRadius() / circle1.GetRadius();
				m_gJoint4 = m_world.CreateJoint(jd4) as b2GearJoint;
				
				b2GearJointDef jd5 = new b2GearJointDef();
				jd5.bodyA = body2;
				jd5.bodyB = body3;
				jd5.joint1 = m_gJoint2;
				jd5.joint2 = m_gJoint3;
				jd5.ratio = -1.0 / circle2.GetRadius();
				m_gJoint5 = m_world.CreateJoint(jd5) as b2GearJoint;
			//}
			
			
			
			//
			// PULLEY
			//
			//{
				sd = new b2PolygonShape();
				sd.SetAsBox(50 / m_physScale, 20 / m_physScale);
				fixtureDef.shape = sd;
				fixtureDef.density = 5.0;
				
				bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				
				bd.position.Set(480 / m_physScale, 200 / m_physScale);
				body2 = m_world.CreateBody(bd);
				body2.CreateFixture(fixtureDef);
				
				b2PulleyJointDef pulleyDef = new b2PulleyJointDef();
				
				b2Vec2 anchor1 = new b2Vec2(335 / m_physScale, 180 / m_physScale);
				b2Vec2 anchor2 = new b2Vec2(480 / m_physScale, 180 / m_physScale);
				b2Vec2 groundAnchor1 = new b2Vec2(335 / m_physScale, 50 / m_physScale);
				b2Vec2 groundAnchor2 = new b2Vec2(480 / m_physScale, 50 / m_physScale);
				pulleyDef.Initialize(body3, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 2.0);
				
				pulleyDef.maxLengthA = 200 / m_physScale;
				pulleyDef.maxLengthB = 150 / m_physScale;
				
				//m_joint1 = m_world.CreateJoint(as as pulleyDef) b2PulleyJoint;
				m_world.CreateJoint(pulleyDef) as b2PulleyJoint;
				
				
				// Add a circle to weigh down the pulley
				b2CircleShape circ = new b2CircleShape(40 / m_physScale);
				fixtureDef.shape = circ;
				fixtureDef.friction = 0.3;
				fixtureDef.restitution = 0.3;
				fixtureDef.density = 5.0;
				bd.position.Set(485 / m_physScale, 100 / m_physScale);
				body1 = circleBody = m_world.CreateBody(bd);
				body1.CreateFixture(fixtureDef);
			//}
			
			//
			// LINE JOINT
			//
			{
				sd = new b2PolygonShape();
				sd.SetAsBox(7.5 / m_physScale, 30.0 / m_physScale);
				fixtureDef.shape = sd;
				fixtureDef.density = 1.0;
				
				bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				bd.position.Set(500 / m_physScale, 500/2 / m_physScale);
				body = m_world.CreateBody(bd);
				body.CreateFixture(fixtureDef);
				
				b2LineJointDef ljd = new b2LineJointDef();
				ljd.Initialize(ground, body, body.GetPosition(), new b2Vec2(0.4, 0.6));
				
				ljd.lowerTranslation = -1.0;
				ljd.upperTranslation = 1.0;
				ljd.enableLimit = true;
				
				ljd.maxMotorForce = 1.0;
				ljd.motorSpeed = 0.0;
				ljd.enableMotor = true;
				
				m_world.CreateJoint(ljd);
			}
			
			//
			// FRICTION JOINT
			//
			{
				b2FrictionJointDef fjd = new b2FrictionJointDef();
				fjd.Initialize(circleBody, m_world.GetGroundBody(), circleBody.GetPosition());
				fjd.collideConnected = true;
				fjd.maxForce = 200.0;
				m_world.CreateJoint(fjd);
			}
			
			//
			// WELD JOINT
			//
			// Not enabled as Weld joints are not encouraged compared with merging two bodies
			if( false)
			{
				b2WeldJointDef wjd = new b2WeldJointDef();
				wjd.Initialize(circleBody, body, circleBody.GetPosition());
				m_world.CreateJoint(wjd);
			}
		}
		
		
		//===============
		// Member Data 
		//===============
		 b2RevoluteJoint m_joint1 ;
		 b2PrismaticJoint m_joint2 ;
		
		 b2RevoluteJoint m_gJoint1 ;
		 b2RevoluteJoint m_gJoint2 ;
		 b2PrismaticJoint m_gJoint3 ;
		 b2GearJoint m_gJoint4 ;
		 b2GearJoint m_gJoint5 ;
		
	}
	
