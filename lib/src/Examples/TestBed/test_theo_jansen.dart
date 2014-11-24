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
	
	 class TestTheoJansen extends Test{
	 TestTheoJansen(Stopwatch w) : super(w) {
			
			// Set Text field
			Main.m_aboutText.text = "Theo Jansen Walker";
			
			// scale walker by variable to easily change size
			tScale = m_physScale * 2;
			
			// Set position in world space
			m_offset.Set(120.0/m_physScale, 250/m_physScale);
			m_motorSpeed = -2.0;
			m_motorOn = true;
			b2Vec2 pivot = new b2Vec2(0.0, -24.0/tScale);
			
			b2PolygonShape pd;
			b2CircleShape cd;
			b2FixtureDef fd;
			b2BodyDef bd;
			b2Body body;
			
			for (int i = 0; i < 40; ++i)
			{
				cd = new b2CircleShape(7.5/tScale);
				
				bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				// Position in world space
				bd.position.Set((new Random().nextDouble() * 620 + 10)/m_physScale, 350/m_physScale);
				
				body = m_world.CreateBody(bd);
				body.CreateFixture2(cd, 1.0);
			}
			
			{
				pd = new b2PolygonShape();
				pd.SetAsBox(75 / tScale, 30 / tScale);
				fd = new b2FixtureDef();
				fd.shape = pd;
				fd.density = 1.0;
				fd.filter.groupIndex = -1;
				bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				//bd.position = pivot + m_offset;
				bd.position = b2Math.AddVV(pivot, m_offset);
				m_chassis = m_world.CreateBody(bd);
				m_chassis.CreateFixture(fd);
			}
			
			{
				cd = new b2CircleShape(48 / tScale);
				fd = new b2FixtureDef();
				fd.shape = cd;
				fd.density = 1.0;
				fd.filter.groupIndex = -1;
				bd = new b2BodyDef();
				bd.type = b2Body.b2_dynamicBody;
				//bd.position = pivot + m_offset;
				bd.position = b2Math.AddVV(pivot, m_offset);
				m_wheel = m_world.CreateBody(bd);
				m_wheel.CreateFixture(fd);
			}
			
			{
				b2RevoluteJointDef jd = new b2RevoluteJointDef();
				b2Vec2 po = pivot.Copy();
				po.Add(m_offset);
				jd.Initialize(m_wheel, m_chassis, po);
				jd.collideConnected = false;
				jd.motorSpeed = m_motorSpeed;
				jd.maxMotorTorque = 400.0;
				jd.enableMotor = m_motorOn;
				m_motorJoint = m_world.CreateJoint(jd) as b2RevoluteJoint;
			}
			
			b2Vec2 wheelAnchor;
			
			//wheelAnchor = pivot + b2Vec2(0.0f, -0.8);
			wheelAnchor = new b2Vec2(0.0, 24.0/tScale);
			wheelAnchor.Add(pivot);
			
			CreateLeg(-1.0, wheelAnchor);
			CreateLeg(1.0, wheelAnchor);
			
			m_wheel.SetPositionAndAngle(m_wheel.GetPosition(), 120.0 * PI / 180.0);
			CreateLeg(-1.0, wheelAnchor);
			CreateLeg(1.0, wheelAnchor);
			
			m_wheel.SetPositionAndAngle(m_wheel.GetPosition(), -120.0 * PI / 180.0);
			CreateLeg(-1.0, wheelAnchor);
			CreateLeg(1.0, wheelAnchor);
			
		}
		
		
		
		  void CreateLeg(double s,b2Vec2 wheelAnchor){
			
			b2Vec2 p1 = new b2Vec2(162 * s/tScale, 183/tScale);
			b2Vec2 p2 = new b2Vec2(216 * s/tScale, 36 /tScale);
			b2Vec2 p3 = new b2Vec2(129 * s/tScale, 57 /tScale);
			b2Vec2 p4 = new b2Vec2( 93 * s/tScale, -24  /tScale);
			b2Vec2 p5 = new b2Vec2(180 * s/tScale, -45  /tScale);
			b2Vec2 p6 = new b2Vec2( 75 * s/tScale, -111 /tScale);
			
			//b2PolygonDef sd1, sd2;
			b2PolygonShape sd1 = new b2PolygonShape();
			b2PolygonShape sd2 = new b2PolygonShape();
			b2FixtureDef fd1 = new b2FixtureDef();
			b2FixtureDef fd2 = new b2FixtureDef();
			fd1.shape = sd1;
			fd2.shape = sd2;
			fd1.filter.groupIndex = -1;
			fd2.filter.groupIndex = -1;
			fd1.density = 1.0;
			fd2.density = 1.0;
			
			if (s > 0.0)
			{
				sd1.SetAsVector([p3, p2, p1]);
				sd2.SetAsVector([
					b2Math.SubtractVV(p6, p4),
					b2Math.SubtractVV(p5, p4),
					new b2Vec2()
					]);
			}
			else
			{
				sd1.SetAsVector([p2, p3, p1]);
				sd2.SetAsVector([
					b2Math.SubtractVV(p5, p4),
					b2Math.SubtractVV(p6, p4),
					new b2Vec2()
					]);
			}
			
			//b2BodyDef bd1, bd2;
			b2BodyDef bd1 = new b2BodyDef();
			bd1.type = b2Body.b2_dynamicBody;
			b2BodyDef bd2 = new b2BodyDef();
			bd2.type = b2Body.b2_dynamicBody;
			bd1.position.SetV(m_offset);
			bd2.position = b2Math.AddVV(p4, m_offset);
			
			bd1.angularDamping = 10.0;
			bd2.angularDamping = 10.0;
			
			b2Body body1 = m_world.CreateBody(bd1);
			b2Body body2 = m_world.CreateBody(bd2);
			
			body1.CreateFixture(fd1);
			body2.CreateFixture(fd2);
			
			b2DistanceJointDef djd = new b2DistanceJointDef();
			
			// Using a soft distance constraint can reduce some jitter.
			// It also makes the structure seem a bit more fluid by
			// acting like a suspension system.
			djd.dampingRatio = 0.5;
			djd.frequencyHz = 10.0;
			
			djd.Initialize(body1, body2, b2Math.AddVV(p2, m_offset), b2Math.AddVV(p5, m_offset));
			m_world.CreateJoint(djd);
			
			djd.Initialize(body1, body2, b2Math.AddVV(p3, m_offset), b2Math.AddVV(p4, m_offset));
			m_world.CreateJoint(djd);
			
			djd.Initialize(body1, m_wheel, b2Math.AddVV(p3, m_offset), b2Math.AddVV(wheelAnchor, m_offset));
			m_world.CreateJoint(djd);
			
			djd.Initialize(body2, m_wheel, b2Math.AddVV(p6, m_offset), b2Math.AddVV(wheelAnchor, m_offset));
			m_world.CreateJoint(djd);
			
			b2RevoluteJointDef rjd = new b2RevoluteJointDef();
			
			rjd.Initialize(body2, m_chassis, b2Math.AddVV(p4, m_offset));
			m_world.CreateJoint(rjd);
			
		}
		
		
		
		 @override 
		 void Update(){
			
			//case 'a':
			if (Input.isKeyPressed(65)){ // A
				m_chassis.SetAwake(true);
				m_motorJoint.SetMotorSpeed(-m_motorSpeed);
			}
			//case 's':
			if (Input.isKeyPressed(83)){ // S
				m_chassis.SetAwake(true);
				m_motorJoint.SetMotorSpeed(0.0);
			}
			//case 'd':
			if (Input.isKeyPressed(68)){ // D
				m_chassis.SetAwake(true);
				m_motorJoint.SetMotorSpeed(m_motorSpeed);
			}
			//case 'm':
			if (Input.isKeyPressed(77)){ // M
				m_chassis.SetAwake(true);
				m_motorJoint.EnableMotor(!m_motorJoint.IsMotorEnabled());
			}
			
			// Finally update super class
			super.Update();
		}
		
		
		//===============
		// Member Data 
		//===============
		 double tScale;
		
		 b2Vec2 m_offset = new b2Vec2();
		 b2Body m_chassis;
		 b2Body m_wheel;
		 b2RevoluteJoint m_motorJoint;
		 bool m_motorOn = true;
		 double m_motorSpeed;
		
	}
	
