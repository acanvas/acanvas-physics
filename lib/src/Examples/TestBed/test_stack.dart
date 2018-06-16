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

part of acanvas_physics;

class TestStack extends Test {
  TestStack(Stopwatch w) : super(w) {
    // Set Text field
    Main.m_aboutText.text = "Stacked Boxes";

    // Add bodies
    /*
			b2PolygonShape sd = new b2PolygonShape();
			b2BodyDef bd = new b2BodyDef();
			bd.type = b2Body.b2_dynamicBody;
			//bd.isBullet = true;
			b2Body b;

			b2FixtureDef fd = new b2FixtureDef();
			fd.density = 1.0;
			fd.friction = 0.5;
			fd.restitution = 0.1;
			fd.shape = sd;
			 * 
			 */

    int i = 0;
    // Create 3 stacks
    for (i = 0; i < 10; i++) {
      b2PolygonShape sd = new b2PolygonShape();
      sd.SetAsBox((10.0) / m_physScale, (10.0) / m_physScale);
      //bd.position.Set((640/2+100+new Random().nextDouble()*0.02 - 0.01) / m_physScale, (360-5-i*25) / m_physScale);
      b2BodyDef bd = new b2BodyDef();
      bd.allowSleep = false;
      bd.bullet = true;
      bd.type = b2Body.b2_dynamicBody;
      bd.position.Set((640 / 2 + 100) / m_physScale, (360 - 5 - i * 25) / m_physScale);

      b2FixtureDef fd = new b2FixtureDef();
      fd.density = 1.0;
      fd.friction = 0.5;
      fd.restitution = 0.1;
      fd.shape = sd;

      b2Body b = m_world.CreateBody(bd);
      b.CreateFixture2(sd, 22.0);
    }

    return;

    for (i = 0; i < 10; i++) {
      sd.SetAsBox((10) / m_physScale, (10) / m_physScale);
      bd.position
          .Set((640 / 2 - 0 + new Random().nextDouble() * 0.02 - 0.01) / m_physScale, (360 - 5 - i * 25) / m_physScale);
      b = m_world.CreateBody(bd);
      b.CreateFixture(fd);
    }
    for (i = 0; i < 10; i++) {
      sd.SetAsBox((10) / m_physScale, (10) / m_physScale);
      bd.position.Set(
          (640 / 2 + 200 + new Random().nextDouble() * 0.02 - 0.01) / m_physScale, (360 - 5 - i * 25) / m_physScale);
      b = m_world.CreateBody(bd);
      b.CreateFixture(fd);
    }
    // Create ramp
    List vxs = [new b2Vec2(0.0, 0.0), new b2Vec2(0.0, -100 / m_physScale), new b2Vec2(200 / m_physScale, 0.0)];
    sd.SetAsVector(vxs, vxs.length);
    fd.density = 0.0;
    bd.type = b2Body.b2_staticBody;
    bd.userData = "ramp";
    bd.position.Set(0.0, 360 / m_physScale);
    b = m_world.CreateBody(bd);
    b.CreateFixture(fd);

    // Create ball
    b2CircleShape cd = new b2CircleShape();
    cd.m_radius = 40 / m_physScale;
    fd.density = 2.0;
    fd.restitution = 0.2;
    fd.friction = 0.5;
    fd.shape = cd;
    bd.type = b2Body.b2_dynamicBody;
    bd.userData = "ball";
    bd.position.Set(50 / m_physScale, 100 / m_physScale);
    b = m_world.CreateBody(bd);
    b.CreateFixture(fd);
  }

  //===============
  // Member Data
  //===============
}
