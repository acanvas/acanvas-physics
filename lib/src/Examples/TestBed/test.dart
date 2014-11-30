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

class Test {


  //===============
  // Member Data
  //===============
  b2World m_world;
  b2Body m_bomb;
  b2MouseJoint m_mouseJoint;
  int m_velocityIterations = 10;
  int m_positionIterations = 10;
  double m_timeStep = 1.0 / 30.0;
  double m_physScale = 30.0;
  // world mouse position
  static double mouseXWorldPhys = 0.0;
  static double mouseYWorldPhys = 0.0;
  static double mouseXWorld = 0.0;
  static double mouseYWorld = 0.0;
  // Sprite to draw in to
  Sprite m_sprite;
  Stopwatch watch;


  Test(this.watch) {

    m_sprite = Main.m_sprite;

    b2AABB worldAABB = new b2AABB();
    worldAABB.lowerBound.Set(-1000.0, -1000.0);
    worldAABB.upperBound.Set(1000.0, 1000.0);

    // Define the gravity vector
    b2Vec2 gravity = new b2Vec2(0.0, 10.0);

    // Allow bodies to sleep
    bool doSleep = true;

    // Construct a world object
    m_world = new b2World(gravity, doSleep);
    //m_world.SetBroadPhase(new b2BroadPhase(worldAABB));
    m_world.SetWarmStarting(true);
    // set debug draw
    b2DebugDraw dbgDraw = new b2DebugDraw();
    //Sprite dbgSprite = new Sprite();
    //m_sprite.addChild(dbgSprite);
    dbgDraw.SetSprite(m_sprite);
    dbgDraw.SetDrawScale(30.0);
    dbgDraw.SetFillAlpha(0.3);
    dbgDraw.SetLineThickness(1.0);
    dbgDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
    m_world.SetDebugDraw(dbgDraw);

    // Create border of boxes
    b2PolygonShape wall = new b2PolygonShape();
    b2BodyDef wallBd = new b2BodyDef();
    wallBd.type = b2Body.b2_staticBody;
    b2Body wallB;

    // Left
    wallBd.position.Set(-95.0 / m_physScale, 360 / m_physScale / 2);
    wall.SetAsBox(100 / m_physScale, 400 / m_physScale / 2);
    wallB = m_world.CreateBody(wallBd);
    wallB.CreateFixture2(wall);
    // Right
    wallBd.position.Set((640 + 95) / m_physScale, 360 / m_physScale / 2);
    wallB = m_world.CreateBody(wallBd);
    wallB.CreateFixture2(wall);
    // Top
    wallBd.position.Set(640 / m_physScale / 2, -95 / m_physScale);
    wall.SetAsBox(680 / m_physScale / 2, 100 / m_physScale);
    wallB = m_world.CreateBody(wallBd);
    wallB.CreateFixture2(wall);
    // Bottom
    wallBd.position.Set(640 / m_physScale / 2, (360 + 95) / m_physScale);
    wallB = m_world.CreateBody(wallBd);
    wallB.CreateFixture2(wall);
  }


  void Update() {
    // Update mouse joint
    UpdateMouseWorld();
    MouseDestroy();
    MouseDrag();

    // Update physics
    int physStart = watch.elapsedMilliseconds;
    m_world.Step(m_timeStep, m_velocityIterations, m_positionIterations);
    m_world.ClearForces();

    Main.m_fpsCounter.updatePhys(physStart);

    // Render
    m_world.DrawDebugData();
    // joints
    /*for (b2Joint jj = m_world.m_jointList; jj; jj = jj.m_next){
				//DrawJoint(jj);
			}
			// bodies
			for (b2Body bb = m_world.m_bodyList; bb; bb = bb.m_next){
				for (b2Shape s = bb.GetShapeList(); s != null; s = s.GetNext()){
					//DrawShape(s);
				}
			}*/

    //DrawPairs();
    //DrawBounds();

  }



  //===============
  // Update mouseWorld
  //===============
  void UpdateMouseWorld() {
    mouseXWorldPhys = (Input.mouseX) / m_physScale;
    mouseYWorldPhys = (Input.mouseY) / m_physScale;

    mouseXWorld = (Input.mouseX);
    mouseYWorld = (Input.mouseY);
  }



  //===============
  // Mouse Drag
  //===============
  void MouseDrag() {
    // mouse press
    if (Input.mouseDown && m_mouseJoint == null) {

      b2Body body = GetBodyAtMouse();

      if (body != null ) {
        b2MouseJointDef md = new b2MouseJointDef();
        md.bodyA = m_world.GetGroundBody();
        md.bodyB = body;
        md.target.Set(mouseXWorldPhys, mouseYWorldPhys);
        md.collideConnected = true;
        md.maxForce = 300.0 * body.GetMass();
        m_mouseJoint = m_world.CreateJoint(md) as b2MouseJoint;
        body.SetAwake(true);
      }
    }


    // mouse release
    if (!Input.mouseDown) {
      if (m_mouseJoint != null) {
        m_world.DestroyJoint(m_mouseJoint);
        m_mouseJoint = null;
      }
    }


    // mouse move
    if (m_mouseJoint != null) {
      b2Vec2 p2 = new b2Vec2(mouseXWorldPhys, mouseYWorldPhys);
      m_mouseJoint.SetTarget(p2);
    }
  }



  //===============
  // Mouse Destroy
  //===============
  void MouseDestroy() {
    // mouse press
    if (!Input.mouseDown && Input.isKeyPressed(68/*D*/)) {

      b2Body body = GetBodyAtMouse(true);

      if (body != null ) {
        m_world.DestroyBody(body);
        return;
      }
    }
  }



  //===============
  // GetBodyAtMouse
  //===============
  b2Vec2 mousePVec = new b2Vec2();
  b2Body GetBodyAtMouse([bool includeStatic = false]) {
    // Make a small box.
    mousePVec.Set(mouseXWorldPhys, mouseYWorldPhys);
    b2AABB aabb = new b2AABB();
    aabb.lowerBound.Set(mouseXWorldPhys - 0.001, mouseYWorldPhys - 0.001);
    aabb.upperBound.Set(mouseXWorldPhys + 0.001, mouseYWorldPhys + 0.001);
    b2Body body = null;
    b2Fixture fixture;

    // Query the world for overlapping shapes.
    bool GetBodyCallback(b2Fixture fixture) {
      b2Shape shape = fixture.GetShape();
      if (fixture.GetBody().GetType() != b2Body.b2_staticBody || includeStatic) {
        bool inside = shape.TestPoint(fixture.GetBody().GetTransform(), mousePVec);
        if (inside ) {
          body = fixture.GetBody();
          return false;
        }
      }
      return true;
    }
    m_world.QueryAABB(GetBodyCallback, aabb);
    return body;
  }



  //===============
  // Draw Bounds
  //===============
  /*
	 DrawBounds(){
			b2AABB b = new b2AABB();
			
			b2BroadPhase bp = m_world.m_broadPhase;
			b2Vec2 invQ = new b2Vec2();
			invQ.Set(1.0 / bp.m_quantizationFactor.x, 1.0 / bp.m_quantizationFactor.y);
			
			for (int i = 0; i < b2Settings.b2_maxProxies; ++i)
			{
				b2Proxy p = bp.m_proxyPool[ i ];
				if (p.IsValid() == false)
				{
					continue;
				}
				
				b.minVertex.x = bp.m_worldAABB.minVertex.x + invQ.x * bp.m_bounds[0][p.lowerBounds[0]].value;
				b.minVertex.y = bp.m_worldAABB.minVertex.y + invQ.y * bp.m_bounds[1][p.lowerBounds[1]].value;
				b.maxVertex.x = bp.m_worldAABB.minVertex.x + invQ.x * bp.m_bounds[0][p.upperBounds[0]].value;
				b.maxVertex.y = bp.m_worldAABB.minVertex.y + invQ.y * bp.m_bounds[1][p.upperBounds[1]].value;
				
				m_sprite.graphics.lineStyle(1,0xff22ff,1);
				m_sprite.graphics.moveTo(b.minVertex.x * m_physScale, b.minVertex.y * m_physScale);
				m_sprite.graphics.lineTo(b.maxVertex.x * m_physScale, b.minVertex.y * m_physScale);
				m_sprite.graphics.lineTo(b.maxVertex.x * m_physScale, b.maxVertex.y * m_physScale);
				m_sprite.graphics.lineTo(b.minVertex.x * m_physScale, b.maxVertex.y * m_physScale);
				m_sprite.graphics.lineTo(b.minVertex.x * m_physScale, b.minVertex.y * m_physScale);
			}
		}
		
		
		//===============
		// Draw Pairs
		//===============
		  void DrawPairs(){
			
			b2BroadPhase bp = m_world.m_broadPhase;
			b2Vec2 invQ = new b2Vec2();
			invQ.Set(1.0 / bp.m_quantizationFactor.x, 1.0 / bp.m_quantizationFactor.y);
			
			for (int i = 0; i < b2Pair.b2_tableCapacity; ++i)
			{
				int index = bp.m_pairManager.m_hashTable[i];
				while (index != b2Pair.b2_nullPair)
				{
					b2Pair pair = bp.m_pairManager.m_pairs[ index ];
					b2Proxy p1 = bp.m_proxyPool[ pair.proxyId1 ];
					b2Proxy p2 = bp.m_proxyPool[ pair.proxyId2 ];
					
					b2AABB b1 = new b2AABB();
					b2AABB b2 = new b2AABB();
					b1.minVertex.x = bp.m_worldAABB.minVertex.x + invQ.x * bp.m_bounds[0][p1.lowerBounds[0]].value;
					b1.minVertex.y = bp.m_worldAABB.minVertex.y + invQ.y * bp.m_bounds[1][p1.lowerBounds[1]].value;
					b1.maxVertex.x = bp.m_worldAABB.minVertex.x + invQ.x * bp.m_bounds[0][p1.upperBounds[0]].value;
					b1.maxVertex.y = bp.m_worldAABB.minVertex.y + invQ.y * bp.m_bounds[1][p1.upperBounds[1]].value;
					b2.minVertex.x = bp.m_worldAABB.minVertex.x + invQ.x * bp.m_bounds[0][p2.lowerBounds[0]].value;
					b2.minVertex.y = bp.m_worldAABB.minVertex.y + invQ.y * bp.m_bounds[1][p2.lowerBounds[1]].value;
					b2.maxVertex.x = bp.m_worldAABB.minVertex.x + invQ.x * bp.m_bounds[0][p2.upperBounds[0]].value;
					b2.maxVertex.y = bp.m_worldAABB.minVertex.y + invQ.y * bp.m_bounds[1][p2.upperBounds[1]].value;
					
					b2Vec2 x1 = b2Math.MulFV(0.5, b2Math.AddVV(b1.minVertex, b1.maxVertex) );
					b2Vec2 x2 = b2Math.MulFV(0.5, b2Math.AddVV(b2.minVertex, b2.maxVertex) );
					
					m_sprite.graphics.lineStyle(1,0xff2222,1);
					m_sprite.graphics.moveTo(x1.x * m_physScale, x1.y * m_physScale);
					m_sprite.graphics.lineTo(x2.x * m_physScale, x2.y * m_physScale);
					
					index = pair.next;
				}
			}
		}
		
		//===============
		// Draw Contacts
		//===============
		  void DrawContacts(){
			for (b2Contact c = m_world.m_contactList; c; c = c.m_next)
			{
				List ms = c.GetManifolds();
				for (int i = 0; i < c.GetManifoldCount(); ++i)
				{
					b2Manifold m = ms[ i ];
					//this.graphics.lineStyle(3,0x11CCff,0.7);
					
					for (int j = 0; j < m.pointCount; ++j)
					{	
						m_sprite.graphics.lineStyle(m.points[j].normalImpulse,0x11CCff,0.7);
						b2Vec2 v = m.points[j].position;
						m_sprite.graphics.moveTo(v.x * m_physScale, v.y * m_physScale);
						m_sprite.graphics.lineTo(v.x * m_physScale, v.y * m_physScale);
						
					}
				}
			}
		}
		
		
		//===============
		// Draw Shape 
		//===============
		  void DrawShape(b2Shape shape){
			switch (shape.m_type)
			{
			case b2Shape.e_circleShape:
				{
					b2CircleShape circle = shape as b2CircleShape;
					b2Vec2 pos = circle.m_position;
					double r = circle.m_radius;
					double k_segments = 16.0;
					double k_increment = 2.0 * PI / k_segments;
					m_sprite.graphics.lineStyle(1,0xffffff,1);
					m_sprite.graphics.moveTo((pos.x + r) * m_physScale, (pos.y) * m_physScale);
					double theta = 0.0;
					
					for (int i = 0; i < k_segments; ++i)
					{
						b2Vec2 d = new b2Vec2(r * /*Math.*/cos(theta), r * /*Math.*/sin(theta));
						b2Vec2 v = b2Math.AddVV(pos , d);
						m_sprite.graphics.lineTo((v.x) * m_physScale, (v.y) * m_physScale);
						theta += k_increment;
					}
					m_sprite.graphics.lineTo((pos.x + r) * m_physScale, (pos.y) * m_physScale);
					
					m_sprite.graphics.moveTo((pos.x) * m_physScale, (pos.y) * m_physScale);
					b2Vec2 ax = circle.m_R.col1;
					b2Vec2 pos2 = new b2Vec2(pos.x + r * ax.x, pos.y + r * ax.y);
					m_sprite.graphics.lineTo((pos2.x) * m_physScale, (pos2.y) * m_physScale);
				}
				break;
			case b2Shape.e_polyShape:
				{
					b2PolyShape poly = shape as b2PolyShape;
					b2Vec2 tV = b2Math.AddVV(poly.m_position, b2Math.b2MulMV(poly.m_R, poly.m_vertices[i]));
					m_sprite.graphics.lineStyle(1,0xffffff,1);
					m_sprite.graphics.moveTo(tV.x * m_physScale, tV.y * m_physScale);
					
					for (i = 0; i < poly.m_vertexCount; ++i)
					{
						v = b2Math.AddVV(poly.m_position, b2Math.b2MulMV(poly.m_R, poly.m_vertices[i]));
						m_sprite.graphics.lineTo(v.x * m_physScale, v.y * m_physScale);
					}
					m_sprite.graphics.lineTo(tV.x * m_physScale, tV.y * m_physScale);
				}
				break;
			}
		}
		
		
		//===============
		// Draw Joint 
		//===============
		  void DrawJoint(b2Joint joint)
		{
			b2Body b1 = joint.m_body1;
			b2Body b2 = joint.m_body2;
			
			b2Vec2 x1 = b1.m_position;
			b2Vec2 x2 = b2.m_position;
			b2Vec2 p1 = joint.GetAnchor1();
			b2Vec2 p2 = joint.GetAnchor2();
			
			m_sprite.graphics.lineStyle(1,0x44aaff,1/1);
			
			switch (joint.m_type)
			{
			case b2Joint.e_distanceJoint:
			case b2Joint.e_mouseJoint:
				m_sprite.graphics.moveTo(p1.x * m_physScale, p1.y * m_physScale);
				m_sprite.graphics.lineTo(p2.x * m_physScale, p2.y * m_physScale);
				break;
				
			case b2Joint.e_pulleyJoint:
				b2PulleyJoint pulley = joint as b2PulleyJoint;
				b2Vec2 s1 = pulley.GetGroundPoint1();
				b2Vec2 s2 = pulley.GetGroundPoint2();
				m_sprite.graphics.moveTo(s1.x * m_physScale, s1.y * m_physScale);
				m_sprite.graphics.lineTo(p1.x * m_physScale, p1.y * m_physScale);
				m_sprite.graphics.moveTo(s2.x * m_physScale, s2.y * m_physScale);
				m_sprite.graphics.lineTo(p2.x * m_physScale, p2.y * m_physScale);
				break;
				
			default:
				if (b1 == m_world.m_groundBody){
					m_sprite.graphics.moveTo(p1.x * m_physScale, p1.y * m_physScale);
					m_sprite.graphics.lineTo(x2.x * m_physScale, x2.y * m_physScale);
				}
				else if (b2 == m_world.m_groundBody){
					m_sprite.graphics.moveTo(p1.x * m_physScale, p1.y * m_physScale);
					m_sprite.graphics.lineTo(x1.x * m_physScale, x1.y * m_physScale);
				}
				else{
					m_sprite.graphics.moveTo(x1.x * m_physScale, x1.y * m_physScale);
					m_sprite.graphics.lineTo(p1.x * m_physScale, p1.y * m_physScale);
					m_sprite.graphics.lineTo(x2.x * m_physScale, x2.y * m_physScale);
					m_sprite.graphics.lineTo(p2.x * m_physScale, p2.y * m_physScale);
				}
			}
		}*/
}
