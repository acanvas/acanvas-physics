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





/**
* The world class manages all physics entities, dynamic simulation,
* and asynchronous queries. 
*/
class b2World {

  // Construct a world object.
  /**
	* @param gravity the world gravity vector.
	* @param doSleep improve performance by not simulating inactive bodies.
	*/
  b2World(b2Vec2 gravity, bool doSleep) {

    m_destructionListener = null;
    m_debugDraw = null;

    m_bodyList = null;
    m_contactList = null;
    m_jointList = null;
    m_controllerList = null;

    m_bodyCount = 0;
    m_contactCount = 0;
    m_jointCount = 0;
    m_controllerCount = 0;

    m_warmStarting = true;
    m_continuousPhysics = true;

    m_allowSleep = doSleep;
    m_gravity = gravity;

    m_inv_dt0 = 0.0;

    m_contactManager.m_world = this;

    b2BodyDef bd = new b2BodyDef();
    m_groundBody = CreateBody(bd);
  }

  /**
	* Destruct the world. All physics entities are destroyed and all heap memory is released.
	*/
  //~b2World();

  /**
	* Register a destruction listener.
	*/
  void SetDestructionListener(b2DestructionListener listener) {
    m_destructionListener = listener;
  }

  /**
	* Register a contact filter to provide specific control over collision.
	* Otherwise the default filter is used (b2_defaultFilter).
	*/
  void SetContactFilter(b2ContactFilter filter) {
    m_contactManager.m_contactFilter = filter;
  }

  /**
	* Register a contact event listener
	*/
  void SetContactListener(b2ContactListener listener) {
    m_contactManager.m_contactListener = listener;
  }

  /**
	* Register a routine for debug drawing. The debug draw functions are called
	* inside the b2World::Step method, so make sure your renderer is ready to
	* consume draw commands when you call Step().
	*/
  void SetDebugDraw(b2DebugDraw debugDraw) {
    m_debugDraw = debugDraw;
  }

  /**
	 * Use the given object as a broadphase.
	 * The old broadphase will not be cleanly emptied.
	 * @warning It is not recommended you call this except immediately after constructing the world.
	 * @warning This function is locked during callbacks.
	 */
  void SetBroadPhase(IBroadPhase broadPhase) {
    IBroadPhase oldBroadPhase = m_contactManager.m_broadPhase;
    m_contactManager.m_broadPhase = broadPhase;
    for (b2Body b = m_bodyList; b; b = b.m_next) {
      for (b2Fixture f = b.m_fixtureList; f; f = f.m_next) {
        f.m_proxy = broadPhase.CreateProxy(oldBroadPhase.GetFatAABB(f.m_proxy), f);
      }
    }
  }

  /**
	* Perform validation of internal data structures.
	*/
  void Validate() {
    m_contactManager.m_broadPhase.Validate();
  }

  /**
	* Get the doubleber of broad-phase proxies.
	*/
  int GetProxyCount() {
    return m_contactManager.m_broadPhase.GetProxyCount();
  }

  /**
	* Create a rigid body given a definition. No reference to the definition
	* is retained.
	* @warning This function is locked during callbacks.
	*/
  b2Body CreateBody(b2BodyDef def) {

    //b2Settings.b2Assert(m_lock == false);
    if (IsLocked() == true) {
      return null;
    }

    //void* mem = m_blockAllocator.Allocate(sizeof(b2Body));
    b2Body b = new b2Body(def, this);

    // Add to world doubly linked list.
    b.m_prev = null;
    b.m_next = m_bodyList;
    if (m_bodyList != null) {
      m_bodyList.m_prev = b;
    }
    m_bodyList = b;
    ++m_bodyCount;

    return b;

  }

  /**
	* Destroy a rigid body given a definition. No reference to the definition
	* is retained. This function is locked during callbacks.
	* @warning This automatically deletes all associated shapes and joints.
	* @warning This function is locked during callbacks.
	*/
  void DestroyBody(b2Body b) {

    //b2Settings.b2Assert(m_bodyCount > 0);
    //b2Settings.b2Assert(m_lock == false);
    if (IsLocked() == true) {
      return;
    }

    // Delete the attached joints.
    b2JointEdge jn = b.m_jointList;
    while (jn != null) {
      b2JointEdge jn0 = jn;
      jn = jn.next;

      if (m_destructionListener != null) {
        m_destructionListener.SayGoodbyeJoint(jn0.joint);
      }

      DestroyJoint(jn0.joint);
    }

    // Detach controllers attached to this body
    b2ControllerEdge coe = b.m_controllerList;
    while (coe != null) {
      b2ControllerEdge coe0 = coe;
      coe = coe.nextController;
      coe0.controller.RemoveBody(b);
    }

    // Delete the attached contacts.
    b2ContactEdge ce = b.m_contactList;
    while (ce != null) {
      b2ContactEdge ce0 = ce;
      ce = ce.next;
      m_contactManager.Destroy(ce0.contact);
    }
    b.m_contactList = null;

    // Delete the attached fixtures. This destroys broad-phase
    // proxies.
    b2Fixture f = b.m_fixtureList;
    while (f != null) {
      b2Fixture f0 = f;
      f = f.m_next;

      if (m_destructionListener != null) {
        m_destructionListener.SayGoodbyeFixture(f0);
      }

      f0.DestroyProxy(m_contactManager.m_broadPhase);
      f0.Destroy();
      //f0->~b2Fixture();
      //m_blockAllocator.Free(f0, sizeof(b2Fixture));

    }
    b.m_fixtureList = null;
    b.m_fixtureCount = 0;

    // Remove world body list.
    if (b.m_prev != null) {
      b.m_prev.m_next = b.m_next;
    }

    if (b.m_next != null) {
      b.m_next.m_prev = b.m_prev;
    }

    if (b == m_bodyList) {
      m_bodyList = b.m_next;
    }

    --m_bodyCount;
    //b->~b2Body();
    //m_blockAllocator.Free(b, sizeof(b2Body));

  }

  /**
	* Create a joint to constrain bodies together. No reference to the definition
	* is retained. This may cause the connected bodies to cease colliding.
	* @warning This function is locked during callbacks.
	*/
  b2Joint CreateJoint(b2JointDef def) {

    //b2Settings.b2Assert(m_lock == false);

    b2Joint j = b2Joint.Create(def, null);

    // Connect to the world list.
    j.m_prev = null;
    j.m_next = m_jointList;
    if (m_jointList != null) {
      m_jointList.m_prev = j;
    }
    m_jointList = j;
    ++m_jointCount;

    // Connect to the bodies' doubly linked lists.
    j.m_edgeA.joint = j;
    j.m_edgeA.other = j.m_bodyB;
    j.m_edgeA.prev = null;
    j.m_edgeA.next = j.m_bodyA.m_jointList;
    if (j.m_bodyA.m_jointList != null) j.m_bodyA.m_jointList.prev = j.m_edgeA;
    j.m_bodyA.m_jointList = j.m_edgeA;

    j.m_edgeB.joint = j;
    j.m_edgeB.other = j.m_bodyA;
    j.m_edgeB.prev = null;
    j.m_edgeB.next = j.m_bodyB.m_jointList;
    if (j.m_bodyB.m_jointList != null) j.m_bodyB.m_jointList.prev = j.m_edgeB;
    j.m_bodyB.m_jointList = j.m_edgeB;

    b2Body bodyA = def.bodyA;
    b2Body bodyB = def.bodyB;

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (def.collideConnected == false) {
      b2ContactEdge edge = bodyB.GetContactList();
      while (edge != null) {
        if (edge.other == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          edge.contact.FlagForFiltering();
        }

        edge = edge.next;
      }
    }

    // Note: creating a joint doesn't wake the bodies.

    return j;

  }

  /**
	* Destroy a joint. This may cause the connected bodies to begin colliding.
	* @warning This function is locked during callbacks.
	*/
  void DestroyJoint(b2Joint j) {

    //b2Settings.b2Assert(m_lock == false);

    bool collideConnected = j.m_collideConnected;

    // Remove from the doubly linked list.
    if (j.m_prev != null) {
      j.m_prev.m_next = j.m_next;
    }

    if (j.m_next != null) {
      j.m_next.m_prev = j.m_prev;
    }

    if (j == m_jointList) {
      m_jointList = j.m_next;
    }

    // Disconnect from island graph.
    b2Body bodyA = j.m_bodyA;
    b2Body bodyB = j.m_bodyB;

    // Wake up connected bodies.
    bodyA.SetAwake(true);
    bodyB.SetAwake(true);

    // Remove from body 1.
    if (j.m_edgeA.prev != null) {
      j.m_edgeA.prev.next = j.m_edgeA.next;
    }

    if (j.m_edgeA.next != null) {
      j.m_edgeA.next.prev = j.m_edgeA.prev;
    }

    if (j.m_edgeA == bodyA.m_jointList) {
      bodyA.m_jointList = j.m_edgeA.next;
    }

    j.m_edgeA.prev = null;
    j.m_edgeA.next = null;

    // Remove from body 2
    if (j.m_edgeB.prev != null) {
      j.m_edgeB.prev.next = j.m_edgeB.next;
    }

    if (j.m_edgeB.next != null) {
      j.m_edgeB.next.prev = j.m_edgeB.prev;
    }

    if (j.m_edgeB == bodyB.m_jointList) {
      bodyB.m_jointList = j.m_edgeB.next;
    }

    j.m_edgeB.prev = null;
    j.m_edgeB.next = null;

    b2Joint.Destroy(j, null);

    //b2Settings.b2Assert(m_jointCount > 0);
    --m_jointCount;

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (collideConnected == false) {
      b2ContactEdge edge = bodyB.GetContactList();
      while (edge != null) {
        if (edge.other == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          edge.contact.FlagForFiltering();
        }

        edge = edge.next;
      }
    }

  }

  /**
	 * Add a controller to the world list
	 */
  b2Controller AddController(b2Controller c) {
    c.m_next = m_controllerList;
    c.m_prev = null;
    m_controllerList = c;

    c.m_world = this;

    m_controllerCount++;

    return c;
  }

  void RemoveController(b2Controller c) {
    //TODO: Remove bodies from controller
    if (c.m_prev != null) c.m_prev.m_next = c.m_next;
    if (c.m_next != null) c.m_next.m_prev = c.m_prev;
    if (m_controllerList == c) m_controllerList = c.m_next;

    m_controllerCount--;
  }

  b2Controller CreateController(b2Controller controller) {
    if (controller.m_world != this) throw new ArgumentError("Controller can only be a member of one world");

    controller.m_next = m_controllerList;
    controller.m_prev = null;
    if (m_controllerList != null) m_controllerList.m_prev = controller;
    m_controllerList = controller;
    ++m_controllerCount;

    controller.m_world = this;

    return controller;
  }

  void DestroyController(b2Controller controller) {
    //b2Settings.b2Assert(m_controllerCount > 0);
    controller.Clear();
    if (controller.m_next != null) controller.m_next.m_prev = controller.m_prev;
    if (controller.m_prev != null) controller.m_prev.m_next = controller.m_next;
    if (controller == m_controllerList) m_controllerList = controller.m_next;
    --m_controllerCount;
  }

  /**
	* Enable/disable warm starting. For testing.
	*/
  void SetWarmStarting(bool flag) {
    m_warmStarting = flag;
  }

  /**
	* Enable/disable continuous physics. For testing.
	*/
  void SetContinuousPhysics(bool flag) {
    m_continuousPhysics = flag;
  }

  /**
	* Get the doubleber of bodies.
	*/
  int GetBodyCount() {
    return m_bodyCount;
  }

  /**
	* Get the doubleber of joints.
	*/
  int GetJointCount() {
    return m_jointCount;
  }

  /**
	* Get the doubleber of contacts (each may have 0 or more contact points).
	*/
  int GetContactCount() {
    return m_contactCount;
  }

  /**
	* Change the global gravity vector.
	*/
  void SetGravity(b2Vec2 gravity) {
    m_gravity = gravity;
  }

  /**
	* Get the global gravity vector.
	*/
  b2Vec2 GetGravity() {
    return m_gravity;
  }

  /**
	* The world provides a single static ground body with no collision shapes.
	* You can use this to simplify the creation of joints and static shapes.
	*/
  b2Body GetGroundBody() {
    return m_groundBody;
  }

  static b2TimeStep s_timestep2 = new b2TimeStep();
  /**
	* Take a time step. This performs collision detection, integration,
	* and constraint solution.
	* @param timeStep the amount of time to simulate, this should not vary.
	* @param velocityIterations for the velocity constraint solver.
	* @param positionIterations for the position constraint solver.
	*/
  void Step(double dt, int velocityIterations, int positionIterations) {
    if ((m_flags & e_newFixture) > 0) {
      m_contactManager.FindNewContacts();
      m_flags &= ~e_newFixture;
    }

    m_flags |= e_locked;

    b2TimeStep step = s_timestep2;
    step.dt = dt;
    step.velocityIterations = velocityIterations;
    step.positionIterations = positionIterations;
    if (dt > 0.0) {
      step.inv_dt = 1.0 / dt;
    } else {
      step.inv_dt = 0.0;
    }

    step.dtRatio = m_inv_dt0 * dt;

    step.warmStarting = m_warmStarting;

    // Update contacts.
    m_contactManager.Collide();

    // Integrate velocities, solve velocity constraints, and integrate positions.
    if (step.dt > 0.0) {
      Solve(step);
    }

    // Handle TOI events.
    if (m_continuousPhysics && step.dt > 0.0) {
      SolveTOI(step);
    }

    if (step.dt > 0.0) {
      m_inv_dt0 = step.inv_dt;
    }
    m_flags &= ~e_locked;
  }

  /**
	 * Call this after you are done with time steps to clear the forces. You normally
	 * call this after each call to Step, unless you are performing sub-steps.
	 */
  void ClearForces() {
    for (b2Body body = m_bodyList; body != null; body = body.m_next) {
      body.m_force.SetZero();
      body.m_torque = 0.0;
    }
  }

  static b2Transform s_xf = new b2Transform();
  /**
	 * Call this to draw shapes and other debug draw data.
	 */
  void DrawDebugData() {

    if (m_debugDraw == null) {
      return;
    }

    m_debugDraw.m_sprite.graphics.clear();

    int flags = m_debugDraw.GetFlags();

    int i;
    b2Body b;
    b2Fixture f;
    b2Shape s;
    b2Joint j;
    IBroadPhase bp;
    b2Vec2 invQ = new b2Vec2();
    b2Vec2 x1 = new b2Vec2();
    b2Vec2 x2 = new b2Vec2();
    b2Transform xf;
    b2AABB b1 = new b2AABB();
    b2AABB b2 = new b2AABB();
    List vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];

    // Store color here and reuse, to reduce allocations
    b2Color color = new b2Color(0.0, 0.0, 0.0);

    if ((flags & b2DebugDraw.e_shapeBit) > 0) {
      for (b = m_bodyList; b != null; b = b.m_next) {
        xf = b.m_xf;
        for (f = b.GetFixtureList(); f != null; f = f.m_next) {
          s = f.GetShape();
          if (b.IsActive() == false) {
            color.Set(0.5, 0.5, 0.3);
            DrawShape(s, xf, color);
          } else if (b.GetType() == b2Body.b2_staticBody) {
            color.Set(0.5, 0.9, 0.5);
            DrawShape(s, xf, color);
          } else if (b.GetType() == b2Body.b2_kinematicBody) {
            color.Set(0.5, 0.5, 0.9);
            DrawShape(s, xf, color);
          } else if (b.IsAwake() == false) {
            color.Set(0.6, 0.6, 0.6);
            DrawShape(s, xf, color);
          } else {
            color.Set(0.9, 0.7, 0.7);
            DrawShape(s, xf, color);
          }
        }
      }
    }

    if ((flags & b2DebugDraw.e_jointBit) > 0) {
      for (j = m_jointList; j != null; j = j.m_next) {
        DrawJoint(j);
      }
    }

    if ((flags & b2DebugDraw.e_controllerBit) > 0) {
      for (b2Controller c = m_controllerList; c; c = c.m_next) {
        c.Draw(m_debugDraw);
      }
    }

    if ((flags & b2DebugDraw.e_pairBit) > 0) {
      color.Set(0.3, 0.9, 0.9);
      for (b2Contact contact = m_contactManager.m_contactList; contact != null; contact = contact.GetNext()) {
        b2Fixture fixtureA = contact.GetFixtureA();
        b2Fixture fixtureB = contact.GetFixtureB();

        b2Vec2 cA = fixtureA.GetAABB().GetCenter();
        b2Vec2 cB = fixtureB.GetAABB().GetCenter();

        m_debugDraw.DrawSegment(cA, cB, color);
      }
    }

    if ((flags & b2DebugDraw.e_aabbBit) > 0) {
      bp = m_contactManager.m_broadPhase;

      vs = [new b2Vec2(), new b2Vec2(), new b2Vec2(), new b2Vec2()];

      for (b = m_bodyList; b; b = b.GetNext()) {
        if (b.IsActive() == false) {
          continue;
        }
        for (f = b.GetFixtureList(); f; f = f.GetNext()) {
          b2AABB aabb = bp.GetFatAABB(f.m_proxy);
          vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
          vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
          vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
          vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

          m_debugDraw.DrawPolygon(vs, 4, color);
        }
      }
    }

    if ((flags & b2DebugDraw.e_centerOfMassBit) > 0) {
      for (b = m_bodyList; b != null; b = b.m_next) {
        xf = s_xf;
        xf.R = b.m_xf.R;
        xf.position = b.GetWorldCenter();
        m_debugDraw.DrawTransform(xf);
      }
    }
  }

  /**
	 * Query the world for all fixtures that potentially overlap the
	 * provided AABB.
	 * @param callback a user implemented callback class. It should match signature
	 * <code>function Callback(fixture:b2Fixture):bool</code>
	 * Return true to continue to the next fixture.
	 * @param aabb the query box.
	 */
  void QueryAABB(Function callback, b2AABB aabb) {
    IBroadPhase broadPhase = m_contactManager.m_broadPhase;
    bool WorldQueryWrapper(dynamic proxy) {
      return callback(broadPhase.GetUserData(proxy));
    }
    broadPhase.Query(WorldQueryWrapper, aabb);
  }
  /**
	 * Query the world for all fixtures that precisely overlap the
	 * provided transformed shape.
	 * @param callback a user implemented callback class. It should match signature
	 * <code>function Callback(fixture:b2Fixture):bool</code>
	 * Return true to continue to the next fixture.
	 * @asonly
	 */
  void QueryShape(Function callback, b2Shape shape, [b2Transform transform = null]) {
    if (transform == null) {
      transform = new b2Transform();
      transform.SetIdentity();
    }
    IBroadPhase broadPhase = m_contactManager.m_broadPhase;
    bool WorldQueryWrapper(dynamic proxy) {
      b2Fixture fixture = broadPhase.GetUserData(proxy) as b2Fixture;
      if (b2Shape.TestOverlap(shape, transform, fixture.GetShape(), fixture.GetBody().GetTransform())) return callback(fixture);
      return true;
    }
    b2AABB aabb = new b2AABB();
    shape.ComputeAABB(aabb, transform);
    broadPhase.Query(WorldQueryWrapper, aabb);
  }

  /**
	 * Query the world for all fixtures that contain a point.
	 * @param callback a user implemented callback class. It should match signature
	 * <code>function Callback(fixture:b2Fixture):bool</code>
	 * Return true to continue to the next fixture.
	 * @asonly
	 */
  void QueryPoint(Function callback, b2Vec2 p) {
    IBroadPhase broadPhase = m_contactManager.m_broadPhase;
    bool WorldQueryWrapper(dynamic proxy) {
      b2Fixture fixture = broadPhase.GetUserData(proxy) as b2Fixture;
      if (fixture.TestPoint(p)) return callback(fixture);
      return true;
    }
    // Make a small box.
    b2AABB aabb = new b2AABB();
    aabb.lowerBound.Set(p.x - b2Settings.b2_linearSlop, p.y - b2Settings.b2_linearSlop);
    aabb.upperBound.Set(p.x + b2Settings.b2_linearSlop, p.y + b2Settings.b2_linearSlop);
    broadPhase.Query(WorldQueryWrapper, aabb);
  }

  /**
	 * Ray-cast the world for all fixtures in the path of the ray. Your callback
	 * Controls whether you get the closest point, any point, or n-points
	 * The ray-cast ignores shapes that contain the starting point
	 * @param callback A callback function which must be of signature: dynamic <code>function Callback(fixture:b2Fixture,    // The fixture hit by the ray
	 * point:b2Vec2,         // The point of initial intersection
	 * normal:b2Vec2,        // The normal vector at the point of intersection
	 * fraction:double       // The fractional length along the ray of the intersection
	 * ):double
	 * </code>
	 * Callback should return the new length of the ray as a fraction of the original length.
	 * By returning 0, you immediately terminate.
	 * By returning 1, you continue wiht the original ray.
	 * By returning the current fraction, you proceed to find the closest point.
	 * @param point1 the ray starting point
	 * @param point2 the ray ending point
	 */
  void RayCast(Function callback, b2Vec2 point1, b2Vec2 point2) {
    IBroadPhase broadPhase = m_contactManager.m_broadPhase;
    b2RayCastOutput output = new b2RayCastOutput();
    double RayCastWrapper(b2RayCastInput input, dynamic proxy) {
      dynamic userData = broadPhase.GetUserData(proxy);
      b2Fixture fixture = userData as b2Fixture;
      bool hit = fixture.RayCast(output, input);
      if (hit ) {
        double fraction = output.fraction;
        b2Vec2 point = new b2Vec2((1.0 - fraction) * point1.x + fraction * point2.x, (1.0 - fraction) * point1.y + fraction * point2.y);
        return callback(fixture, point, output.normal, fraction);
      }
      return input.maxFraction;
    }
    b2RayCastInput input = new b2RayCastInput(point1, point2);
    broadPhase.RayCast(RayCastWrapper, input);
  }

  b2Fixture RayCastOne(b2Vec2 point1, b2Vec2 point2) {
    b2Fixture result;
    double RayCastOneWrapper(b2Fixture fixture, b2Vec2 point, b2Vec2 normal, double fraction) {
      result = fixture;
      return fraction;
    }
    RayCast(RayCastOneWrapper, point1, point2);
    return result;
  }

  List<b2Fixture> RayCastAll(b2Vec2 point1, b2Vec2 point2) {
    List<b2Fixture> result = new List<b2Fixture>();
    double RayCastAllWrapper(b2Fixture fixture, b2Vec2 point, b2Vec2 normal, double fraction) {
      result[result.length] = fixture;
      return 1.0;
    }
    RayCast(RayCastAllWrapper, point1, point2);
    return result;
  }

  /**
	* Get the world body list. With the returned body, use b2Body::GetNext to get
	* the next body in the world list. A NULL body indicates the end of the list.
	* @return the head of the world body list.
	*/
  b2Body GetBodyList() {
    return m_bodyList;
  }

  /**
	* Get the world joint list. With the returned joint, use b2Joint::GetNext to get
	* the next joint in the world list. A NULL joint indicates the end of the list.
	* @return the head of the world joint list.
	*/
  b2Joint GetJointList() {
    return m_jointList;
  }

  /**
	 * Get the world contact list. With the returned contact, use b2Contact::GetNext to get
	 * the next contact in the world list. A NULL contact indicates the end of the list.
	 * @return the head of the world contact list.
	 * @warning contacts are 
	 */
  b2Contact GetContactList() {
    return m_contactList;
  }

  /**
	 * Is the world locked (in the middle of a time step).
	 */
  bool IsLocked() {
    return (m_flags & e_locked) > 0;
  }

  //--------------- Internals Below -------------------
  // Internal yet  to make life easier.

  // Find islands, integrate and solve constraints, solve position constraints
  List<b2Body> s_stack = new List<b2Body>();
  void Solve(b2TimeStep step) {
    b2Body b;

    // Step all controllers
    for (b2Controller controller = m_controllerList; controller != null; controller = controller.m_next) {
      controller.Step(step);
    }

    // Size the island for the worst case.
    b2Island island = m_island;
    island.Initialize(m_bodyCount, m_contactCount, m_jointCount, null, m_contactManager.m_contactListener, m_contactSolver);

    // Clear all the island flags.
    for (b = m_bodyList; b != null; b = b.m_next) {
      b.m_flags &= ~b2Body.e_islandFlag;
    }
    for (b2Contact c = m_contactList; c != null; c = c.m_next) {
      c.m_flags &= ~b2Contact.e_islandFlag;
    }
    for (b2Joint j = m_jointList; j != null; j = j.m_next) {
      j.m_islandFlag = false;
    }

    // Build and simulate all awake islands.
    int stackSize = m_bodyCount;
    //b2Body** stack = (b2Body**)m_stackAllocator.Allocate(stackSize * sizeof(b2Body*));
    List<b2Body> stack = s_stack;
    for (b2Body seed = m_bodyList; seed != null; seed = seed.m_next) {
      if ((seed.m_flags & b2Body.e_islandFlag) > 0) {
        continue;
      }

      if (seed.IsAwake() == false || seed.IsActive() == false) {
        continue;
      }

      // The seed can be dynamic or kinematic.
      if (seed.GetType() == b2Body.b2_staticBody) {
        continue;
      }

      // Reset island and stack.
      island.Clear();
      int stackCount = 0;

      if (stack.length - 1 <= stackCount) {
        stack.add(seed);
      } else {
        stack[stackCount] = seed;
      }
      stackCount++;

      seed.m_flags |= b2Body.e_islandFlag;

      // Perform a depth first search (on as DFS) the constraint graph.
      while (stackCount > 0) {
        // Grab the next body off the stack and add it to the island.
        b = stack[--stackCount];
        
        if(b == null) continue;
        
        //b2Assert(b.IsActive() == true);
        island.AddBody(b);

        // Make sure the body is awake.
        if (b.IsAwake() == false) {
          b.SetAwake(true);
        }

        // To keep islands as small as possible, we don't
        // propagate islands across static bodies.
        if (b.GetType() == b2Body.b2_staticBody) {
          continue;
        }

        b2Body other;
        // Search all contacts connected to this body.
        for (b2ContactEdge ce = b.m_contactList; ce != null; ce = ce.next) {
          // Has this contact already been added to an island?
          if ((ce.contact.m_flags & b2Contact.e_islandFlag) > 0) {
            continue;
          }

          // Is this contact solid and touching?
          if (ce.contact.IsSensor() == true || ce.contact.IsEnabled() == false || ce.contact.IsTouching() == false) {
            continue;
          }

          island.AddContact(ce.contact);
          ce.contact.m_flags |= b2Contact.e_islandFlag;

          //b2Body other = ce.other;
          other = ce.other;

          // Was the other body already added to this island?
          if ((other.m_flags & b2Body.e_islandFlag) > 0) {
            continue;
          }

          //b2Settings.b2Assert(stackCount < stackSize);

          if (stack.length - 1 <= stackCount) {
            stack.add(other);
          } else {
            stack[stackCount] = other;
          }
          stackCount++;

          other.m_flags |= b2Body.e_islandFlag;
        }

        // Search all joints connect to this body.
        for (b2JointEdge jn = b.m_jointList; jn != null; jn = jn.next) {
          if (jn.joint.m_islandFlag == true) {
            continue;
          }

          other = jn.other;

          // Don't simulate joints connected to inactive bodies.
          if (other.IsActive() == false) {
            continue;
          }

          island.AddJoint(jn.joint);
          jn.joint.m_islandFlag = true;

          if ((other.m_flags & b2Body.e_islandFlag) > 0) {
            continue;
          }

          //b2Settings.b2Assert(stackCount < stackSize);
          //stack[stackCount++] = other;
          if (stack.length - 1 <= stackCount) {
            stack.add(other);
          } else {
            stack[stackCount] = other;
          }

          stackCount++;
          other.m_flags |= b2Body.e_islandFlag;
        }
      }
      island.Solve(step, m_gravity, m_allowSleep);

      // Post solve cleanup.
      for (int i = 0; i < island.m_bodyCount; ++i) {
        // Allow static bodies to participate in other islands.
        b = island.m_bodies[i];
        if (b.GetType() == b2Body.b2_staticBody) {
          b.m_flags &= ~b2Body.e_islandFlag;
        }
      }
    }

    //m_stackAllocator.Free(stack);
    for (int i = 0; i < stack.length; ++i) {
      if (stack[i] == null) break;
      stack[i] = null;
    }

    // Synchronize fixutres, check for out of range bodies.
    for (b = m_bodyList; b != null; b = b.m_next) {
      if (b.IsAwake() == false || b.IsActive() == false) {
        continue;
      }

      if (b.GetType() == b2Body.b2_staticBody) {
        continue;
      }

      // Update fixtures (for broad-phase).
      b.SynchronizeFixtures();
    }

    // Look for new contacts.
    m_contactManager.FindNewContacts();

  }

  static b2Sweep s_backupA = new b2Sweep();
  static b2Sweep s_backupB = new b2Sweep();
  static b2TimeStep s_timestep = new b2TimeStep();
  static List<b2Body> s_queue = new List<b2Body>();
  // Find TOI contacts and solve them.
  void SolveTOI(b2TimeStep step) {

    b2Body b;
    b2Fixture fA;
    b2Fixture fB;
    b2Body bA;
    b2Body bB;
    b2ContactEdge cEdge;
    b2Joint j;

    // Reserve an island and a queue for TOI island solution.
    b2Island island = m_island;
    island.Initialize(m_bodyCount, b2Settings.b2_maxTOIContactsPerIsland, b2Settings.b2_maxTOIJointsPerIsland, null, m_contactManager.m_contactListener, m_contactSolver);

    //Simple one pass queue
    //Relies on the fact that we're only making one pass
    //through and each body can only be.added/popped one.
    //To.add:
    //  queue[queueStart+queueSize++] = newElement;
    //To pop:
    //  poppedElement = queue[queueStart++];
    //  --queueSize;

    List<b2Body> queue = s_queue;

    for (b = m_bodyList; b != null; b = b.m_next) {
      b.m_flags &= ~b2Body.e_islandFlag;
      b.m_sweep.t0 = 0.0;
    }

    b2Contact c;
    for (c = m_contactList; c != null; c = c.m_next) {
      // Invalidate TOI
      c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
    }

    for (j = m_jointList; j != null; j = j.m_next) {
      j.m_islandFlag = false;
    }

    // Find TOI events and solve them.
    for ( ; ; ) {
      // Find the first TOI.
      b2Contact minContact = null;
      double minTOI = 1.0;

      for (c = m_contactList; c != null; c = c.m_next) {
        // Can this contact generate a solid TOI contact?
        if (c.IsSensor() == true || c.IsEnabled() == false || c.IsContinuous() == false) {
          continue;
        }

        // TODO_ERIN keep a counter on the contact, only respond to M TOIs per contact.

        double toi = 1.0;
        if ((c.m_flags & b2Contact.e_toiFlag) > 0) {
          // This contact has a valid cached TOI.
          toi = c.m_toi;
        } else {
          // Compute the TOI for this contact.
          fA = c.m_fixtureA;
          fB = c.m_fixtureB;
          bA = fA.m_body;
          bB = fB.m_body;

          if ((bA.GetType() != b2Body.b2_dynamicBody || bA.IsAwake() == false) && (bB.GetType() != b2Body.b2_dynamicBody || bB.IsAwake() == false)) {
            continue;
          }

          // Put the sweeps onto the same time interval.
          double t0 = bA.m_sweep.t0;

          if (bA.m_sweep.t0 < bB.m_sweep.t0) {
            t0 = bB.m_sweep.t0;
            bA.m_sweep.Advance(t0);
          } else if (bB.m_sweep.t0 < bA.m_sweep.t0) {
            t0 = bA.m_sweep.t0;
            bB.m_sweep.Advance(t0);
          }

          //b2Settings.b2Assert(t0 < 1.0f);

          // Compute the time of impact.
          toi = c.ComputeTOI(bA.m_sweep, bB.m_sweep);
          b2Settings.b2Assert(0.0 <= toi && toi <= 1.0);

          // If the TOI is in range ...
          if (toi > 0.0 && toi < 1.0) {
            // Interpolate on the actual range.
            //toi = /*Math.*/min((1.0 - toi) * t0 + toi, 1.0);
            toi = (1.0 - toi) * t0 + toi;
            if (toi > 1.0) toi = 1.0;
          }


          c.m_toi = toi;
          c.m_flags |= b2Contact.e_toiFlag;
        }

        if (double.MIN_POSITIVE < toi && toi < minTOI) {
          // This is the minimum TOI found so far.
          minContact = c;
          minTOI = toi;
        }
      }

      if (minContact == null || 1.0 - 100.0 * double.MIN_POSITIVE < minTOI) {
        // No more TOI events. Done!
        break;
      }

      // Advance the bodies to the TOI.
      fA = minContact.m_fixtureA;
      fB = minContact.m_fixtureB;
      bA = fA.m_body;
      bB = fB.m_body;
      s_backupA.Set(bA.m_sweep);
      s_backupB.Set(bB.m_sweep);
      bA.Advance(minTOI);
      bB.Advance(minTOI);

      // The TOI contact likely has some new contact points.
      minContact.Update(m_contactManager.m_contactListener);
      minContact.m_flags &= ~b2Contact.e_toiFlag;

      // Is the contact solid?
      if (minContact.IsSensor() == true || minContact.IsEnabled() == false) {
        // Restore the sweeps
        bA.m_sweep.Set(s_backupA);
        bB.m_sweep.Set(s_backupB);
        bA.SynchronizeTransform();
        bB.SynchronizeTransform();
        continue;
      }

      // Did doubleerical issues prevent;,ontact pointjrom being generated
      if (minContact.IsTouching() == false) {
        // Give up on this TOI
        continue;
      }

      // Build the TOI island. We need a dynamic seed.
      b2Body seed = bA;
      if (seed.GetType() != b2Body.b2_dynamicBody) {
        seed = bB;
      }

      // Reset island and queue.
      island.Clear();
      int queueStart = 0; //start index for queue
      int queueSize = 0; //elements in queue
      
      if (queue.length - 1 <= queueStart + queueSize) {
        queue.add(seed);
                } else {
                  queue[queueStart + queueSize] = seed;
                }

      queueSize++;
      
      seed.m_flags |= b2Body.e_islandFlag;

      // Perform a breadth first search (on as BFS) the contact graph.
      while (queueSize > 0) {
        // Grab the next body off the stack and add it to the island.
        b = queue[queueStart++];
        --queueSize;

        island.AddBody(b);

        // Make sure the body is awake.
        if (b.IsAwake() == false) {
          b.SetAwake(true);
        }

        // To keep islands as small as possible, we don't
        // propagate islands across static or kinematic bodies.
        if (b.GetType() != b2Body.b2_dynamicBody) {
          continue;
        }

        b2Body other;
        // Search all contacts connected to this body.
        for (cEdge = b.m_contactList; cEdge != null; cEdge = cEdge.next) {
          // Does the TOI island still have space for contacts?
          if (island.m_contactCount == island.m_contactCapacity) {
            break;
          }

          // Has this contact already been added to an island?
          if ((cEdge.contact.m_flags & b2Contact.e_islandFlag) > 0) {
            continue;
          }

          // Skip sperate, sensor, or disabled contacts.
          if (cEdge.contact.IsSensor() == true || cEdge.contact.IsEnabled() == false || cEdge.contact.IsTouching() == false) {
            continue;
          }

          island.AddContact(cEdge.contact);
          cEdge.contact.m_flags |= b2Contact.e_islandFlag;

          // Update other body.
          other = cEdge.other;

          // Was the other body already added to this island?
          if ((other.m_flags & b2Body.e_islandFlag) > 0) {
            continue;
          }

          // Synchronize the connected body.
          if (other.GetType() != b2Body.b2_staticBody) {
            other.Advance(minTOI);
            other.SetAwake(true);
          }

          //b2Settings.b2Assert(queueStart + queueSize < queueCapacity);
          //queue[queueStart + queueSize] = other;
          //++queueSize;
          
          if (queue.length - 1 <= queueStart + queueSize) {
                  queue.add(other);
                          } else {
                            queue[queueStart + queueSize] = other;
                          }

                queueSize++;
          
          other.m_flags |= b2Body.e_islandFlag;
        }

        for (b2JointEdge jEdge = b.m_jointList; jEdge != null ; jEdge = jEdge.next) {
          if (island.m_jointCount == island.m_jointCapacity) continue;

          if (jEdge.joint.m_islandFlag == true) continue;

          other = jEdge.other;
          if (other.IsActive() == false) {
            continue;
          }

          island.AddJoint(jEdge.joint);
          jEdge.joint.m_islandFlag = true;

          if ((other.m_flags & b2Body.e_islandFlag) > 0) continue;

          // Synchronize the connected body.
          if (other.GetType() != b2Body.b2_staticBody) {
            other.Advance(minTOI);
            other.SetAwake(true);
          }

          //b2Settings.b2Assert(queueStart + queueSize < queueCapacity);
          if (queue.length - 1 <= queueStart + queueSize) {
                 queue.add(seed);
                         } else {
                           queue[queueStart + queueSize] = seed;
                         }

               queueSize++;
          other.m_flags |= b2Body.e_islandFlag;
        }
      }

      b2TimeStep subStep = s_timestep;
      subStep.warmStarting = false;
      subStep.dt = (1.0 - minTOI) * step.dt;
      subStep.inv_dt = 1.0 / subStep.dt;
      subStep.dtRatio = 0.0;
      subStep.velocityIterations = step.velocityIterations;
      subStep.positionIterations = step.positionIterations;

      island.SolveTOI(subStep);

      int i;
      // Post solve cleanup.
      for (i = 0; i < island.m_bodyCount; ++i) {
        // Allow bodies to participate in future TOI islands.
        b = island.m_bodies[i];
        b.m_flags &= ~b2Body.e_islandFlag;

        if (b.IsAwake() == false) {
          continue;
        }

        if (b.GetType() != b2Body.b2_dynamicBody) {
          continue;
        }

        // Update fixtures (for broad-phase).
        b.SynchronizeFixtures();

        // Invalidate all contact TOIs associated with this body. Some of these
        // may not be in the island because they were not touching.
        for (cEdge = b.m_contactList; cEdge != null; cEdge = cEdge.next) {
          cEdge.contact.m_flags &= ~b2Contact.e_toiFlag;
        }
      }

      for (i = 0; i < island.m_contactCount; ++i) {
        // Allow contacts to participate in future TOI islands.
        c = island.m_contacts[i];
        c.m_flags &= ~(b2Contact.e_toiFlag | b2Contact.e_islandFlag);
      }

      for (i = 0; i < island.m_jointCount; ++i) {
        // Allow joints to participate in future TOI islands
        j = island.m_joints[i];
        j.m_islandFlag = false;
      }

      // Commit fixture proxy movements to the broad-phase so that new contacts are created.
      // Also, some contacts can be destroyed.
      m_contactManager.FindNewContacts();
    }

    //m_stackAllocator.Free(queue);
  }

  static b2Color s_jointColor = new b2Color(0.5, 0.8, 0.8);
  //
  void DrawJoint(b2Joint joint) {

    b2Body b1 = joint.GetBodyA();
    b2Body b2 = joint.GetBodyB();
    b2Transform xf1 = b1.m_xf;
    b2Transform xf2 = b2.m_xf;
    b2Vec2 x1 = xf1.position;
    b2Vec2 x2 = xf2.position;
    b2Vec2 p1 = joint.GetAnchorA();
    b2Vec2 p2 = joint.GetAnchorB();

    //b2Color color(0.5f, 0.8f, 0.8f);
    b2Color color = s_jointColor;

    switch (joint.m_type) {
      case b2Joint.e_distanceJoint:
        m_debugDraw.DrawSegment(p1, p2, color);
        break;

      case b2Joint.e_pulleyJoint:
        {
          b2PulleyJoint pulley = (joint as b2PulleyJoint);
          b2Vec2 s1 = pulley.GetGroundAnchorA();
          b2Vec2 s2 = pulley.GetGroundAnchorB();
          m_debugDraw.DrawSegment(s1, p1, color);
          m_debugDraw.DrawSegment(s2, p2, color);
          m_debugDraw.DrawSegment(s1, s2, color);
        }
        break;

      case b2Joint.e_mouseJoint:
        m_debugDraw.DrawSegment(p1, p2, color);
        break;

      default:
        if (b1 != m_groundBody) m_debugDraw.DrawSegment(x1, p1, color);
        m_debugDraw.DrawSegment(p1, p2, color);
        if (b2 != m_groundBody) m_debugDraw.DrawSegment(x2, p2, color);
    }
  }

  void DrawShape(b2Shape shape, b2Transform xf, b2Color color) {

    switch (shape.m_type) {
      case b2Shape.e_circleShape:
        {
          b2CircleShape circle = (shape as b2CircleShape);

          b2Vec2 center = b2Math.MulX(xf, circle.m_p);
          double radius = circle.m_radius;
          b2Vec2 axis = xf.R.col1;

          m_debugDraw.DrawSolidCircle(center, radius, axis, color);
        }
        break;

      case b2Shape.e_polygonShape:
        {
          int i;
          b2PolygonShape poly = (shape as b2PolygonShape);
          int vertexCount = poly.GetVertexCount();
          List<b2Vec2> localVertices = poly.GetVertices();

          List<b2Vec2> vertices = new List<b2Vec2>(vertexCount);

          for (i = 0; i < vertexCount; ++i) {
            vertices[i] = b2Math.MulX(xf, localVertices[i]);
          }

          m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
        }
        break;

      case b2Shape.e_edgeShape:
        {
          b2EdgeShape edge = shape as b2EdgeShape;

          m_debugDraw.DrawSegment(b2Math.MulX(xf, edge.GetVertex1()), b2Math.MulX(xf, edge.GetVertex2()), color);

        }
        break;
    }
  }

  int m_flags = 0;

  b2ContactManager m_contactManager = new b2ContactManager();

  // These two are stored purely for efficiency purposes, they don't maintain
  // any data outside of a call to Step
  b2ContactSolver m_contactSolver = new b2ContactSolver();
  b2Island m_island = new b2Island();

  b2Body m_bodyList;
  b2Joint m_jointList;

  b2Contact m_contactList;

  int m_bodyCount;
  int m_contactCount;
  int m_jointCount;
  b2Controller m_controllerList;
  int m_controllerCount;

  b2Vec2 m_gravity;
  bool m_allowSleep;

  b2Body m_groundBody;

  b2DestructionListener m_destructionListener;
  b2DebugDraw m_debugDraw;

  // This is used to compute the time step ratio to support a variable time step.
  double m_inv_dt0;

  // This is for debugging the solver.
  static bool m_warmStarting;

  // This is for debugging the solver.
  static bool m_continuousPhysics;

  // m_flags
  static final int e_newFixture = 0x0001;
  static final int e_locked = 0x0002;

}

