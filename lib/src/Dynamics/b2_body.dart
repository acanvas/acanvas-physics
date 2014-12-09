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
* A rigid body.
*/
class b2Body {

  double connectEdges(b2EdgeShape s1, b2EdgeShape s2, double angle1) {
    double angle2 = atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
    double coreOffset = /*Math.*/tan((angle2 - angle1) * 0.5);
    b2Vec2 core = b2Math.MulFV(coreOffset, s2.GetDirectionVector());
    core = b2Math.SubtractVV(core, s2.GetNormalVector());
    core = b2Math.MulFV(b2Settings.b2_toiSlop, core);
    core = b2Math.AddVV(core, s2.GetVertex1());
    b2Vec2 cornerDir = b2Math.AddVV(s1.GetDirectionVector(), s2.GetDirectionVector());
    cornerDir.Normalize();
    bool convex = b2Math.Dot(s1.GetDirectionVector(), s2.GetNormalVector()) > 0.0;
    s1.SetNextEdge(s2, core, cornerDir, convex);
    s2.SetPrevEdge(s1, core, cornerDir, convex);
    return angle2;
  }

  /**
	 * Creates a fixture and attach it to this body. Use this function if you need
	 * to set some fixture parameters, like friction. Otherwise you can create the
	 * fixture directly from a shape.
	 * If the density is non-zero, this function automatically updates the mass of the body.
	 * Contacts are not created until the next time step.
	 * @param fixtureDef the fixture definition.
	 * @warning This function is locked during callbacks.
	 */
  b2Fixture CreateFixture(b2FixtureDef def) {
    //b2Settings.b2Assert(m_world.IsLocked() == false);
    if (m_world.IsLocked() == true) {
      return null;
    }

    // TODO: Decide on a better place to initialize edgeShapes. (b2Shape::Create() can't
    //       return more than one shape to add to parent body... maybe it should add
    //       shapes directly to the body instead of returning them?)
    /*
		if (def.type == b2Shape.e_edgeShape) {
			b2EdgeChainDef edgeDef = def as b2EdgeChainDef;
			b2Vec2 v1;
			b2Vec2 v2;
			int i = 0;
			
			if (edgeDef.isALoop) {
				v1 = edgeDef.vertices[edgeDef.vertexCount-1];
				i = 0;
			} else {
				v1 = edgeDef.vertices[0];
				i = 1;
			}
			
			b2EdgeShape s0 = null;
			b2EdgeShape s1 = null;
			b2EdgeShape s2 = null;
			double angle = 0.0;
			for (; i < edgeDef.vertexCount; i++) {
				v2 = edgeDef.vertices[i];
				
				//void* mem = m_world->m_blockAllocator.Allocate(sizeof(b2EdgeShape));
				s2 = new b2EdgeShape(v1, v2, def);
				s2.m_next = m_shapeList;
				m_shapeList = s2;
				++m_shapeCount;
				s2.m_body = this;
				s2.CreateProxy(m_world.m_broadPhase, m_xf);
				s2.UpdateSweepRadius(m_sweep.localCenter);
				
				if (s1 == null) {
					s0 = s2;
					angle = Math.atan2(s2.GetDirectionVector().y, s2.GetDirectionVector().x);
				} else {
					angle = connectEdges(s1, s2, angle);
				}
				s1 = s2;
				v1 = v2;
			}
			if (edgeDef.isALoop) connectEdges(s1, s0, angle);
			return s0;
		}*/

    b2Fixture fixture = new b2Fixture();
    fixture.Create(this, m_xf, def);

    if ((m_flags & e_activeFlag) > 0) {
      IBroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
      fixture.CreateProxy(broadPhase, m_xf);
    }

    fixture.m_next = m_fixtureList;
    m_fixtureList = fixture;
    ++m_fixtureCount;

    fixture.m_body = this;

    // Adjust mass properties if needed
    if (fixture.m_density > 0.0) {
      ResetMassData();
    }

    // Let the world know we have a new fixture. This will cause new contacts to be created
    // at the beginning of the next time step.
    m_world.m_flags |= b2World.e_newFixture;

    return fixture;
  }

  /**
	 * Creates a fixture from a shape and attach it to this body.
	 * This is a convenience function. Use b2FixtureDef if you need to set parameters
	 * like friction, restitution, user data, or filtering.
	 * This function automatically updates the mass of the body.
	 * @param shape the shape to be cloned.
	 * @param density the shape density (set to zero for static bodies).
	 * @warning This function is locked during callbacks.
	 */
  b2Fixture CreateFixture2(b2Shape shape, [double density = 0.0]) {
    b2FixtureDef def = new b2FixtureDef();
    def.shape = shape;
    def.density = density;

    return CreateFixture(def);
  }

  /**
	 * Destroy a fixture. This removes the fixture from the broad-phase and
	 * destroys all contacts associated with this fixture. This will
	 * automatically adjust the mass of the body if the body is dynamic and the
	 * fixture has positive density.
	 * All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	 * @param fixture the fixture to be removed.
	 * @warning This function is locked during callbacks.
	 */
  void DestroyFixture(b2Fixture fixture) {
    //b2Settings.b2Assert(m_world.IsLocked() == false);
    if (m_world.IsLocked() == true) {
      return;
    }

    //b2Settings.b2Assert(m_fixtureCount > 0);
    //b2Fixture** node = &m_fixtureList;
    b2Fixture node = m_fixtureList;
    b2Fixture ppF = null; // Fix pointer-pointer stuff
    bool found = false;
    while (node != null) {
      if (node == fixture) {
        if (ppF != null) ppF.m_next = fixture.m_next; else m_fixtureList = fixture.m_next;
        //node = fixture.m_next;
        found = true;
        break;
      }

      ppF = node;
      node = node.m_next;
    }

    // You tried to remove a shape that is not attached to this body.
    //b2Settings.b2Assert(found);

    // Destroy any contacts associated with the fixture.
    b2ContactEdge edge = m_contactList;
    while (edge != null) {
      b2Contact c = edge.contact;
      edge = edge.next;

      b2Fixture fixtureA = c.GetFixtureA();
      b2Fixture fixtureB = c.GetFixtureB();
      if (fixture == fixtureA || fixture == fixtureB) {
        // This destros the contact and removes it from
        // this body's contact list
        m_world.m_contactManager.Destroy(c);
      }
    }

    if ((m_flags & e_activeFlag) > 0) {
      IBroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
      fixture.DestroyProxy(broadPhase);
    } else {
      //b2Assert(fixture->m_proxyId == b2BroadPhase::e_nullProxy);
    }

    fixture.Destroy();
    fixture.m_body = null;
    fixture.m_next = null;

    --m_fixtureCount;

    // Reset the mass data.
    ResetMassData();
  }

  /**
	* Set the position of the body's origin and rotation (radians).
	* This breaks any contacts and wakes the other bodies.
	* @param position the new world position of the body's origin (not necessarily
	* the center of mass).
	* @param angle the new world rotation angle of the body in radians.
	*/
  void SetPositionAndAngle(b2Vec2 position, double angle) {

    b2Fixture f;

    //b2Settings.b2Assert(m_world.IsLocked() == false);
    if (m_world.IsLocked() == true) {
      return;
    }

    m_xf.R.Set(angle);
    m_xf.position.SetV(position);

    //m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
    //b2MulMV(m_xf.R, m_sweep.localCenter);
    b2Mat22 tMat = m_xf.R;
    b2Vec2 tVec = m_sweep.localCenter;
    // (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    // (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    //return T.position + b2Mul(T.R, v);
    m_sweep.c.x += m_xf.position.x;
    m_sweep.c.y += m_xf.position.y;
    //m_sweep.c0 = m_sweep.c
    m_sweep.c0.SetV(m_sweep.c);

    m_sweep.a0 = m_sweep.a = angle;

    IBroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
    for (f = m_fixtureList; f != null; f = f.m_next) {
      f.Synchronize(broadPhase, m_xf, m_xf);
    }
    m_world.m_contactManager.FindNewContacts();
  }

  /**
	 * Set the position of the body's origin and rotation (radians).
	 * This breaks any contacts and wakes the other bodies.
	 * Note this is less efficient than the other overload - you should use that
	 * if the angle is available.
	 * @param xf the transform of position and angle to set the bdoy to.
	 */
  void SetTransform(b2Transform xf) {
    SetPositionAndAngle(xf.position, xf.GetAngle());
  }

  /**
	* Get the body transform for the body's origin.
	* @return the world transform of the body's origin.
	*/
  b2Transform GetTransform() {
    return m_xf;
  }

  /**
	* Get the world body origin position.
	* @return the world position of the body's origin.
	*/
  b2Vec2 GetPosition() {
    return m_xf.position;
  }

  /**
	 * Setthe world body origin position.
	 * @param position the new position of the body
	 */
  void SetPosition(b2Vec2 position) {
    SetPositionAndAngle(position, GetAngle());
  }

  /**
	* Get the angle in radians.
	* @return the current world rotation angle in radians.
	*/
  double GetAngle() {
    return m_sweep.a;
  }

  /**
	 * Set the world body angle
	 * @param angle the new angle of the body.
	 */
  void SetAngle(double angle) {
    SetPositionAndAngle(GetPosition(), angle);
  }


  /**
	* Get the world position of the center of mass.
	*/
  b2Vec2 GetWorldCenter() {
    return m_sweep.c;
  }

  /**
	* Get the local position of the center of mass.
	*/
  b2Vec2 GetLocalCenter() {
    return m_sweep.localCenter;
  }

  /**
	* Set the linear velocity of the center of mass.
	* @param v the new linear velocity of the center of mass.
	*/
  void SetLinearVelocity(b2Vec2 v) {
    if (m_type == b2_staticBody) {
      return;
    }
    m_linearVelocity.SetV(v);
  }

  /**
	* Get the linear velocity of the center of mass.
	* @return the linear velocity of the center of mass.
	*/
  b2Vec2 GetLinearVelocity() {
    return m_linearVelocity;
  }

  /**
	* Set the angular velocity.
	* @param omega the new angular velocity in radians/second.
	*/
  void SetAngularVelocity(double omega) {
    if (m_type == b2_staticBody) {
      return;
    }
    m_angularVelocity = omega;
  }

  /**
	* Get the angular velocity.
	* @return the angular velocity in radians/second.
	*/
  double GetAngularVelocity() {
    return m_angularVelocity;
  }

  /**
	 * Get the definition containing the body properties.
	 * @asonly
	 */
  b2BodyDef GetDefinition() {
    b2BodyDef bd = new b2BodyDef();
    bd.type = GetType();
    bd.allowSleep = (m_flags & e_allowSleepFlag) == e_allowSleepFlag;
    bd.angle = GetAngle();
    bd.angularDamping = m_angularDamping;
    bd.angularVelocity = m_angularVelocity;
    bd.fixedRotation = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
    bd.bullet = (m_flags & e_bulletFlag) == e_bulletFlag;
    bd.awake = (m_flags & e_awakeFlag) == e_awakeFlag;
    bd.linearDamping = m_linearDamping;
    bd.linearVelocity.SetV(GetLinearVelocity());
    bd.position = GetPosition();
    bd.userData = GetUserData();
    return bd;
  }

  /**
	* Apply a force at a world point. If the force is not
	* applied at the center of mass, it will generate a torque and
	* affect the angular velocity. This wakes up the body.
	* @param force the world force vector, usually in Newtons (N).
	* @param point the world position of the point of application.
	*/
  void ApplyForce(b2Vec2 force, b2Vec2 point) {
    if (m_type != b2_dynamicBody) {
      return;
    }

    if (IsAwake() == false) {
      SetAwake(true);
    }

    //m_force += force;
    m_force.x += force.x;
    m_force.y += force.y;
    //m_torque += b2Cross(point - m_sweep.c, force);
    m_torque += ((point.x - m_sweep.c.x) * force.y - (point.y - m_sweep.c.y) * force.x);
  }

  /**
	* Apply a torque. This affects the angular velocity
	* without affecting the linear velocity of the center of mass.
	* This wakes up the body.
	* @param torque about the z-axis (out of the screen), usually in N-m.
	*/
  void ApplyTorque(double torque) {
    if (m_type != b2_dynamicBody) {
      return;
    }

    if (IsAwake() == false) {
      SetAwake(true);
    }
    m_torque += torque;
  }

  /**
	* Apply an impulse at a point. This immediately modifies the velocity.
	* It also modifies the angular velocity if the point of application
	* is not at the center of mass. This wakes up the body.
	* @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	* @param point the world position of the point of application.
	*/
  void ApplyImpulse(b2Vec2 impulse, b2Vec2 point) {
    if (m_type != b2_dynamicBody) {
      return;
    }

    if (IsAwake() == false) {
      SetAwake(true);
    }
    //m_linearVelocity += m_invMass * impulse;
    m_linearVelocity.x += m_invMass * impulse.x;
    m_linearVelocity.y += m_invMass * impulse.y;
    //m_angularVelocity += m_invI * b2Cross(point - m_sweep.c, impulse);
    m_angularVelocity += m_invI * ((point.x - m_sweep.c.x) * impulse.y - (point.y - m_sweep.c.y) * impulse.x);
  }

  /**
	 * Splits a body into two, preserving dynamic properties
	 * @param	callback Called once per fixture, return true to move this fixture to the new body
	 * <code>function Callback(fixture:b2Fixture):bool</code>
	 * @return The newly created bodies
	 * @asonly
	 */
  b2Body Split(Function callback) {
    b2Vec2 linearVelocity = GetLinearVelocity().Copy();//Reset mass will alter this
    double angularVelocity = GetAngularVelocity();
    b2Vec2 center = GetWorldCenter();
    b2Body body1 = this;
    b2Body body2 = m_world.CreateBody(GetDefinition());

    b2Fixture prev;
    for (b2Fixture f = body1.m_fixtureList; f; ) {
      if (callback(f)) {
        b2Fixture next = f.m_next;
        // Remove fixture
        if (prev != null) {
          prev.m_next = next;
        } else {
          body1.m_fixtureList = next;
        }
        body1.m_fixtureCount--;

        // Add fixture
        f.m_next = body2.m_fixtureList;
        body2.m_fixtureList = f;
        body2.m_fixtureCount++;

        f.m_body = body2;

        f = next;
      } else {
        prev = f;
        f = f.m_next;
      }
    }

    body1.ResetMassData();
    body2.ResetMassData();

    // Compute consistent velocites for new bodies based on cached velocity
    b2Vec2 center1 = body1.GetWorldCenter();
    b2Vec2 center2 = body2.GetWorldCenter();

    b2Vec2 velocity1 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center1, center)));

    b2Vec2 velocity2 = b2Math.AddVV(linearVelocity, b2Math.CrossFV(angularVelocity, b2Math.SubtractVV(center2, center)));

    body1.SetLinearVelocity(velocity1);
    body2.SetLinearVelocity(velocity2);
    body1.SetAngularVelocity(angularVelocity);
    body2.SetAngularVelocity(angularVelocity);

    body1.SynchronizeFixtures();
    body2.SynchronizeFixtures();

    return body2;
  }

  /**
	 * Merges another body into this. Only fixtures, mass and velocity are effected,
	 * Other properties are ignored
	 * @asonly
	 */
  void Merge(b2Body other) {
    // Recalculate velocities
    b2Body body1 = this;
    b2Body body2 = other;

    b2Fixture f;
    for (f = other.m_fixtureList; f; ) {
      b2Fixture next = f.m_next;

      // Remove fixture
      other.m_fixtureCount--;

      // Add fixture
      f.m_next = m_fixtureList;
      m_fixtureList = f;
      m_fixtureCount++;

      f.m_body = body2;

      f = next;
    }
    body1.m_fixtureCount = 0;


    // Compute consistent velocites for new bodies based on cached velocity
    b2Vec2 center1 = body1.GetWorldCenter();
    b2Vec2 center2 = body2.GetWorldCenter();

    b2Vec2 velocity1 = body1.GetLinearVelocity().Copy();
    b2Vec2 velocity2 = body2.GetLinearVelocity().Copy();

    double angular1 = body1.GetAngularVelocity();
    double angular = body2.GetAngularVelocity();

    // TODO

    body1.ResetMassData();

    SynchronizeFixtures();
  }

  /**
	* Get the total mass of the body.
	* @return the mass, usually in kilograms (kg).
	*/
  double GetMass() {
    return m_mass;
  }

  /**
	* Get the central rotational inertia of the body.
	* @return the rotational inertia, usually in kg-m^2.
	*/
  double GetInertia() {
    return m_I;
  }

  /** 
	 * Get the mass data of the body. The rotational inertial is relative to the center of mass.
	 */
  void GetMassData(b2MassData data) {
    data.mass = m_mass;
    data.I = m_I;
    data.center.SetV(m_sweep.localCenter);
  }

  /**
	 * Set the mass properties to @override 
		the mass properties of the fixtures
	 * Note that this changes the center of mass position.
	 * Note that creating or destroying fixtures can also alter the mass.
	 * This function has no effect if the body isn't dynamic.
	 * @warning The supplied rotational inertia should be relative to the center of mass
	 * @param	data the mass properties.
	 */
  void SetMassData(b2MassData massData) {
    b2Settings.b2Assert(m_world.IsLocked() == false);
    if (m_world.IsLocked() == true) {
      return;
    }

    if (m_type != b2_dynamicBody) {
      return;
    }

    m_invMass = 0.0;
    m_I = 0.0;
    m_invI = 0.0;

    m_mass = massData.mass;

    // Compute the center of mass.
    if (m_mass <= 0.0) {
      m_mass = 1.0;
    }
    m_invMass = 1.0 / m_mass;

    if (massData.I > 0.0 && (m_flags & e_fixedRotationFlag) == 0) {
      // Center the inertia about the center of mass
      m_I = massData.I - m_mass * (massData.center.x * massData.center.x + massData.center.y * massData.center.y);
      m_invI = 1.0 / m_I;
    }

    // Move center of mass
    b2Vec2 oldCenter = m_sweep.c.Copy();
    m_sweep.localCenter.SetV(massData.center);
    m_sweep.c0.SetV(b2Math.MulX(m_xf, m_sweep.localCenter));
    m_sweep.c.SetV(m_sweep.c0);

    // Update center of mass velocity
    //m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
    m_linearVelocity.x += m_angularVelocity * -(m_sweep.c.y - oldCenter.y);
    m_linearVelocity.y += m_angularVelocity * (m_sweep.c.x - oldCenter.x);

  }

  /**
	 * This resets the mass properties to the sum of the mass properties of the fixtures.
	 * This normally does not need to be called unless you called SetMassData to @override
	 
		* the mass and later you want to reset the mass.
	 */
  void ResetMassData() {
    // Compute mass data from shapes. Each shape has it's own density
    m_mass = 0.0;
    m_invMass = 0.0;
    m_I = 0.0;
    m_invI = 0.0;
    m_sweep.localCenter.SetZero();

    // Static and kinematic bodies have zero mass.
    if (m_type == b2_staticBody || m_type == b2_kinematicBody) {
      return;
    }
    //b2Assert(m_type == b2_dynamicBody);

    // Accumulate mass over all fixtures.
    b2Vec2 center = b2Vec2.Make(0.0, 0.0);
    for (b2Fixture f = m_fixtureList; f != null; f = f.m_next) {
      if (f.m_density == 0.0) {
        continue;
      }

      b2MassData massData = f.GetMassData();
      m_mass += massData.mass;
      center.x += massData.center.x * massData.mass;
      center.y += massData.center.y * massData.mass;
      m_I += massData.I;
    }

    // Compute the center of mass.
    if (m_mass > 0.0) {
      m_invMass = 1.0 / m_mass;
      center.x *= m_invMass;
      center.y *= m_invMass;
    } else {
      // Force all dynamic bodies to have a positive mass.
      m_mass = 1.0;
      m_invMass = 1.0;
    }

    if (m_I > 0.0 && (m_flags & e_fixedRotationFlag) == 0) {
      // Center the inertia about the center of mass
      m_I -= m_mass * (center.x * center.x + center.y * center.y);
      m_I *= m_inertiaScale;
      b2Settings.b2Assert(m_I > 0);
      m_invI = 1.0 / m_I;
    } else {
      m_I = 0.0;
      m_invI = 0.0;
    }

    // Move center of mass
    b2Vec2 oldCenter = m_sweep.c.Copy();
    m_sweep.localCenter.SetV(center);
    m_sweep.c0.SetV(b2Math.MulX(m_xf, m_sweep.localCenter));
    m_sweep.c.SetV(m_sweep.c0);

    // Update center of mass velocity
    //m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
    m_linearVelocity.x += m_angularVelocity * -(m_sweep.c.y - oldCenter.y);
    m_linearVelocity.y += m_angularVelocity * (m_sweep.c.x - oldCenter.x);

  }

  /**
	 * Get the world coordinates of a point given the local coordinates.
	 * @param localPoint a point on the body measured relative the the body's origin.
	 * @return the same point expressed in world coordinates.
	 */
  b2Vec2 GetWorldPoint(b2Vec2 localPoint) {
    //return b2Math.b2MulX(m_xf, localPoint);
    b2Mat22 A = m_xf.R;
    b2Vec2 u = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
    u.x += m_xf.position.x;
    u.y += m_xf.position.y;
    return u;
  }

  /**
	 * Get the world coordinates of a vector given the local coordinates.
	 * @param localLista vector fixed in the body.
	 * @return the same vector expressed in world coordinates.
	 */
  b2Vec2 GetWorldVector(b2Vec2 localVector) {
    return b2Math.MulMV(m_xf.R, localVector);
  }

  /**
	 * Gets a local point relative to the body's origin given a world point.
	 * @param a point in world coordinates.
	 * @return the corresponding local point relative to the body's origin.
	 */
  b2Vec2 GetLocalPoint(b2Vec2 worldPoint) {
    return b2Math.MulXT(m_xf, worldPoint);
  }

  /**
	 * Gets a local vector given a world vector.
	 * @param a vector in world coordinates.
	 * @return the corresponding local vector.
	 */
  b2Vec2 GetLocalVector(b2Vec2 worldVector) {
    return b2Math.MulTMV(m_xf.R, worldVector);
  }

  /**
	* Get the world linear velocity of a world point attached to this body.
	* @param a point in world coordinates.
	* @return the world velocity of a point.
	*/
  b2Vec2 GetLinearVelocityFromWorldPoint(b2Vec2 worldPoint) {
    //return          m_linearVelocity   + b2Cross(m_angularVelocity,   worldPoint   - m_sweep.c);
    return new b2Vec2(m_linearVelocity.x - m_angularVelocity * (worldPoint.y - m_sweep.c.y), m_linearVelocity.y + m_angularVelocity * (worldPoint.x - m_sweep.c.x));
  }

  /**
	* Get the world velocity of a local point.
	* @param a point in local coordinates.
	* @return the world velocity of a point.
	*/
  b2Vec2 GetLinearVelocityFromLocalPoint(b2Vec2 localPoint) {
    //return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
    b2Mat22 A = m_xf.R;
    b2Vec2 worldPoint = new b2Vec2(A.col1.x * localPoint.x + A.col2.x * localPoint.y, A.col1.y * localPoint.x + A.col2.y * localPoint.y);
    worldPoint.x += m_xf.position.x;
    worldPoint.y += m_xf.position.y;
    return new b2Vec2(m_linearVelocity.x - m_angularVelocity * (worldPoint.y - m_sweep.c.y), m_linearVelocity.y + m_angularVelocity * (worldPoint.x - m_sweep.c.x));
  }

  /**
	* Get the linear damping of the body.
	*/
  double GetLinearDamping() {
    return m_linearDamping;
  }

  /**
	* Set the linear damping of the body.
	*/
  void SetLinearDamping(double linearDamping) {
    m_linearDamping = linearDamping;
  }

  /**
	* Get the angular damping of the body
	*/
  double GetAngularDamping() {
    return m_angularDamping;
  }

  /**
	* Set the angular damping of the body.
	*/
  void SetAngularDamping(double angularDamping) {
    m_angularDamping = angularDamping;
  }

  /**
	 * Set the type of this body. This may alter the mass and velocity
	 * @param	type - edouble stored as a static member of b2Body
	 */
  void SetType(int type) {
    if (m_type == type) {
      return;
    }

    m_type = type;

    ResetMassData();

    if (m_type == b2_staticBody) {
      m_linearVelocity.SetZero();
      m_angularVelocity = 0.0;
    }

    SetAwake(true);

    m_force.SetZero();
    m_torque = 0.0;

    // Since the body type changed, we need to flag contacts for filtering.
    for (b2ContactEdge ce = m_contactList; ce; ce = ce.next) {
      ce.contact.FlagForFiltering();
    }
  }

  /**
	 * Get the type of this body.
	 * @return type edouble as a int
	 */
  int GetType() {
    return m_type;
  }

  /**
	* Should this body be treated like a bullet for continuous collision detection?
	*/
  void SetBullet(bool flag) {
    if (flag) {
      m_flags |= e_bulletFlag;
    } else {
      m_flags &= ~e_bulletFlag;
    }
  }

  /**
	* Is this body treated like a bullet for continuous collision detection?
	*/
  bool IsBullet() {
    return (m_flags & e_bulletFlag) == e_bulletFlag;
  }

  /**
	 * Is this body allowed to sleep
	 * @param	flag
	 */
  void SetSleepingAllowed(bool flag) {
    if (flag) {
      m_flags |= e_allowSleepFlag;
    } else {
      m_flags &= ~e_allowSleepFlag;
      SetAwake(true);
    }
  }

  /**
	 * Set the sleep state of the body. A sleeping body has vety low CPU cost.
	 * @param	flag - set to true to put body to sleep, false to wake it
	 */
  void SetAwake(bool flag) {
    if (flag) {
      m_flags |= e_awakeFlag;
      m_sleepTime = 0.0;
    } else {
      m_flags &= ~e_awakeFlag;
      m_sleepTime = 0.0;
      m_linearVelocity.SetZero();
      m_angularVelocity = 0.0;
      m_force.SetZero();
      m_torque = 0.0;
    }
  }

  /**
	 * Get the sleeping state of this body.
	 * @return true if body is sleeping
	 */
  bool IsAwake() {
    return (m_flags & e_awakeFlag) == e_awakeFlag;
  }

  /**
	 * Set this body to have fixed rotation. This causes the mass to be reset.
	 * @param	fixed - true means no rotation
	 */
  void SetFixedRotation(bool fixed) {
    if (fixed == true) {
      m_flags |= e_fixedRotationFlag;
    } else {
      m_flags &= ~e_fixedRotationFlag;
    }

    ResetMassData();
  }

  /**
	* Does this body have fixed rotation?
	* @return true means fixed rotation
	*/
  bool IsFixedRotation() {
    return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
  }

  /** Set the active state of the body. An inactive body is not
	* simulated and cannot be collided with or woken up.
	* If you pass a flag of true, all fixtures will be added to the
	* broad-phase.
	* If you pass a flag of false, all fixtures will be removed from
	* the broad-phase and all contacts will be destroyed.
	* Fixtures and joints are otherwise unaffected. You may continue
	* to create/destroy fixtures and joints on inactive bodies.
	* Fixtures on an inactive body are implicitly inactive and will
	* not participate in collisions, ray-casts, or queries.
	* Joints connected to an inactive body are implicitly inactive.
	* An inactive body is still owned by a b2World object and remains
	* in the body list.
	*/
  void SetActive(bool flag) {
    if (flag == IsActive()) {
      return;
    }

    IBroadPhase broadPhase;
    b2Fixture f;
    if (flag) {
      m_flags |= e_activeFlag;

      // Create all proxies.
      broadPhase = m_world.m_contactManager.m_broadPhase;
      for (f = m_fixtureList; f; f = f.m_next) {
        f.CreateProxy(broadPhase, m_xf);
      }
      // Contacts are created the next time step.
    } else {
      m_flags &= ~e_activeFlag;

      // Destroy all proxies.
      broadPhase = m_world.m_contactManager.m_broadPhase;
      for (f = m_fixtureList; f; f = f.m_next) {
        f.DestroyProxy(broadPhase);
      }

      // Destroy the attached contacts.
      b2ContactEdge ce = m_contactList;
      while (ce != null) {
        b2ContactEdge ce0 = ce;
        ce = ce.next;
        m_world.m_contactManager.Destroy(ce0.contact);
      }
      m_contactList = null;
    }
  }

  /**
	 * Get the active state of the body.
	 * @return true if active.
	 */
  bool IsActive() {
    return (m_flags & e_activeFlag) == e_activeFlag;
  }

  /**
	* Is this body allowed to sleep?
	*/
  bool IsSleepingAllowed() {
    return (m_flags & e_allowSleepFlag) == e_allowSleepFlag;
  }

  /**
	* Get the list of all fixtures attached to this body.
	*/
  b2Fixture GetFixtureList() {
    return m_fixtureList;
  }

  /**
	* Get the list of all joints attached to this body.
	*/
  b2JointEdge GetJointList() {
    return m_jointList;
  }

  /**
	 * Get the list of all controllers attached to this body.
	 */
  b2ControllerEdge GetControllerList() {
    return m_controllerList;
  }

  /**
	 * Get a list of all contacts attached to this body.
	 */
  b2ContactEdge GetContactList() {
    return m_contactList;
  }

  /**
	* Get the next body in the world's body list.
	*/
  b2Body GetNext() {
    return m_next;
  }

  /**
	* Get the user data pointer that was provided in the body definition.
	*/
  dynamic GetUserData() {
    return m_userData;
  }

  /**
	* Set the user data. Use this to store your application specific data.
	*/
  void SetUserData(dynamic data) {
    m_userData = data;
  }

  /**
	* Get the parent world of this body.
	*/
  b2World GetWorld() {
    return m_world;
  }

  //--------------- Internals Below -------------------


  // Constructor
  /**
	 * @
	 */
  b2Body(b2BodyDef bd, b2World world) {
    //b2Settings.b2Assert(world.IsLocked() == false);

    //b2Settings.b2Assert(bd.position.IsValid());
    //b2Settings.b2Assert(bd.linearVelocity.IsValid());
    //b2Settings.b2Assert(b2Math.b2IsValid(bd.angle));
    //b2Settings.b2Assert(b2Math.b2IsValid(bd.angularVelocity));
    //b2Settings.b2Assert(b2Math.b2IsValid(bd.inertiaScale) && bd.inertiaScale >= 0.0);
    //b2Settings.b2Assert(b2Math.b2IsValid(bd.angularDamping) && bd.angularDamping >= 0.0);
    //b2Settings.b2Assert(b2Math.b2IsValid(bd.linearDamping) && bd.linearDamping >= 0.0);

    m_flags = 0;

    if (bd.bullet) {
      m_flags |= e_bulletFlag;
    }
    if (bd.fixedRotation) {
      m_flags |= e_fixedRotationFlag;
    }
    if (bd.allowSleep) {
      m_flags |= e_allowSleepFlag;
    }
    if (bd.awake) {
      m_flags |= e_awakeFlag;
    }
    if (bd.active) {
      m_flags |= e_activeFlag;
    }

    m_world = world;

    m_xf.position.SetV(bd.position);
    m_xf.R.Set(bd.angle);

    m_sweep.localCenter.SetZero();
    m_sweep.t0 = 1.0;
    m_sweep.a0 = m_sweep.a = bd.angle;

    //m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
    //b2MulMV(m_xf.R, m_sweep.localCenter);
    b2Mat22 tMat = m_xf.R;
    b2Vec2 tVec = m_sweep.localCenter;
    // (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y)
    m_sweep.c.x = (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    // (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y)
    m_sweep.c.y = (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
    //return T.position + b2Mul(T.R, v);
    m_sweep.c.x += m_xf.position.x;
    m_sweep.c.y += m_xf.position.y;
    //m_sweep.c0 = m_sweep.c
    m_sweep.c0.SetV(m_sweep.c);

    m_jointList = null;
    m_controllerList = null;
    m_contactList = null;
    m_controllerCount = 0;
    m_prev = null;
    m_next = null;

    m_linearVelocity.SetV(bd.linearVelocity);
    m_angularVelocity = bd.angularVelocity;

    m_linearDamping = bd.linearDamping;
    m_angularDamping = bd.angularDamping;

    m_force.Set(0.0, 0.0);
    m_torque = 0.0;

    m_sleepTime = 0.0;

    m_type = bd.type;

    if (m_type == b2_dynamicBody) {
      m_mass = 1.0;
      m_invMass = 1.0;
    } else {
      m_mass = 0.0;
      m_invMass = 0.0;
    }

    m_I = 0.0;
    m_invI = 0.0;

    m_inertiaScale = bd.inertiaScale;

    m_userData = bd.userData;

    m_fixtureList = null;
    m_fixtureCount = 0;
  }

  // Destructor
  //~b2Body();

  //
  static b2Transform s_xf1 = new b2Transform();
  //
  void SynchronizeFixtures() {

    b2Transform xf1 = s_xf1;
    xf1.R.Set(m_sweep.a0);
    //xf1.position = m_sweep.c0 - b2Mul(xf1.R, m_sweep.localCenter);
    b2Mat22 tMat = xf1.R;
    b2Vec2 tVec = m_sweep.localCenter;
    xf1.position.x = m_sweep.c0.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    xf1.position.y = m_sweep.c0.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);

    b2Fixture f;
    IBroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
    for (f = m_fixtureList; f != null; f = f.m_next) {
      f.Synchronize(broadPhase, xf1, m_xf);
    }
  }

  void SynchronizeTransform() {
    m_xf.R.Set(m_sweep.a);
    //m_xf.position = m_sweep.c - b2Mul(m_xf.R, m_sweep.localCenter);
    b2Mat22 tMat = m_xf.R;
    b2Vec2 tVec = m_sweep.localCenter;
    m_xf.position.x = m_sweep.c.x - (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
    m_xf.position.y = m_sweep.c.y - (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
  }

  // This is used to prevent connected bodies from colliding.
  // It may lie, depending on the collideConnected flag.
  bool ShouldCollide(b2Body other) {
    // At least one body should be dynamic
    if (m_type != b2_dynamicBody && other.m_type != b2_dynamicBody) {
      return false;
    }
    // Does a joint prevent collision?
    for (b2JointEdge jn = m_jointList; jn != null; jn = jn.next) {
      if (jn.other == other) if (jn.joint.m_collideConnected == false) {
        return false;
      }
    }

    return true;
  }

  void Advance(double t) {
    // Advance to the new safe time.
    m_sweep.Advance(t);
    m_sweep.c.SetV(m_sweep.c0);
    m_sweep.a = m_sweep.a0;
    SynchronizeTransform();
  }

  int m_flags = 0;
  int m_type = 0;

  int m_islandIndex = 0;

  b2Transform m_xf = new b2Transform(); // the body origin transform

  b2Sweep m_sweep = new b2Sweep(); // the swept motion for CCD

  b2Vec2 m_linearVelocity = new b2Vec2();
  double m_angularVelocity = 0.0;

  b2Vec2 m_force = new b2Vec2();
  double m_torque = 0.0;

  b2World m_world;
  b2Body m_prev;
  b2Body m_next;

  b2Fixture m_fixtureList;
  int m_fixtureCount = 0;

  b2ControllerEdge m_controllerList;
  int m_controllerCount = 0;

  b2JointEdge m_jointList;
  b2ContactEdge m_contactList;

  double m_mass, m_invMass;
  double m_I, m_invI;

  double m_inertiaScale = 0.0;

  double m_linearDamping = 0.0;
  double m_angularDamping = 0.0;

  double m_sleepTime = 0.0;

  dynamic m_userData;


  // m_flags
  //edouble
  //{
  static int e_islandFlag = 0x0001;
  static int e_awakeFlag = 0x0002;
  static int e_allowSleepFlag = 0x0004;
  static int e_bulletFlag = 0x0008;
  static int e_fixedRotationFlag = 0x0010;
  static int e_activeFlag = 0x0020;
  //}

  // m_type
  //edouble
  //{
  /// The body type.
  /// static: zero mass, zero velocity, may be manually moved
  /// kinematic: zero mass, non-zero velocity set by user, moved by solver
  /// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
  static const int b2_staticBody = 0;
  static const int b2_kinematicBody = 1;
  static const int b2_dynamicBody = 2;
  //}

}
