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





/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture
 * inherits its transform from its parent. Fixtures hold additional non-geometric data
 * such as friction, collision filters, etc.
 * Fixtures are created via b2Body::CreateFixture.
 * @warning you cannot reuse fixtures.
 */
 class b2Fixture
{
	/**
	 * Get the type of the child shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	  int GetType()
	{
		return m_shape.GetType();
	}
	
	/**
	 * Get the child shape. You can modify the child shape, however you should not change the
	 * doubleber of vertices because this will crash some collision caching mechanisms.
	 */
	  b2Shape GetShape()
	{
		return m_shape;
	}
	
	/**
	 * Set if this fixture is a sensor.
	 */
	  void SetSensor(bool sensor)
	{
		if ( m_isSensor == sensor)
			return;
			
		m_isSensor = sensor;
		
		if (m_body == null)
			return;
			
		b2ContactEdge edge = m_body.GetContactList();
		while (edge != null)
		{
			b2Contact contact = edge.contact;
			b2Fixture fixtureA = contact.GetFixtureA();
			b2Fixture fixtureB = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
			edge = edge.next;
		}
		
	}
	
	/**
	 * Is this fixture a sensor (non-solid)?
	 * @return the true if the shape is a sensor.
	 */
	  bool IsSensor()
	{
		return m_isSensor;
	}
	
	/**
	 * Set the contact filtering data. This will not update contacts until the next time
	 * step when either parent body is active and awake.
	 */
	  void SetFilterData(b2FilterData filter)
	{
		m_filter = filter.Copy();
		
		if (m_body != null)
			return;
			
		b2ContactEdge edge = m_body.GetContactList();
		while (edge != null)
		{
			b2Contact contact = edge.contact;
			b2Fixture fixtureA = contact.GetFixtureA();
			b2Fixture fixtureB = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.FlagForFiltering();
			edge = edge.next;
		}
	}
	
	/**
	 * Get the contact filtering data.
	 */
	  b2FilterData GetFilterData()
	{
		return m_filter.Copy();
	}
	
	/**
	 * Get the parent body of this fixture. This is NULL if the fixture is not attached.
	 * @return the parent body.
	 */
	  b2Body GetBody()
	{
		return m_body;
	}
	
	/**
	 * Get the next fixture in the parent body's fixture list.
	 * @return the next shape.
	 */
	  b2Fixture GetNext()
	{
		return m_next;
	}
	
	/**
	 * Get the user data that was assigned in the fixture definition. Use this to
	 * store your application specific data.
	 */
	  dynamic GetUserData()
	{
		return m_userData;
	}
	
	/**
	 * Set the user data. Use this to store your application specific data.
	 */
	  void SetUserData(dynamic data)
	{
		m_userData = data;
	}
	
	/**
	 * Test a point for containment in this fixture.
	 * @param xf the shape world transform.
	 * @param p a point in world coordinates.
	 */
	  bool TestPoint(b2Vec2 p)
	{
		return m_shape.TestPoint(m_body.GetTransform(), p);
	}
	
	/**
	 * Perform a ray cast against this shape.
	 * @param output the ray-cast results.
	 * @param input the ray-cast input parameters.
	 */
	  bool RayCast(b2RayCastOutput output,b2RayCastInput input)
	{
		return m_shape.RayCast(output, input, m_body.GetTransform());
	}
	
	/**
	 * Get the mass data for this fixture. The mass data is based on the density and
	 * the shape. The rotational inertia is about the shape's origin. This operation may be expensive
	 * @param massData - this is a reference to a valid massData, if it is null a new b2MassData is allocated and then returned
	 * @note if the input is null then you must get the return value.
	 */
	  b2MassData GetMassData([b2MassData massData=null])
	{
		if ( massData == null )
		{
			massData = new b2MassData();
		}
		m_shape.ComputeMass(massData, m_density);
		return massData;
	}
	
	/**
	 * Set the density of this fixture. This will _not_ automatically adjust the mass
	 * of the body. You must call b2Body::ResetMassData to update the body's mass.
	 * @param	density
	 */
	  void SetDensity(double density) {
		//b2Settings.b2Assert(b2Math.b2IsValid(density) && density >= 0.0);
		m_density = density;
	}
	
	/**
	 * Get the density of this fixture.
	 * @return density
	 */
	  double GetDensity() {
		return m_density;
	}
	
	/**
	 * Get the coefficient of friction.
	 */
	  double GetFriction()
	{
		return m_friction;
	}
	
	/**
	 * Set the coefficient of friction.
	 */
	  void SetFriction(double friction)
	{
		m_friction = friction;
	}
	
	/**
	 * Get the coefficient of restitution.
	 */
	  double GetRestitution()
	{
		return m_restitution;
	}
	
	/**
	 * Get the coefficient of restitution.
	 */
	  void SetRestitution(double restitution)
	{
		m_restitution = restitution;
	}
	
	/**
	 * Get the fixture's AABB. This AABB may be enlarge and/or stale.
	 * If you need a more accurate AABB, compute it using the shape and
	 * the body transform.
	 * @return
	 */
	  b2AABB GetAABB() {
		return m_aabb;
	}
	
	/**
	 * @
	 */
	 b2Fixture()
	{
		m_aabb = new b2AABB();
		m_userData = null;
		m_body = null;
		m_next = null;
		//m_proxyId = b2BroadPhase.e_nullProxy;
		m_shape = null;
		m_density = 0.0;
		
		m_friction = 0.0;
		m_restitution = 0.0;
	}
	
	/**
	 * the destructor cannot access the allocator (no destructor arguments allowed by C++).
	 *  We need separation create/destroy functions from the constructor/destructor because
	 */
	 void Create(b2Body body,b2Transform xf,b2FixtureDef def)
	{
		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;
		
		m_body = body;
		m_next = null;
		
		m_filter = def.filter.Copy();
		
		m_isSensor = def.isSensor;
		
		m_shape = def.shape.Copy();
		
		m_density = def.density;
	}
	
	/**
	 * the destructor cannot access the allocator (no destructor arguments allowed by C++).
	 *  We need separation create/destroy functions from the constructor/destructor because
	 */
	 void Destroy()
	{
		// The proxy must be destroyed before calling this.
		//b2Assert(m_proxyId == b2BroadPhase::e_nullProxy);
		
		// Free the child shape
		m_shape = null;
	}
	
	/**
	 * This supports body activation/deactivation.
	 */ 
	 void CreateProxy(IBroadPhase broadPhase,b2Transform xf) {
		//b2Assert(m_proxyId == b2BroadPhase::e_nullProxy);
		
		// Create proxy in the broad-phase.
		m_shape.ComputeAABB(m_aabb, xf);
		m_proxy = broadPhase.CreateProxy(m_aabb, this);
	}
	
	/**
	 * This supports body activation/deactivation.
	 */
	 void DestroyProxy(IBroadPhase broadPhase) {
		if (m_proxy == null)
		{
			return;
		}
		
		// Destroy proxy in the broad-phase.
		broadPhase.DestroyProxy(m_proxy);
		m_proxy = null;
	}
	
	 void Synchronize(IBroadPhase broadPhase,b2Transform transform1,b2Transform transform2)
	{
		if (m_proxy == null)
			return;
			
		// Compute an AABB that ocvers the swept shape (may miss some rotation effect)
		b2AABB aabb1 = new b2AABB();
		b2AABB aabb2 = new b2AABB();
		m_shape.ComputeAABB(aabb1, transform1);
		m_shape.ComputeAABB(aabb2, transform2);
		
		m_aabb.CombineI(aabb1, aabb2);
		b2Vec2 displacement = b2Math.SubtractVV(transform2.position, transform1.position);
		broadPhase.MoveProxy(m_proxy, m_aabb, displacement);
	}
	
	 b2MassData m_massData;
	
	b2AABB m_aabb;
	double m_density = 0.0;
	b2Fixture m_next;
	b2Body m_body;
	b2Shape m_shape;
	
	double m_friction = 0.0;
	double m_restitution = 0.0;
	
	dynamic m_proxy;
	b2FilterData m_filter = new b2FilterData();
	
	bool m_isSensor;
	
	dynamic m_userData;
}



