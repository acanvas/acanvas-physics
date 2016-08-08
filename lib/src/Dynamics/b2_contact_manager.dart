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

 part of rockdot_physics;






// Delegate of b2World.
/**
* @
*/
 class b2ContactManager 
{
	 b2ContactManager() {
		m_world = null;
		m_contactCount = 0;
		m_contactFilter = b2ContactFilter.b2_defaultFilter;
		m_contactListener = b2ContactListener.b2_defaultListener;
		m_contactFactory = new b2ContactFactory(m_allocator);
		m_broadPhase = new b2DynamicTreeBroadPhase();
	}

	// This is a callback from the broadphase when two AABB proxies begin
	// to overlap. We create a b2Contact to manage the narrow phase.
	  void AddPair(dynamic proxyUserDataA,dynamic proxyUserDataB) {
		b2Fixture fixtureA = proxyUserDataA as b2Fixture;
		b2Fixture fixtureB = proxyUserDataB as b2Fixture;
		
		b2Body bodyA = fixtureA.GetBody();
		b2Body bodyB = fixtureB.GetBody();
		
		// Are the fixtures on the same body?
		if (bodyA == bodyB)
			return;
		
		// Does a contact already exist?
		b2ContactEdge edge = bodyB.GetContactList();
		while (edge != null)
		{
			if (edge.other == bodyA)
			{
				b2Fixture fA = edge.contact.GetFixtureA();
				b2Fixture fB = edge.contact.GetFixtureB();
				if (fA == fixtureA && fB == fixtureB)
					return;
				if (fA == fixtureB && fB == fixtureA)
					return;
			}
			edge = edge.next;
		}
		
		//Does a joint override collision? Is at least one body dynamic?
		if (bodyB.ShouldCollide(bodyA) == false)
		{
			return;
		}
		
		// Check user filtering
		if (m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
		{
			return;
		}
		
		// Call the factory.
		b2Contact c = m_contactFactory.Create(fixtureA, fixtureB);
		
		// Contact creation may swap shapes.
		fixtureA = c.GetFixtureA();
		fixtureB = c.GetFixtureB();
		bodyA = fixtureA.m_body;
		bodyB = fixtureB.m_body;
		
		// Insert into the world.
		c.m_prev = null;
		c.m_next = m_world.m_contactList;
		if (m_world.m_contactList != null)
		{
			m_world.m_contactList.m_prev = c;
		}
		m_world.m_contactList = c;
		
		
		// Connect to island graph.
		
		// Connect to body A
		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;
		
		c.m_nodeA.prev = null;
		c.m_nodeA.next = bodyA.m_contactList;
		if (bodyA.m_contactList != null)
		{
			bodyA.m_contactList.prev = c.m_nodeA;
		}
		bodyA.m_contactList = c.m_nodeA;
		
		// Connect to body 2
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;
		
		c.m_nodeB.prev = null;
		c.m_nodeB.next = bodyB.m_contactList;
		if (bodyB.m_contactList != null)
		{
			bodyB.m_contactList.prev = c.m_nodeB;
		}
		bodyB.m_contactList = c.m_nodeB;
		
		++m_world.m_contactCount;
		return;
		
	}

	  void FindNewContacts()
	{
		m_broadPhase.UpdatePairs(AddPair);
	}
	
	static  final b2ContactPoint s_evalCP = new b2ContactPoint();
	  void Destroy(b2Contact c)
	{
		
		b2Fixture fixtureA = c.GetFixtureA();
		b2Fixture fixtureB = c.GetFixtureB();
		b2Body bodyA = fixtureA.GetBody();
		b2Body bodyB = fixtureB.GetBody();
		
		if (c.IsTouching())
		{
			m_contactListener.EndContact(c);
		}
		
		// Remove from the world.
		if (c.m_prev != null)
		{
			c.m_prev.m_next = c.m_next;
		}
		
		if (c.m_next != null)
		{
			c.m_next.m_prev = c.m_prev;
		}
		
		if (c == m_world.m_contactList)
		{
			m_world.m_contactList = c.m_next;
		}
		
		// Remove from body A
		if (c.m_nodeA.prev != null)
		{
			c.m_nodeA.prev.next = c.m_nodeA.next;
		}
		
		if (c.m_nodeA.next != null)
		{
			c.m_nodeA.next.prev = c.m_nodeA.prev;
		}
		
		if (c.m_nodeA == bodyA.m_contactList)
		{
			bodyA.m_contactList = c.m_nodeA.next;
		}
		
		// Remove from body 2
		if (c.m_nodeB.prev != null)
		{
			c.m_nodeB.prev.next = c.m_nodeB.next;
		}
		
		if (c.m_nodeB.next != null)
		{
			c.m_nodeB.next.prev = c.m_nodeB.prev;
		}
		
		if (c.m_nodeB == bodyB.m_contactList)
		{
			bodyB.m_contactList = c.m_nodeB.next;
		}
		
		// Call the factory.
		m_contactFactory.Destroy(c);
		--m_contactCount;
	}
	

	// This is the top level collision call for the time step. Here
	// all the narrow phase collision is processed for the world
	// contact list.
	  void Collide()
	{
		// Update awake contacts.
		b2Contact c = m_world.m_contactList;
		b2Contact cNuke;
		while (c != null)
		{
			b2Fixture fixtureA = c.GetFixtureA();
			b2Fixture fixtureB = c.GetFixtureB();
			b2Body bodyA = fixtureA.GetBody();
			b2Body bodyB = fixtureB.GetBody();
			if (bodyA.IsAwake() == false && bodyB.IsAwake() == false)
			{
				c = c.GetNext();
				continue;
			}
			
			// Is this contact flagged for filtering?
			if ((c.m_flags & b2Contact.e_filterFlag) > 0)
			{
				// Should these bodies collide?
				if (bodyB.ShouldCollide(bodyA) == false)
				{
					cNuke = c;
					c = cNuke.GetNext();
					Destroy(cNuke);
					continue;
				}
				
				// Check user filtering.
				if (m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
				{
					cNuke = c;
					c = cNuke.GetNext();
					Destroy(cNuke);
					continue;
				}
				
				// Clear the filtering flag
				c.m_flags &= ~b2Contact.e_filterFlag;
			}
			
			dynamic proxyA = fixtureA.m_proxy;
			dynamic proxyB = fixtureB.m_proxy;
			
			bool overlap = m_broadPhase.TestOverlap(proxyA, proxyB);
			
			// Here we destroy contacts that cease to overlap in the broadphase
			if ( overlap == false)
			{
				cNuke = c;
				c = cNuke.GetNext();
				Destroy(cNuke);
				continue;
			}
			
			c.Update(m_contactListener);
			c = c.GetNext();
		}
	}

	
	b2World m_world;
	IBroadPhase m_broadPhase;
	
	b2Contact m_contactList;
	int m_contactCount = 0;
	b2ContactFilter m_contactFilter;
	b2ContactListener m_contactListener;
	b2ContactFactory m_contactFactory;
	dynamic m_allocator;
}

