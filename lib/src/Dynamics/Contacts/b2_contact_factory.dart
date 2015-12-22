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






//typedef b2Contact* b2ContactCreateFcn(b2Shape* shape1, b2Shape* shape2, b2BlockAllocator* allocator);
//typedef void b2ContactDestroyFcn(b2Contact* contact, b2BlockAllocator* allocator);



/**
 * This class manages creation and destruction of b2Contact objects.
 * @
 */
 class b2ContactFactory
{
	 b2ContactFactory(dynamic allocator)
	{
		m_allocator = allocator;
		InitializeRegisters();
	}
	
	 void AddType(Function createFcn,Function destroyFcn,int type1,int type2)
	{
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		m_registers[type1][type2].createFcn = createFcn;
		m_registers[type1][type2].destroyFcn = destroyFcn;
		m_registers[type1][type2].primary = true;
		
		if (type1 != type2)
		{
			m_registers[type2][type1].createFcn = createFcn;
			m_registers[type2][type1].destroyFcn = destroyFcn;
			m_registers[type2][type1].primary = false;
		}
	}
	 void InitializeRegisters(){
		m_registers = new List<List<b2ContactRegister>>(b2Shape.e_shapeTypeCount);
		for (int i = 0; i < b2Shape.e_shapeTypeCount; i++){
			m_registers[i] = new List<b2ContactRegister>(b2Shape.e_shapeTypeCount);
			for (int j = 0; j < b2Shape.e_shapeTypeCount; j++){
				m_registers[i][j] = new b2ContactRegister();
			}
		}
		
		AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
		AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
		AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);
		
		AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape);
		AddType(b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape);
	}
	  b2Contact Create(b2Fixture fixtureA,b2Fixture fixtureB){
		int type1 = fixtureA.GetType();
		int type2 = fixtureB.GetType();
		
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		b2ContactRegister reg = m_registers[type1][type2];
		
		b2Contact c;
		
		if (reg.pool != null)
		{
			// Pop a contact off the pool
			c = reg.pool;
			reg.pool = c.m_next;
			reg.poolCount--;
			c.Reset(fixtureA, fixtureB);
			return c;
		}
		
		Function createFcn = reg.createFcn;
		if (createFcn != null)
		{
			if (reg.primary)
			{
				c = createFcn(m_allocator);
				c.Reset(fixtureA, fixtureB);
				return c;
			}
			else
			{
				c = createFcn(m_allocator);
				c.Reset(fixtureB, fixtureA);
				return c;
			}
		}
		else
		{
			return null;
		}
	}
	  void Destroy(b2Contact contact){
		if (contact.m_manifold.m_pointCount > 0)
		{
			contact.m_fixtureA.m_body.SetAwake(true);
			contact.m_fixtureB.m_body.SetAwake(true);
		}
		
		int type1 = contact.m_fixtureA.GetType();
		int type2 = contact.m_fixtureB.GetType();
		
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type1 && type1 < b2Shape.e_shapeTypeCount);
		//b2Settings.b2Assert(b2Shape.e_unknownShape < type2 && type2 < b2Shape.e_shapeTypeCount);
		
		b2ContactRegister reg = m_registers[type1][type2];
		
		if( true )
		{
			reg.poolCount++;
			contact.m_next = reg.pool;
			reg.pool = contact;
		}
		
		Function destroyFcn = reg.destroyFcn;
		destroyFcn(contact, m_allocator);
	}

	
	 List<List<b2ContactRegister>> m_registers;
	 dynamic m_allocator;
}


