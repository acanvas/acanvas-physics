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
	
	
	






/**
* A gear joint is used to connect two joints together. Either joint
* can be a revolute or prismatic joint. You specify a gear ratio
* to bind the motions together: dynamic coordinate1 + ratio * coordinate2 = constant
* The ratio can be negative or positive. If one joint is a revolute joint
* and the other joint is a prismatic joint, then the ratio will have units
* of length or units of 1/length.
* @warning The revolute and prismatic joints must be attached to
* fixed bodies (which must be body1 on those joints).
* @see b2GearJointDef
*/

 class b2GearJoint extends b2Joint
{
	/** @inheritDoc */
	 @override 
		 b2Vec2 GetAnchorA(){
		//return m_bodyA->GetWorldPoint(m_localAnchor1);
		return m_bodyA.GetWorldPoint(m_localAnchor1);
	}
	/** @inheritDoc */
	 @override 
		 b2Vec2 GetAnchorB(){
		//return m_bodyB->GetWorldPoint(m_localAnchor2);
		return m_bodyB.GetWorldPoint(m_localAnchor2);
	}
	/** @inheritDoc */
	 @override 
		 b2Vec2 GetReactionForce(double inv_dt){
		// TODO_ERIN not tested
		// b2Vec2 P = m_impulse * m_J.linear2;
		//return inv_dt * P;
		return new b2Vec2(inv_dt * m_impulse * m_J.linearB.x, inv_dt * m_impulse * m_J.linearB.y);
	}
	/** @inheritDoc */
	 @override 
		 double GetReactionTorque(double inv_dt){
		// TODO_ERIN not tested
		//b2Vec2 r = b2Mul(m_bodyB->m_xf.R, m_localAnchor2 - m_bodyB->GetLocalCenter());
		b2Mat22 tMat = m_bodyB.m_xf.R;
		double rX = m_localAnchor1.x - m_bodyB.m_sweep.localCenter.x;
		double rY = m_localAnchor1.y - m_bodyB.m_sweep.localCenter.y;
		double tX = tMat.col1.x * rX + tMat.col2.x * rY;
		rY = tMat.col1.y * rX + tMat.col2.y * rY;
		rX = tX;
		//b2Vec2 P = m_impulse * m_J.linearB;
		double PX = m_impulse * m_J.linearB.x;
		double PY = m_impulse * m_J.linearB.y;
		//float32 L = m_impulse * m_J.angularB - b2Cross(r, P);
		//return inv_dt * L;
		return inv_dt * (m_impulse * m_J.angularB - rX * PY + rY * PX);
	}

	/**
	 * Get the gear ratio.
	 */
	  double GetRatio(){
		return m_ratio;
	}
	
	/**
	 * Set the gear ratio.
	 */
	  void SetRatio(double ratio) {
		//b2Settings.b2Assert(b2Math.b2IsValid(ratio));
		m_ratio = ratio;
	}

	//--------------- Internals Below -------------------

	/** @ */
	 b2GearJoint(b2GearJointDef def):super(def){
		// parent constructor
		
		int type1 = def.joint1.m_type;
		int type2 = def.joint2.m_type;
		
		//b2Settings.b2Assert(type1 == b2Joint.e_revoluteJoint || type1 == b2Joint.e_prismaticJoint);
		//b2Settings.b2Assert(type2 == b2Joint.e_revoluteJoint || type2 == b2Joint.e_prismaticJoint);
		//b2Settings.b2Assert(def.joint1.GetBodyA().GetType() == b2Body.b2_staticBody);
		//b2Settings.b2Assert(def.joint2.GetBodyA().GetType() == b2Body.b2_staticBody);
		
		m_revolute1 = null;
		m_prismatic1 = null;
		m_revolute2 = null;
		m_prismatic2 = null;
		
		double coordinate1 = 0.0;
		double coordinate2 = 0.0;
		
		m_ground1 = def.joint1.GetBodyA();
		m_bodyA = def.joint1.GetBodyB();
		if (type1 == b2Joint.e_revoluteJoint)
		{
			m_revolute1 = def.joint1 as b2RevoluteJoint;
			m_groundAnchor1.SetV( m_revolute1.m_localAnchor1 );
			m_localAnchor1.SetV( m_revolute1.m_localAnchor2 );
			coordinate1 = m_revolute1.GetJointAngle();
		}
		else
		{
			m_prismatic1 = def.joint1 as b2PrismaticJoint;
			m_groundAnchor1.SetV( m_prismatic1.m_localAnchor1 );
			m_localAnchor1.SetV( m_prismatic1.m_localAnchor2 );
			coordinate1 = m_prismatic1.GetJointTranslation();
		}
		
		m_ground2 = def.joint2.GetBodyA();
		m_bodyB = def.joint2.GetBodyB();
		if (type2 == b2Joint.e_revoluteJoint)
		{
			m_revolute2 = def.joint2 as b2RevoluteJoint;
			m_groundAnchor2.SetV( m_revolute2.m_localAnchor1 );
			m_localAnchor2.SetV( m_revolute2.m_localAnchor2 );
			coordinate2 = m_revolute2.GetJointAngle();
		}
		else
		{
			m_prismatic2 = def.joint2 as b2PrismaticJoint;
			m_groundAnchor2.SetV( m_prismatic2.m_localAnchor1 );
			m_localAnchor2.SetV( m_prismatic2.m_localAnchor2 );
			coordinate2 = m_prismatic2.GetJointTranslation();
		}
		
		m_ratio = def.ratio;
		
		m_constant = coordinate1 + m_ratio * coordinate2;
		
		m_impulse = 0.0;
		
	}

	@override 
		 void InitVelocityConstraints(b2TimeStep step){
		b2Body g1 = m_ground1;
		b2Body g2 = m_ground2;
		b2Body bA = m_bodyA;
		b2Body bB = m_bodyB;
		
		// temp vars
		double ugX = 0.0;
		double ugY = 0.0;
		double rX = 0.0;
		double rY = 0.0;
		b2Mat22 tMat;
		b2Vec2 tVec;
		double crug = 0.0;
		double tX = 0.0;
		
		double K = 0.0;
		m_J.SetZero();
		
		if (m_revolute1 != null)
		{
			m_J.angularA = -1.0;
			K += bA.m_invI;
		}
		else
		{
			//b2Vec2 ug = b2MulMV(g1->m_xf.R, m_prismatic1->m_localXAxis1);
			tMat = g1.m_xf.R;
			tVec = m_prismatic1.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//b2Vec2 r = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
			tMat = bA.m_xf.R;
			rX = m_localAnchor1.x - bA.m_sweep.localCenter.x;
			rY = m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;
			
			//double crug = b2Cross(r, ug);
			crug = rX * ugY - rY * ugX;
			//m_J.linearA = -ug;
			m_J.linearA.Set(-ugX, -ugY);
			m_J.angularA = -crug;
			K += bA.m_invMass + bA.m_invI * crug * crug;
		}
		
		if (m_revolute2 != null)
		{
			m_J.angularB = -m_ratio;
			K += m_ratio * m_ratio * bB.m_invI;
		}
		else
		{
			//b2Vec2 ug = b2Mul(g2->m_xf.R, m_prismatic2->m_localXAxis1);
			tMat = g2.m_xf.R;
			tVec = m_prismatic2.m_localXAxis1;
			ugX = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
			ugY = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
			//b2Vec2 r = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
			tMat = bB.m_xf.R;
			rX = m_localAnchor2.x - bB.m_sweep.localCenter.x;
			rY = m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = tMat.col1.x * rX + tMat.col2.x * rY;
			rY = tMat.col1.y * rX + tMat.col2.y * rY;
			rX = tX;
			
			//float32 crug = b2Cross(r, ug);
			crug = rX * ugY - rY * ugX;
			//m_J.linearB = -m_ratio * ug;
			m_J.linearB.Set(-m_ratio*ugX, -m_ratio*ugY);
			m_J.angularB = -m_ratio * crug;
			K += m_ratio * m_ratio * (bB.m_invMass + bB.m_invI * crug * crug);
		}
		
		// Compute effective mass.
		m_mass = K > 0.0?1.0 / K:0.0;
		
		if (step.warmStarting)
		{
			// Warm starting.
			//bA.m_linearVelocity += bA.m_invMass * m_impulse * m_J.linearA;
			bA.m_linearVelocity.x += bA.m_invMass * m_impulse * m_J.linearA.x;
			bA.m_linearVelocity.y += bA.m_invMass * m_impulse * m_J.linearA.y;
			bA.m_angularVelocity += bA.m_invI * m_impulse * m_J.angularA;
			//bB.m_linearVelocity += bB.m_invMass * m_impulse * m_J.linearB;
			bB.m_linearVelocity.x += bB.m_invMass * m_impulse * m_J.linearB.x;
			bB.m_linearVelocity.y += bB.m_invMass * m_impulse * m_J.linearB.y;
			bB.m_angularVelocity += bB.m_invI * m_impulse * m_J.angularB;
		}
		else
		{
			m_impulse = 0.0;
		}
	}
	
	@override 
		 void SolveVelocityConstraints(b2TimeStep step)
	{
		//B2_NOT_USED(step);
		
		b2Body bA = m_bodyA;
		b2Body bB = m_bodyB;
		
		double Cdot = m_J.Compute(	bA.m_linearVelocity, bA.m_angularVelocity,
										bB.m_linearVelocity, bB.m_angularVelocity);
		
		double impulse = - m_mass * Cdot;
		m_impulse += impulse;
		
		bA.m_linearVelocity.x += bA.m_invMass * impulse * m_J.linearA.x;
		bA.m_linearVelocity.y += bA.m_invMass * impulse * m_J.linearA.y;
		bA.m_angularVelocity  += bA.m_invI * impulse * m_J.angularA;
		bB.m_linearVelocity.x += bB.m_invMass * impulse * m_J.linearB.x;
		bB.m_linearVelocity.y += bB.m_invMass * impulse * m_J.linearB.y;
		bB.m_angularVelocity  += bB.m_invI * impulse * m_J.angularB;
	}
	
	@override 
		 bool SolvePositionConstraints(double baumgarte)
	{
		//B2_NOT_USED(baumgarte);
		
		double linearError = 0.0;
		
		b2Body bA = m_bodyA;
		b2Body bB = m_bodyB;
		
		double coordinate1 = 0.0;
		double coordinate2 = 0.0;
		if (m_revolute1 != null)
		{
			coordinate1 = m_revolute1.GetJointAngle();
		}
		else
		{
			coordinate1 = m_prismatic1.GetJointTranslation();
		}
		
		if (m_revolute2 != null)
		{
			coordinate2 = m_revolute2.GetJointAngle();
		}
		else
		{
			coordinate2 = m_prismatic2.GetJointTranslation();
		}
		
		double C = m_constant - (coordinate1 + m_ratio * coordinate2);
		
		double impulse = -m_mass * C;
		
		bA.m_sweep.c.x += bA.m_invMass * impulse * m_J.linearA.x;
		bA.m_sweep.c.y += bA.m_invMass * impulse * m_J.linearA.y;
		bA.m_sweep.a += bA.m_invI * impulse * m_J.angularA;
		bB.m_sweep.c.x += bB.m_invMass * impulse * m_J.linearB.x;
		bB.m_sweep.c.y += bB.m_invMass * impulse * m_J.linearB.y;
		bB.m_sweep.a += bB.m_invI * impulse * m_J.angularB;
		
		bA.SynchronizeTransform();
		bB.SynchronizeTransform();
		
		// TODO_ERIN not implemented
		return linearError < b2Settings.b2_linearSlop;
	}

	 b2Body m_ground1;
	 b2Body m_ground2;

	// One of these is NULL.
	 b2RevoluteJoint m_revolute1;
	 b2PrismaticJoint m_prismatic1;

	// One of these is NULL.
	 b2RevoluteJoint m_revolute2;
	 b2PrismaticJoint m_prismatic2;

	 b2Vec2 m_groundAnchor1 = new b2Vec2();
	 b2Vec2 m_groundAnchor2 = new b2Vec2();

	 b2Vec2 m_localAnchor1 = new b2Vec2();
	 b2Vec2 m_localAnchor2 = new b2Vec2();

	 b2Jacobian m_J = new b2Jacobian();

	 double m_constant = 0.0;
	 double m_ratio = 0.0;

	// Effective mass
	 double m_mass = 0.0;

	// Impulse for accumulation/warm starting.
	 double m_impulse = 0.0;
}


