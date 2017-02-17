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
* The pulley joint is connected to two bodies and two fixed ground points.
* The pulley supports a ratio such that: dynamic length1 + ratio * length2 <= constant
* Yes, the force transmitted is scaled by the ratio.
* The pulley also enforces a maximum length limit on both sides. This is
* useful to prevent one side of the pulley hitting the top.
* @see b2PulleyJointDef
*/
class b2PulleyJoint extends b2Joint {
  /** @inheritDoc */
  @override
  b2Vec2 GetAnchorA() {
    return m_bodyA.GetWorldPoint(m_localAnchor1);
  }

  /** @inheritDoc */
  @override
  b2Vec2 GetAnchorB() {
    return m_bodyB.GetWorldPoint(m_localAnchor2);
  }

  /** @inheritDoc */
  @override
  b2Vec2 GetReactionForce(double inv_dt) {
    //b2Vec2 P = m_impulse * m_u2;
    //return inv_dt * P;
    return new b2Vec2(inv_dt * m_impulse * m_u2.x, inv_dt * m_impulse * m_u2.y);
  }

  /** @inheritDoc */
  @override
  double GetReactionTorque(double inv_dt) {
    //B2_NOT_USED(inv_dt);
    return 0.0;
  }

  /**
	 * Get the first ground anchor.
	 */
  b2Vec2 GetGroundAnchorA() {
    //return m_ground.m_xf.position + m_groundAnchor1;
    b2Vec2 a = m_ground.m_xf.position.Copy();
    a.Add(m_groundAnchor1);
    return a;
  }

  /**
	 * Get the second ground anchor.
	 */
  b2Vec2 GetGroundAnchorB() {
    //return m_ground.m_xf.position + m_groundAnchor2;
    b2Vec2 a = m_ground.m_xf.position.Copy();
    a.Add(m_groundAnchor2);
    return a;
  }

  /**
	 * Get the current length of the segment attached to body1.
	 */
  double GetLength1() {
    b2Vec2 p = m_bodyA.GetWorldPoint(m_localAnchor1);
    //b2Vec2 s = m_ground->m_xf.position + m_groundAnchor1;
    double sX = m_ground.m_xf.position.x + m_groundAnchor1.x;
    double sY = m_ground.m_xf.position.y + m_groundAnchor1.y;
    //b2Vec2 d = p - s;
    double dX = p.x - sX;
    double dY = p.y - sY;
    //return d.Length();
    return sqrt(dX * dX + dY * dY);
  }

  /**
	 * Get the current length of the segment attached to body2.
	 */
  double GetLength2() {
    b2Vec2 p = m_bodyB.GetWorldPoint(m_localAnchor2);
    //b2Vec2 s = m_ground->m_xf.position + m_groundAnchor2;
    double sX = m_ground.m_xf.position.x + m_groundAnchor2.x;
    double sY = m_ground.m_xf.position.y + m_groundAnchor2.y;
    //b2Vec2 d = p - s;
    double dX = p.x - sX;
    double dY = p.y - sY;
    //return d.Length();
    return sqrt(dX * dX + dY * dY);
  }

  /**
	 * Get the pulley ratio.
	 */
  double GetRatio() {
    return m_ratio;
  }

  //--------------- Internals Below -------------------

  /** @ */
  b2PulleyJoint(b2PulleyJointDef def) : super(def) {
    // parent

    b2Mat22 tMat;
    double tX = 0.0;
    double tY = 0.0;

    m_ground = m_bodyA.m_world.m_groundBody;
    //m_groundAnchor1 = def->groundAnchorA - m_ground->m_xf.position;
    m_groundAnchor1.x = def.groundAnchorA.x - m_ground.m_xf.position.x;
    m_groundAnchor1.y = def.groundAnchorA.y - m_ground.m_xf.position.y;
    //m_groundAnchor2 = def->groundAnchorB - m_ground->m_xf.position;
    m_groundAnchor2.x = def.groundAnchorB.x - m_ground.m_xf.position.x;
    m_groundAnchor2.y = def.groundAnchorB.y - m_ground.m_xf.position.y;
    //m_localAnchor1 = def->localAnchorA;
    m_localAnchor1.SetV(def.localAnchorA);
    //m_localAnchor2 = def->localAnchorB;
    m_localAnchor2.SetV(def.localAnchorB);

    //b2Settings.b2Assert(def.ratio != 0.0);
    m_ratio = def.ratio;

    m_constant = def.lengthA + m_ratio * def.lengthB;

    m_maxLength1 = b2Math.Min(def.maxLengthA, m_constant - m_ratio * b2_minPulleyLength);
    m_maxLength2 = b2Math.Min(def.maxLengthB, (m_constant - b2_minPulleyLength) / m_ratio);

    m_impulse = 0.0;
    m_limitImpulse1 = 0.0;
    m_limitImpulse2 = 0.0;
  }

  @override
  void InitVelocityConstraints(b2TimeStep step) {
    b2Body bA = m_bodyA;
    b2Body bB = m_bodyB;

    b2Mat22 tMat;

    //b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
    tMat = bA.m_xf.R;
    double r1X = m_localAnchor1.x - bA.m_sweep.localCenter.x;
    double r1Y = m_localAnchor1.y - bA.m_sweep.localCenter.y;
    double tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
    r1X = tX;
    //b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
    tMat = bB.m_xf.R;
    double r2X = m_localAnchor2.x - bB.m_sweep.localCenter.x;
    double r2Y = m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
    r2X = tX;

    //b2Vec2 p1 = bA->m_sweep.c + r1;
    double p1X = bA.m_sweep.c.x + r1X;
    double p1Y = bA.m_sweep.c.y + r1Y;
    //b2Vec2 p2 = bB->m_sweep.c + r2;
    double p2X = bB.m_sweep.c.x + r2X;
    double p2Y = bB.m_sweep.c.y + r2Y;

    //b2Vec2 s1 = m_ground->m_xf.position + m_groundAnchor1;
    double s1X = m_ground.m_xf.position.x + m_groundAnchor1.x;
    double s1Y = m_ground.m_xf.position.y + m_groundAnchor1.y;
    //b2Vec2 s2 = m_ground->m_xf.position + m_groundAnchor2;
    double s2X = m_ground.m_xf.position.x + m_groundAnchor2.x;
    double s2Y = m_ground.m_xf.position.y + m_groundAnchor2.y;

    // Get the pulley axes.
    //m_u1 = p1 - s1;
    m_u1.Set(p1X - s1X, p1Y - s1Y);
    //m_u2 = p2 - s2;
    m_u2.Set(p2X - s2X, p2Y - s2Y);

    double length1 = m_u1.Length();
    double length2 = m_u2.Length();

    if (length1 > b2Settings.b2_linearSlop) {
      //m_u1 *= 1.0f / length1;
      m_u1.Multiply(1.0 / length1);
    } else {
      m_u1.SetZero();
    }

    if (length2 > b2Settings.b2_linearSlop) {
      //m_u2 *= 1.0f / length2;
      m_u2.Multiply(1.0 / length2);
    } else {
      m_u2.SetZero();
    }

    double C = m_constant - length1 - m_ratio * length2;
    if (C > 0.0) {
      m_state = b2Joint.e_inactiveLimit;
      m_impulse = 0.0;
    } else {
      m_state = b2Joint.e_atUpperLimit;
    }

    if (length1 < m_maxLength1) {
      m_limitState1 = b2Joint.e_inactiveLimit;
      m_limitImpulse1 = 0.0;
    } else {
      m_limitState1 = b2Joint.e_atUpperLimit;
    }

    if (length2 < m_maxLength2) {
      m_limitState2 = b2Joint.e_inactiveLimit;
      m_limitImpulse2 = 0.0;
    } else {
      m_limitState2 = b2Joint.e_atUpperLimit;
    }

    // Compute effective mass.
    //double cr1u1 = b2Cross(r1, m_u1);
    double cr1u1 = r1X * m_u1.y - r1Y * m_u1.x;
    //double cr2u2 = b2Cross(r2, m_u2);
    double cr2u2 = r2X * m_u2.y - r2Y * m_u2.x;

    m_limitMass1 = bA.m_invMass + bA.m_invI * cr1u1 * cr1u1;
    m_limitMass2 = bB.m_invMass + bB.m_invI * cr2u2 * cr2u2;
    m_pulleyMass = m_limitMass1 + m_ratio * m_ratio * m_limitMass2;
    //b2Settings.b2Assert(m_limitMass1 > double.MIN_VALUE);
    //b2Settings.b2Assert(m_limitMass2 > double.MIN_VALUE);
    //b2Settings.b2Assert(m_pulleyMass > double.MIN_VALUE);
    m_limitMass1 = 1.0 / m_limitMass1;
    m_limitMass2 = 1.0 / m_limitMass2;
    m_pulleyMass = 1.0 / m_pulleyMass;

    if (step.warmStarting) {
      // Scale impulses to support variable time steps.
      m_impulse *= step.dtRatio;
      m_limitImpulse1 *= step.dtRatio;
      m_limitImpulse2 *= step.dtRatio;

      // Warm starting.
      //b2Vec2 P1 = (-m_impulse - m_limitImpulse1) * m_u1;
      double P1X = (-m_impulse - m_limitImpulse1) * m_u1.x;
      double P1Y = (-m_impulse - m_limitImpulse1) * m_u1.y;
      //b2Vec2 P2 = (-m_ratio * m_impulse - m_limitImpulse2) * m_u2;
      double P2X = (-m_ratio * m_impulse - m_limitImpulse2) * m_u2.x;
      double P2Y = (-m_ratio * m_impulse - m_limitImpulse2) * m_u2.y;
      //bA.m_linearVelocity += bA.m_invMass * P1;
      bA.m_linearVelocity.x += bA.m_invMass * P1X;
      bA.m_linearVelocity.y += bA.m_invMass * P1Y;
      //bA.m_angularVelocity += bA.m_invI * b2Cross(r1, P1);
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
      //bB.m_linearVelocity += bB.m_invMass * P2;
      bB.m_linearVelocity.x += bB.m_invMass * P2X;
      bB.m_linearVelocity.y += bB.m_invMass * P2Y;
      //bB.m_angularVelocity += bB.m_invI * b2Cross(r2, P2);
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
    } else {
      m_impulse = 0.0;
      m_limitImpulse1 = 0.0;
      m_limitImpulse2 = 0.0;
    }
  }

  @override
  void SolveVelocityConstraints(b2TimeStep step) {
    //B2_NOT_USED(step)
    b2Body bA = m_bodyA;
    b2Body bB = m_bodyB;

    b2Mat22 tMat;

    //b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
    tMat = bA.m_xf.R;
    double r1X = m_localAnchor1.x - bA.m_sweep.localCenter.x;
    double r1Y = m_localAnchor1.y - bA.m_sweep.localCenter.y;
    double tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
    r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
    r1X = tX;
    //b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
    tMat = bB.m_xf.R;
    double r2X = m_localAnchor2.x - bB.m_sweep.localCenter.x;
    double r2Y = m_localAnchor2.y - bB.m_sweep.localCenter.y;
    tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
    r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
    r2X = tX;

    // temp vars
    double v1X = 0.0;
    double v1Y = 0.0;
    double v2X = 0.0;
    double v2Y = 0.0;
    double P1X = 0.0;
    double P1Y = 0.0;
    double P2X = 0.0;
    double P2Y = 0.0;
    double Cdot = 0.0;
    double impulse = 0.0;
    double oldImpulse = 0.0;

    if (m_state == b2Joint.e_atUpperLimit) {
      //b2Vec2 v1 = bA->m_linearVelocity + b2Cross(bA->m_angularVelocity, r1);
      v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y);
      v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
      //b2Vec2 v2 = bB->m_linearVelocity + b2Cross(bB->m_angularVelocity, r2);
      v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y);
      v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);

      //Cdot = -b2Dot(m_u1, v1) - m_ratio * b2Dot(m_u2, v2);
      Cdot = -(m_u1.x * v1X + m_u1.y * v1Y) - m_ratio * (m_u2.x * v2X + m_u2.y * v2Y);
      impulse = m_pulleyMass * (-Cdot);
      oldImpulse = m_impulse;
      m_impulse = b2Math.Max(0.0, m_impulse + impulse);
      impulse = m_impulse - oldImpulse;

      //b2Vec2 P1 = -impulse * m_u1;
      P1X = -impulse * m_u1.x;
      P1Y = -impulse * m_u1.y;
      //b2Vec2 P2 = - m_ratio * impulse * m_u2;
      P2X = -m_ratio * impulse * m_u2.x;
      P2Y = -m_ratio * impulse * m_u2.y;
      //bA.m_linearVelocity += bA.m_invMass * P1;
      bA.m_linearVelocity.x += bA.m_invMass * P1X;
      bA.m_linearVelocity.y += bA.m_invMass * P1Y;
      //bA.m_angularVelocity += bA.m_invI * b2Cross(r1, P1);
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
      //bB.m_linearVelocity += bB.m_invMass * P2;
      bB.m_linearVelocity.x += bB.m_invMass * P2X;
      bB.m_linearVelocity.y += bB.m_invMass * P2Y;
      //bB.m_angularVelocity += bB.m_invI * b2Cross(r2, P2);
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
    }

    if (m_limitState1 == b2Joint.e_atUpperLimit) {
      //b2Vec2 v1 = bA->m_linearVelocity + b2Cross(bA->m_angularVelocity, r1);
      v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y);
      v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);

      //float32 Cdot = -b2Dot(m_u1, v1);
      Cdot = -(m_u1.x * v1X + m_u1.y * v1Y);
      impulse = -m_limitMass1 * Cdot;
      oldImpulse = m_limitImpulse1;
      m_limitImpulse1 = b2Math.Max(0.0, m_limitImpulse1 + impulse);
      impulse = m_limitImpulse1 - oldImpulse;

      //b2Vec2 P1 = -impulse * m_u1;
      P1X = -impulse * m_u1.x;
      P1Y = -impulse * m_u1.y;
      //bA.m_linearVelocity += bA->m_invMass * P1;
      bA.m_linearVelocity.x += bA.m_invMass * P1X;
      bA.m_linearVelocity.y += bA.m_invMass * P1Y;
      //bA.m_angularVelocity += bA->m_invI * b2Cross(r1, P1);
      bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
    }

    if (m_limitState2 == b2Joint.e_atUpperLimit) {
      //b2Vec2 v2 = bB->m_linearVelocity + b2Cross(bB->m_angularVelocity, r2);
      v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y);
      v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);

      //float32 Cdot = -b2Dot(m_u2, v2);
      Cdot = -(m_u2.x * v2X + m_u2.y * v2Y);
      impulse = -m_limitMass2 * Cdot;
      oldImpulse = m_limitImpulse2;
      m_limitImpulse2 = b2Math.Max(0.0, m_limitImpulse2 + impulse);
      impulse = m_limitImpulse2 - oldImpulse;

      //b2Vec2 P2 = -impulse * m_u2;
      P2X = -impulse * m_u2.x;
      P2Y = -impulse * m_u2.y;
      //bB->m_linearVelocity += bB->m_invMass * P2;
      bB.m_linearVelocity.x += bB.m_invMass * P2X;
      bB.m_linearVelocity.y += bB.m_invMass * P2Y;
      //bB->m_angularVelocity += bB->m_invI * b2Cross(r2, P2);
      bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
    }
  }

  @override
  bool SolvePositionConstraints(double baumgarte) {
    //B2_NOT_USED(b2Body as baumgarte)
    b2Body bA = m_bodyA;
    b2Body bB = m_bodyB;

    b2Mat22 tMat;

    //b2Vec2 s1 = m_ground->m_xf.position + m_groundAnchor1;
    double s1X = m_ground.m_xf.position.x + m_groundAnchor1.x;
    double s1Y = m_ground.m_xf.position.y + m_groundAnchor1.y;
    //b2Vec2 s2 = m_ground->m_xf.position + m_groundAnchor2;
    double s2X = m_ground.m_xf.position.x + m_groundAnchor2.x;
    double s2Y = m_ground.m_xf.position.y + m_groundAnchor2.y;

    // temp vars
    double r1X = 0.0;
    double r1Y = 0.0;
    double r2X = 0.0;
    double r2Y = 0.0;
    double p1X = 0.0;
    double p1Y = 0.0;
    double p2X = 0.0;
    double p2Y = 0.0;
    double length1 = 0.0;
    double length2 = 0.0;
    double C = 0.0;
    double impulse = 0.0;
    double oldImpulse = 0.0;
    double oldLimitPositionImpulse = 0.0;

    double tX = 0.0;

    double linearError = 0.0;

    if (m_state == b2Joint.e_atUpperLimit) {
      //b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
      tMat = bA.m_xf.R;
      r1X = m_localAnchor1.x - bA.m_sweep.localCenter.x;
      r1Y = m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      //b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
      tMat = bB.m_xf.R;
      r2X = m_localAnchor2.x - bB.m_sweep.localCenter.x;
      r2Y = m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;

      //b2Vec2 p1 = bA->m_sweep.c + r1;
      p1X = bA.m_sweep.c.x + r1X;
      p1Y = bA.m_sweep.c.y + r1Y;
      //b2Vec2 p2 = bB->m_sweep.c + r2;
      p2X = bB.m_sweep.c.x + r2X;
      p2Y = bB.m_sweep.c.y + r2Y;

      // Get the pulley axes.
      //m_u1 = p1 - s1;
      m_u1.Set(p1X - s1X, p1Y - s1Y);
      //m_u2 = p2 - s2;
      m_u2.Set(p2X - s2X, p2Y - s2Y);

      length1 = m_u1.Length();
      length2 = m_u2.Length();

      if (length1 > b2Settings.b2_linearSlop) {
        //m_u1 *= 1.0f / length1;
        m_u1.Multiply(1.0 / length1);
      } else {
        m_u1.SetZero();
      }

      if (length2 > b2Settings.b2_linearSlop) {
        //m_u2 *= 1.0f / length2;
        m_u2.Multiply(1.0 / length2);
      } else {
        m_u2.SetZero();
      }

      C = m_constant - length1 - m_ratio * length2;
      linearError = b2Math.Max(linearError, -C);
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
      impulse = -m_pulleyMass * C;

      p1X = -impulse * m_u1.x;
      p1Y = -impulse * m_u1.y;
      p2X = -m_ratio * impulse * m_u2.x;
      p2Y = -m_ratio * impulse * m_u2.y;

      bA.m_sweep.c.x += bA.m_invMass * p1X;
      bA.m_sweep.c.y += bA.m_invMass * p1Y;
      bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
      bB.m_sweep.c.x += bB.m_invMass * p2X;
      bB.m_sweep.c.y += bB.m_invMass * p2Y;
      bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);

      bA.SynchronizeTransform();
      bB.SynchronizeTransform();
    }

    if (m_limitState1 == b2Joint.e_atUpperLimit) {
      //b2Vec2 r1 = b2Mul(bA->m_xf.R, m_localAnchor1 - bA->GetLocalCenter());
      tMat = bA.m_xf.R;
      r1X = m_localAnchor1.x - bA.m_sweep.localCenter.x;
      r1Y = m_localAnchor1.y - bA.m_sweep.localCenter.y;
      tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
      r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
      r1X = tX;
      //b2Vec2 p1 = bA->m_sweep.c + r1;
      p1X = bA.m_sweep.c.x + r1X;
      p1Y = bA.m_sweep.c.y + r1Y;

      //m_u1 = p1 - s1;
      m_u1.Set(p1X - s1X, p1Y - s1Y);

      length1 = m_u1.Length();

      if (length1 > b2Settings.b2_linearSlop) {
        //m_u1 *= 1.0 / length1;
        m_u1.x *= 1.0 / length1;
        m_u1.y *= 1.0 / length1;
      } else {
        m_u1.SetZero();
      }

      C = m_maxLength1 - length1;
      linearError = b2Math.Max(linearError, -C);
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
      impulse = -m_limitMass1 * C;

      //P1 = -impulse * m_u1;
      p1X = -impulse * m_u1.x;
      p1Y = -impulse * m_u1.y;

      bA.m_sweep.c.x += bA.m_invMass * p1X;
      bA.m_sweep.c.y += bA.m_invMass * p1Y;
      //bA.m_rotation += bA.m_invI * b2Cross(r1, P1);
      bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);

      bA.SynchronizeTransform();
    }

    if (m_limitState2 == b2Joint.e_atUpperLimit) {
      //b2Vec2 r2 = b2Mul(bB->m_xf.R, m_localAnchor2 - bB->GetLocalCenter());
      tMat = bB.m_xf.R;
      r2X = m_localAnchor2.x - bB.m_sweep.localCenter.x;
      r2Y = m_localAnchor2.y - bB.m_sweep.localCenter.y;
      tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
      r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
      r2X = tX;
      //b2Vec2 p2 = bB->m_position + r2;
      p2X = bB.m_sweep.c.x + r2X;
      p2Y = bB.m_sweep.c.y + r2Y;

      //m_u2 = p2 - s2;
      m_u2.Set(p2X - s2X, p2Y - s2Y);

      length2 = m_u2.Length();

      if (length2 > b2Settings.b2_linearSlop) {
        //m_u2 *= 1.0 / length2;
        m_u2.x *= 1.0 / length2;
        m_u2.y *= 1.0 / length2;
      } else {
        m_u2.SetZero();
      }

      C = m_maxLength2 - length2;
      linearError = b2Math.Max(linearError, -C);
      C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
      impulse = -m_limitMass2 * C;

      //P2 = -impulse * m_u2;
      p2X = -impulse * m_u2.x;
      p2Y = -impulse * m_u2.y;

      //bB.m_sweep.c += bB.m_invMass * P2;
      bB.m_sweep.c.x += bB.m_invMass * p2X;
      bB.m_sweep.c.y += bB.m_invMass * p2Y;
      //bB.m_sweep.a += bB.m_invI * b2Cross(r2, P2);
      bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);

      bB.SynchronizeTransform();
    }

    return linearError < b2Settings.b2_linearSlop;
  }

  b2Body m_ground;
  b2Vec2 m_groundAnchor1 = new b2Vec2();
  b2Vec2 m_groundAnchor2 = new b2Vec2();
  b2Vec2 m_localAnchor1 = new b2Vec2();
  b2Vec2 m_localAnchor2 = new b2Vec2();

  b2Vec2 m_u1 = new b2Vec2();
  b2Vec2 m_u2 = new b2Vec2();

  double m_constant = 0.0;
  double m_ratio = 0.0;

  double m_maxLength1 = 0.0;
  double m_maxLength2 = 0.0;

  // Effective masses
  double m_pulleyMass = 0.0;
  double m_limitMass1 = 0.0;
  double m_limitMass2 = 0.0;

  // Impulses for accumulation/warm starting.
  double m_impulse = 0.0;
  double m_limitImpulse1 = 0.0;
  double m_limitImpulse2 = 0.0;

  int m_state = 0;
  int m_limitState1 = 0;
  int m_limitState2 = 0;

  // static
  static const double b2_minPulleyLength = 2.0;
}
