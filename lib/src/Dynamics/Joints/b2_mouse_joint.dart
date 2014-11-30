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

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

/**
* A mouse joint is used to make a point on a body track a
* specified world point. This a soft constraint with a maximum
* force. This allows the constraint to stretch and without
* applying huge forces.
* Note: this joint is not fully documented as it is intended primarily
* for the testbed. See that for more instructions.
* @see b2MouseJointDef
*/

class b2MouseJoint extends b2Joint {
  b2Vec2 m_localAnchor = new b2Vec2();
  b2Vec2 m_target = new b2Vec2();
  b2Vec2 m_impulse = new b2Vec2();

  b2Mat22 m_mass = new b2Mat22(); // effective mass for point-to-point constraint.
  b2Vec2 m_C = new b2Vec2(); // position error
  double m_maxForce = 0.0;
  double m_frequencyHz = 0.0;
  double m_dampingRatio = 0.0;
  double m_beta = 0.0; // bias factor
  double m_gamma = 0.0; // softness

  /** @inheritDoc */
  @override
  b2Vec2 GetAnchorA() {
    return m_target;
  }
  /** @inheritDoc */
  @override
  b2Vec2 GetAnchorB() {
    return m_bodyB.GetWorldPoint(m_localAnchor);
  }
  /** @inheritDoc */
  @override
  b2Vec2 GetReactionForce(double inv_dt) {
    return new b2Vec2(inv_dt * m_impulse.x, inv_dt * m_impulse.y);
  }
  /** @inheritDoc */
  @override
  double GetReactionTorque(double inv_dt) {
    return 0.0;
  }

  b2Vec2 GetTarget() {
    return m_target;
  }

  /**
	 * Use this to update the target point.
	 */
  void SetTarget(b2Vec2 target) {
    if (m_bodyB.IsAwake() == false) {
      m_bodyB.SetAwake(true);
    }
    m_target = target;
  }

  /// Get the maximum force in Newtons.
  double GetMaxForce() {
    return m_maxForce;
  }

  /// Set the maximum force in Newtons.
  void SetMaxForce(double maxForce) {
    m_maxForce = maxForce;
  }

  /// Get frequency in Hz
  double GetFrequency() {
    return m_frequencyHz;
  }

  /// Set the frequency in Hz
  void SetFrequency(double hz) {
    m_frequencyHz = hz;
  }

  /// Get damping ratio
  double GetDampingRatio() {
    return m_dampingRatio;
  }

  /// Set damping ratio
  void SetDampingRatio(double ratio) {
    m_dampingRatio = ratio;
  }

  //--------------- Internals Below -------------------

  /** @ */
  b2MouseJoint(b2MouseJointDef def) : super(def) {

    //b2Settings.b2Assert(def.target.IsValid());
    //b2Settings.b2Assert(b2Math.b2IsValid(def.maxForce) && def.maxForce > 0.0);
    //b2Settings.b2Assert(b2Math.b2IsValid(def.frequencyHz) && def.frequencyHz > 0.0);
    //b2Settings.b2Assert(b2Math.b2IsValid(def.dampingRatio) && def.dampingRatio > 0.0);

    m_target.SetV(def.target);
    //m_localAnchor = b2MulT(m_bodyB.m_xf, m_target);
    double tX = m_target.x - m_bodyB.m_xf.position.x;
    double tY = m_target.y - m_bodyB.m_xf.position.y;
    b2Mat22 tMat = m_bodyB.m_xf.R;
    m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
    m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);

    m_maxForce = def.maxForce;
    m_impulse.SetZero();

    m_frequencyHz = def.frequencyHz;
    m_dampingRatio = def.dampingRatio;

    m_beta = 0.0;
    m_gamma = 0.0;
  }

  // Presolve vars
  b2Mat22 K = new b2Mat22();
  b2Mat22 K1 = new b2Mat22();
  b2Mat22 K2 = new b2Mat22();
  @override
  void InitVelocityConstraints(b2TimeStep step) {
    b2Body b = m_bodyB;

    double mass = b.GetMass();

    // Frequency
    double omega = 2.0 * PI * m_frequencyHz;

    // Damping co-efficient
    double d = 2.0 * mass * m_dampingRatio * omega;

    // Spring stiffness
    double k = mass * omega * omega;

    // magic formulas
    // gamma has units of inverse mass
    // beta hs units of inverse time
    //b2Settings.b2Assert(d + step.dt * k > double.MIN_VALUE)
    m_gamma = step.dt * (d + step.dt * k);
    m_gamma = m_gamma != 0 ? 1 / m_gamma : 0.0;
    m_beta = step.dt * k * m_gamma;

    b2Mat22 tMat;

    // Compute the effective mass matrix.
    //b2Vec2 r = b2Mul(b->m_xf.R, m_localAnchor - b->GetLocalCenter());
    tMat = b.m_xf.R;
    double rX = m_localAnchor.x - b.m_sweep.localCenter.x;
    double rY = m_localAnchor.y - b.m_sweep.localCenter.y;
    double tX = (tMat.col1.x * rX + tMat.col2.x * rY);
    rY = (tMat.col1.y * rX + tMat.col2.y * rY);
    rX = tX;

    // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
    double invMass = b.m_invMass;
    double invI = b.m_invI;

    //b2Mat22 K1;
    K1.col1.x = invMass;
    K1.col2.x = 0.0;
    K1.col1.y = 0.0;
    K1.col2.y = invMass;

    //b2Mat22 K2;
    K2.col1.x = invI * rY * rY;
    K2.col2.x = -invI * rX * rY;
    K2.col1.y = -invI * rX * rY;
    K2.col2.y = invI * rX * rX;

    //b2Mat22 K = K1 + K2;
    K.SetM(K1);
    K.AddM(K2);
    K.col1.x += m_gamma;
    K.col2.y += m_gamma;

    //m_ptpMass = K.GetInverse();
    K.GetInverse(m_mass);

    //m_C = b.m_position + r - m_target;
    m_C.x = b.m_sweep.c.x + rX - m_target.x;
    m_C.y = b.m_sweep.c.y + rY - m_target.y;

    // Cheat with some damping
    b.m_angularVelocity *= 0.98;

    // Warm starting.
    m_impulse.x *= step.dtRatio;
    m_impulse.y *= step.dtRatio;
    //b.m_linearVelocity += invMass * m_impulse;
    b.m_linearVelocity.x += invMass * m_impulse.x;
    b.m_linearVelocity.y += invMass * m_impulse.y;
    //b.m_angularVelocity += invI * b2Cross(r, m_impulse);
    b.m_angularVelocity += invI * (rX * m_impulse.y - rY * m_impulse.x);
  }

  @override
  void SolveVelocityConstraints(b2TimeStep step) {
    b2Body b = m_bodyB;

    b2Mat22 tMat;
    double tX = 0.0;
    double tY = 0.0;

    // Compute the effective mass matrix.
    //b2Vec2 r = b2Mul(b->m_xf.R, m_localAnchor - b->GetLocalCenter());
    tMat = b.m_xf.R;
    double rX = m_localAnchor.x - b.m_sweep.localCenter.x;
    double rY = m_localAnchor.y - b.m_sweep.localCenter.y;
    tX = (tMat.col1.x * rX + tMat.col2.x * rY);
    rY = (tMat.col1.y * rX + tMat.col2.y * rY);
    rX = tX;

    // Cdot = v + cross(w, r)
    //b2Vec2 Cdot = b->m_linearVelocity + b2Cross(b->m_angularVelocity, r);
    double CdotX = b.m_linearVelocity.x + (-b.m_angularVelocity * rY);
    double CdotY = b.m_linearVelocity.y + (b.m_angularVelocity * rX);
    //b2Vec2 impulse = - b2Mul(m_mass, Cdot + m_beta * m_C + m_gamma * m_impulse);
    tMat = m_mass;
    tX = CdotX + m_beta * m_C.x + m_gamma * m_impulse.x;
    tY = CdotY + m_beta * m_C.y + m_gamma * m_impulse.y;
    double impulseX = -(tMat.col1.x * tX + tMat.col2.x * tY);
    double impulseY = -(tMat.col1.y * tX + tMat.col2.y * tY);

    double oldImpulseX = m_impulse.x;
    double oldImpulseY = m_impulse.y;
    //m_impulse += impulse;
    m_impulse.x += impulseX;
    m_impulse.y += impulseY;
    double maxImpulse = step.dt * m_maxForce;
    if (m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
      //m_impulse *= m_maxImpulse / m_impulse.Length();
      m_impulse.Multiply(maxImpulse / m_impulse.Length());
    }
    //impulse = m_impulse - oldImpulse;
    impulseX = m_impulse.x - oldImpulseX;
    impulseY = m_impulse.y - oldImpulseY;

    //b->m_linearVelocity += b->m_invMass * impulse;
    b.m_linearVelocity.x += b.m_invMass * impulseX;
    b.m_linearVelocity.y += b.m_invMass * impulseY;
    //b->m_angularVelocity += b->m_invI * b2Cross(r, P);
    b.m_angularVelocity += b.m_invI * (rX * impulseY - rY * impulseX);
  }

  @override
  bool SolvePositionConstraints(double baumgarte) {
    //B2_NOT_USED(baumgarte);
    return true;
  }


}
