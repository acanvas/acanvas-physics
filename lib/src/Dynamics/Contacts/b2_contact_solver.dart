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

/**
* @
*/
class b2ContactSolver {
  b2ContactSolver() {}

  static b2WorldManifold s_worldManifold = new b2WorldManifold();
  void Initialize(b2TimeStep step, List<b2Contact> contacts, int contactCount, dynamic allocator) {
    b2Contact contact;

    m_step.Set(step);
    m_allocator = allocator;

    int i = 0;
    b2Vec2 tVec;
    b2Mat22 tMat;

    m_constraintCount = contactCount;

    // fill vector to hold enough constraints
    while (m_constraints.length < m_constraintCount) {
      m_constraints.add(new b2ContactConstraint());
    }

    for (i = 0; i < contactCount; ++i) {
      contact = contacts[i];
      b2Fixture fixtureA = contact.m_fixtureA;
      b2Fixture fixtureB = contact.m_fixtureB;
      b2Shape shapeA = fixtureA.m_shape;
      b2Shape shapeB = fixtureB.m_shape;
      double radiusA = shapeA.m_radius;
      double radiusB = shapeB.m_radius;
      b2Body bodyA = fixtureA.m_body;
      b2Body bodyB = fixtureB.m_body;
      b2Manifold manifold = contact.GetManifold();

      double friction = b2Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
      double restitution = b2Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());

      //b2Vec2 vA = bodyA.m_linearVelocity.Copy();
      double vAX = bodyA.m_linearVelocity.x;
      double vAY = bodyA.m_linearVelocity.y;
      //b2Vec2 vB = bodyB.m_linearVelocity.Copy();
      double vBX = bodyB.m_linearVelocity.x;
      double vBY = bodyB.m_linearVelocity.y;
      double wA = bodyA.m_angularVelocity;
      double wB = bodyB.m_angularVelocity;

      b2Settings.b2Assert(manifold.m_pointCount > 0);

      s_worldManifold.Initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);

      double normalX = s_worldManifold.m_normal.x;
      double normalY = s_worldManifold.m_normal.y;

      b2ContactConstraint cc = m_constraints[i];
      cc.bodyA = bodyA; //p
      cc.bodyB = bodyB; //p
      cc.manifold = manifold; //p
      //c.normal = normal;
      cc.normal.x = normalX;
      cc.normal.y = normalY;
      cc.pointCount = manifold.m_pointCount;
      cc.friction = friction;
      cc.restitution = restitution;

      cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
      cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
      cc.localPoint.x = manifold.m_localPoint.x;
      cc.localPoint.y = manifold.m_localPoint.y;
      cc.radius = radiusA + radiusB;
      cc.type = manifold.m_type;

      for (int k = 0; k < cc.pointCount; ++k) {
        b2ManifoldPoint cp = manifold.m_points[k];
        b2ContactConstraintPoint ccp = cc.points[k];

        ccp.normalImpulse = cp.m_normalImpulse;
        ccp.tangentImpulse = cp.m_tangentImpulse;

        ccp.localPoint.SetV(cp.m_localPoint);

        double rAX = ccp.rA.x = s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x;
        double rAY = ccp.rA.y = s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y;
        double rBX = ccp.rB.x = s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x;
        double rBY = ccp.rB.y = s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y;

        double rnA = rAX * normalY - rAY * normalX; //b2Math.b2Cross(r1, normal);
        double rnB = rBX * normalY - rBY * normalX; //b2Math.b2Cross(r2, normal);

        rnA *= rnA;
        rnB *= rnB;

        double kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;
        //b2Settings.b2Assert(kNormal > double.MIN_VALUE);
        ccp.normalMass = 1.0 / kNormal;

        double kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
        kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;
        //b2Assert(kEqualized > double.MIN_VALUE);
        ccp.equalizedMass = 1.0 / kEqualized;

        //b2Vec2 tangent = b2Math.b2CrossVF(normal, 1.0);
        double tangentX = normalY;
        double tangentY = -normalX;

        //double rtA = b2Math.b2Cross(rA, tangent);
        double rtA = rAX * tangentY - rAY * tangentX;
        //double rtB = b2Math.b2Cross(rB, tangent);
        double rtB = rBX * tangentY - rBY * tangentX;

        rtA *= rtA;
        rtB *= rtB;

        double kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;
        //b2Settings.b2Assert(kTangent > double.MIN_VALUE);
        ccp.tangentMass = 1.0 / kTangent;

        // Setup a velocity bias for restitution.
        ccp.velocityBias = 0.0;
        //b2Dot(c.normal, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
        double tX = vBX + (-wB * rBY) - vAX - (-wA * rAY);
        double tY = vBY + (wB * rBX) - vAY - (wA * rAX);
        //double vRel = b2Dot(cc.normal, t);
        double vRel = cc.normal.x * tX + cc.normal.y * tY;
        if (vRel < -b2Settings.b2_velocityThreshold) {
          ccp.velocityBias += -cc.restitution * vRel;
        }
      }

      // If we have two points, then prepare the block solver.
      if (cc.pointCount == 2) {
        b2ContactConstraintPoint ccp1 = cc.points[0];
        b2ContactConstraintPoint ccp2 = cc.points[1];

        double invMassA = bodyA.m_invMass;
        double invIA = bodyA.m_invI;
        double invMassB = bodyB.m_invMass;
        double invIB = bodyB.m_invI;

        //double rn1A = b2Cross(ccp1.rA, normal);
        //double rn1B = b2Cross(ccp1.rB, normal);
        //double rn2A = b2Cross(ccp2.rA, normal);
        //double rn2B = b2Cross(ccp2.rB, normal);
        double rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
        double rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
        double rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
        double rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX;

        double k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
        double k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
        double k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

        // Ensure a reasonable condition doubleber.
        double k_maxConditiondouble = 100.0;
        if (k11 * k11 < k_maxConditiondouble * (k11 * k22 - k12 * k12)) {
          // K is safe to invert.
          cc.K.col1.Set(k11, k12);
          cc.K.col2.Set(k12, k22);
          cc.K.GetInverse(cc.normalMass);
        } else {
          // The constraints are redundant, just use one.
          // TODO_ERIN use deepest?
          cc.pointCount = 1;
        }
      }
    }

    //b2Settings.b2Assert(count == m_constraintCount);
  }
  //~b2ContactSolver();

  void InitVelocityConstraints(b2TimeStep step) {
    b2Vec2 tVec;
    b2Vec2 tVec2;
    b2Mat22 tMat;

    // Warm start.
    for (int i = 0; i < m_constraintCount; ++i) {
      b2ContactConstraint c = m_constraints[i];

      b2Body bodyA = c.bodyA;
      b2Body bodyB = c.bodyB;
      double invMassA = bodyA.m_invMass;
      double invIA = bodyA.m_invI;
      double invMassB = bodyB.m_invMass;
      double invIB = bodyB.m_invI;
      //b2Vec2 normal = new b2Vec2(c.normal.x, c.normal.y);
      double normalX = c.normal.x;
      double normalY = c.normal.y;
      //b2Vec2 tangent = b2Math.b2CrossVF(normal, 1.0);
      double tangentX = normalY;
      double tangentY = -normalX;

      double tX = 0.0;

      int j = 0;
      int tCount = 0;
      if (step.warmStarting) {
        tCount = c.pointCount;
        for (j = 0; j < tCount; ++j) {
          b2ContactConstraintPoint ccp = c.points[j];
          ccp.normalImpulse *= step.dtRatio;
          ccp.tangentImpulse *= step.dtRatio;
          //b2Vec2 P = ccp->normalImpulse * normal + ccp->tangentImpulse * tangent;
          double PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
          double PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;

          //bodyA.m_angularVelocity -= invIA * b2Math.b2CrossVV(rA, P);
          bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
          //bodyA.m_linearVelocity.Subtract( b2Math.MulFV(invMassA, P) );
          bodyA.m_linearVelocity.x -= invMassA * PX;
          bodyA.m_linearVelocity.y -= invMassA * PY;
          //bodyB.m_angularVelocity += invIB * b2Math.b2CrossVV(rB, P);
          bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
          //bodyB.m_linearVelocity.Add( b2Math.MulFV(invMassB, P) );
          bodyB.m_linearVelocity.x += invMassB * PX;
          bodyB.m_linearVelocity.y += invMassB * PY;
        }
      } else {
        tCount = c.pointCount;
        for (j = 0; j < tCount; ++j) {
          b2ContactConstraintPoint ccp2 = c.points[j];
          ccp2.normalImpulse = 0.0;
          ccp2.tangentImpulse = 0.0;
        }
      }
    }
  }

  void SolveVelocityConstraints() {
    int j = 0;
    b2ContactConstraintPoint ccp;
    double rAX = 0.0;
    double rAY = 0.0;
    double rBX = 0.0;
    double rBY = 0.0;
    double dvX = 0.0;
    double dvY = 0.0;
    double vn = 0.0;
    double vt = 0.0;
    double lambda = 0.0;
    double maxFriction = 0.0;
    double newImpulse = 0.0;
    double PX = 0.0;
    double PY = 0.0;
    double dX = 0.0;
    double dY = 0.0;
    double P1X = 0.0;
    double P1Y = 0.0;
    double P2X = 0.0;
    double P2Y = 0.0;

    b2Mat22 tMat;
    b2Vec2 tVec;

    for (int i = 0; i < m_constraintCount; ++i) {
      b2ContactConstraint c = m_constraints[i];
      b2Body bodyA = c.bodyA;
      b2Body bodyB = c.bodyB;
      double wA = bodyA.m_angularVelocity;
      double wB = bodyB.m_angularVelocity;
      b2Vec2 vA = bodyA.m_linearVelocity;
      b2Vec2 vB = bodyB.m_linearVelocity;

      double invMassA = bodyA.m_invMass;
      double invIA = bodyA.m_invI;
      double invMassB = bodyB.m_invMass;
      double invIB = bodyB.m_invI;
      //b2Vec2 normal = new b2Vec2(c.normal.x, c.normal.y);
      double normalX = c.normal.x;
      double normalY = c.normal.y;
      //b2Vec2 tangent = b2Math.b2CrossVF(normal, 1.0);
      double tangentX = normalY;
      double tangentY = -normalX;
      double friction = c.friction;

      double tX = 0.0;

      //b2Settings.b2Assert(c.pointCount == 1 || c.pointCount == 2);
      // Solve the tangent constraints
      for (j = 0; j < c.pointCount; j++) {
        ccp = c.points[j];

        // Relative velocity at contact
        //b2Vec2 dv = vB + b2Cross(wB, ccp->rB) - vA - b2Cross(wA, ccp->rA);
        dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
        dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;

        // Compute tangent force
        vt = dvX * tangentX + dvY * tangentY;
        lambda = ccp.tangentMass * -vt;

        // b2Clamp the accumulated force
        maxFriction = friction * ccp.normalImpulse;
        newImpulse = b2Math.Clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
        lambda = newImpulse - ccp.tangentImpulse;

        // Apply contact impulse
        PX = lambda * tangentX;
        PY = lambda * tangentY;

        vA.x -= invMassA * PX;
        vA.y -= invMassA * PY;
        wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);

        vB.x += invMassB * PX;
        vB.y += invMassB * PY;
        wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);

        ccp.tangentImpulse = newImpulse;
      }

      // Solve the normal constraints
      int tCount = c.pointCount;
      if (c.pointCount == 1) {
        ccp = c.points[0];

        // Relative velocity at contact
        //b2Vec2 dv = vB + b2Cross(wB, ccp->rB) - vA - b2Cross(wA, ccp->rA);
        dvX = vB.x + (-wB * ccp.rB.y) - vA.x - (-wA * ccp.rA.y);
        dvY = vB.y + (wB * ccp.rB.x) - vA.y - (wA * ccp.rA.x);

        // Compute normal impulse
        //double vn = b2Math.b2Dot(dv, normal);
        vn = dvX * normalX + dvY * normalY;
        lambda = -ccp.normalMass * (vn - ccp.velocityBias);

        // b2Clamp the accumulated impulse
        //newImpulse = b2Math.b2Max(ccp.normalImpulse + lambda, 0.0);
        newImpulse = ccp.normalImpulse + lambda;
        newImpulse = newImpulse > 0 ? newImpulse : 0.0;
        lambda = newImpulse - ccp.normalImpulse;

        // Apply contact impulse
        //b2Vec2 P = lambda * normal;
        PX = lambda * normalX;
        PY = lambda * normalY;

        //vA.Subtract( b2Math.MulFV( invMassA, P ) );
        vA.x -= invMassA * PX;
        vA.y -= invMassA * PY;
        wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX); //invIA * b2Math.b2CrossVV(ccp.rA, P);

        //vB.Add( b2Math.MulFV( invMass2, P ) );
        vB.x += invMassB * PX;
        vB.y += invMassB * PY;
        wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX); //invIB * b2Math.b2CrossVV(ccp.rB, P);

        ccp.normalImpulse = newImpulse;
      } else {
        // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
        // Build the mini LCP for this contact patch
        //
        // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
        //
        // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
        // b = vn_0 - velocityBias
        //
        // The system is solved using the "Total edoubleeration method" (s. Murty). The complementary constraint vn_i * x_i
        // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
        // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
        // solution that satisfies the problem is chosen.
        //
        // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
        // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
        //
        // Substitute:
        //
        // x = x' - a
        //
        // Plug into above equation:
        //
        // vn = A * x + b
        //    = A * (x' - a) + b
        //    = A * x' + b - A * a
        //    = A * x' + b'
        // b' = b - A * a;

        b2ContactConstraintPoint cp1 = c.points[0];
        b2ContactConstraintPoint cp2 = c.points[1];

        double aX = cp1.normalImpulse;
        double aY = cp2.normalImpulse;
        //b2Settings.b2Assert( aX >= 0.0f && aY >= 0.0f );

        // Relative velocity at contact
        //b2Vec2 dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
        double dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
        double dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
        //b2Vec2 dv2 = vB + b2Cross(wB, cpB.r2) - vA - b2Cross(wA, cp2.rA);
        double dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
        double dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;

        // Compute normal velocity
        //double vn1 = b2Dot(dv1, normal);
        double vn1 = dv1X * normalX + dv1Y * normalY;
        //double vn2 = b2Dot(dv2, normal);
        double vn2 = dv2X * normalX + dv2Y * normalY;

        double bX = vn1 - cp1.velocityBias;
        double bY = vn2 - cp2.velocityBias;

        //b -= b2Mul(c.K,a);
        tMat = c.K;
        bX -= tMat.col1.x * aX + tMat.col2.x * aY;
        bY -= tMat.col1.y * aX + tMat.col2.y * aY;

        double k_errorTol = 0.001;
        for (;;) {
          //
          // Case 1: vn = 0
          //
          // 0 = A * x' + b'
          //
          // Solve for x':
          //
          // x' = -inv(A) * b'
          //

          //b2Vec2 x = - b2Mul(c->normalMass, b);
          tMat = c.normalMass;
          double xX = -(tMat.col1.x * bX + tMat.col2.x * bY);
          double xY = -(tMat.col1.y * bX + tMat.col2.y * bY);

          if (xX >= 0.0 && xY >= 0.0) {
            // Resubstitute for the incremental impulse
            //d = x - a;
            dX = xX - aX;
            dY = xY - aY;

            //Aply incremental impulse
            //P1 = d.x * normal;
            P1X = dX * normalX;
            P1Y = dX * normalY;
            //P2 = d.y * normal;
            P2X = dY * normalX;
            P2Y = dY * normalY;

            //vA -= invMass1 * (P1 + P2)
            vA.x -= invMassA * (P1X + P2X);
            vA.y -= invMassA * (P1Y + P2Y);
            //wA -= invIA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);

            //vB += invMassB * (P1 + P2)
            vB.x += invMassB * (P1X + P2X);
            vB.y += invMassB * (P1Y + P2Y);
            //wB += invIB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);

            // Accumulate
            cp1.normalImpulse = xX;
            cp2.normalImpulse = xY;

            //#if B2_DEBUG_SOLVER == 1
            //					// Post conditions
            //					//dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
            //					dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
            //					dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
            //					//dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);
            //					dv1X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
            //					dv1Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
            //					// Compute normal velocity
            //					//vn1 = b2Dot(dv1, normal);
            //					vn1 = dv1X * normalX + dv1Y * normalY;
            //					//vn2 = b2Dot(dv2, normal);
            //					vn2 = dv2X * normalX + dv2Y * normalY;
            //
            //					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
            //					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
            //#endif
            break;
          }

          //
          // Case 2: vn1 = 0  and x2 = 0
          //
          //   0 = a11 * x1' + a12 * 0 + b1'
          // vn2 = a21 * x1' + a22 * 0 + b2'
          //

          xX = -cp1.normalMass * bX;
          xY = 0.0;
          vn1 = 0.0;
          vn2 = c.K.col1.y * xX + bY;

          if (xX >= 0.0 && vn2 >= 0.0) {
            // Resubstitute for the incremental impulse
            //d = x - a;
            dX = xX - aX;
            dY = xY - aY;

            //Aply incremental impulse
            //P1 = d.x * normal;
            P1X = dX * normalX;
            P1Y = dX * normalY;
            //P2 = d.y * normal;
            P2X = dY * normalX;
            P2Y = dY * normalY;

            //vA -= invMassA * (P1 + P2)
            vA.x -= invMassA * (P1X + P2X);
            vA.y -= invMassA * (P1Y + P2Y);
            //wA -= invIA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);

            //vB += invMassB * (P1 + P2)
            vB.x += invMassB * (P1X + P2X);
            vB.y += invMassB * (P1Y + P2Y);
            //wB += invIB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);

            // Accumulate
            cp1.normalImpulse = xX;
            cp2.normalImpulse = xY;

            //#if B2_DEBUG_SOLVER == 1
            //					// Post conditions
            //					//dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
            //					dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
            //					dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
            //					//dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);
            //					dv1X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
            //					dv1Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
            //					// Compute normal velocity
            //					//vn1 = b2Dot(dv1, normal);
            //					vn1 = dv1X * normalX + dv1Y * normalY;
            //					//vn2 = b2Dot(dv2, normal);
            //					vn2 = dv2X * normalX + dv2Y * normalY;
            //
            //					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
            //					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
            //#endif
            break;
          }

          //
          // Case 3: wB = 0 and x1 = 0
          //
          // vn1 = a11 * 0 + a12 * x2' + b1'
          //   0 = a21 * 0 + a22 * x2' + b2'
          //

          xX = 0.0;
          xY = -cp2.normalMass * bY;
          vn1 = c.K.col2.x * xY + bX;
          vn2 = 0.0;
          if (xY >= 0.0 && vn1 >= 0.0) {
            // Resubstitute for the incremental impulse
            //d = x - a;
            dX = xX - aX;
            dY = xY - aY;

            //Aply incremental impulse
            //P1 = d.x * normal;
            P1X = dX * normalX;
            P1Y = dX * normalY;
            //P2 = d.y * normal;
            P2X = dY * normalX;
            P2Y = dY * normalY;

            //vA -= invMassA * (P1 + P2)
            vA.x -= invMassA * (P1X + P2X);
            vA.y -= invMassA * (P1Y + P2Y);
            //wA -= invIA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);

            //vB += invMassB * (P1 + P2)
            vB.x += invMassB * (P1X + P2X);
            vB.y += invMassB * (P1Y + P2Y);
            //wB += invIB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);

            // Accumulate
            cp1.normalImpulse = xX;
            cp2.normalImpulse = xY;

            //#if B2_DEBUG_SOLVER == 1
            //					// Post conditions
            //					//dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
            //					dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
            //					dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
            //					//dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);
            //					dv1X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
            //					dv1Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
            //					// Compute normal velocity
            //					//vn1 = b2Dot(dv1, normal);
            //					vn1 = dv1X * normalX + dv1Y * normalY;
            //					//vn2 = b2Dot(dv2, normal);
            //					vn2 = dv2X * normalX + dv2Y * normalY;
            //
            //					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
            //					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
            //#endif
            break;
          }

          //
          // Case 4: x1 = 0 and x2 = 0
          //
          // vn1 = b1
          // vn2 = b2

          xX = 0.0;
          xY = 0.0;
          vn1 = bX;
          vn2 = bY;

          if (vn1 >= 0.0 && vn2 >= 0.0) {
            // Resubstitute for the incremental impulse
            //d = x - a;
            dX = xX - aX;
            dY = xY - aY;

            //Aply incremental impulse
            //P1 = d.x * normal;
            P1X = dX * normalX;
            P1Y = dX * normalY;
            //P2 = d.y * normal;
            P2X = dY * normalX;
            P2Y = dY * normalY;

            //vA -= invMassA * (P1 + P2)
            vA.x -= invMassA * (P1X + P2X);
            vA.y -= invMassA * (P1Y + P2Y);
            //wA -= invIA * (b2Cross(cp1.rA, P1) + b2Cross(cp2.rA, P2));
            wA -= invIA * (cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);

            //vB += invMassB * (P1 + P2)
            vB.x += invMassB * (P1X + P2X);
            vB.y += invMassB * (P1Y + P2Y);
            //wB += invIB * (b2Cross(cp1.rB, P1) + b2Cross(cp2.rB, P2));
            wB += invIB * (cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);

            // Accumulate
            cp1.normalImpulse = xX;
            cp2.normalImpulse = xY;

            //#if B2_DEBUG_SOLVER == 1
            //					// Post conditions
            //					//dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
            //					dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
            //					dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
            //					//dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);
            //					dv1X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
            //					dv1Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
            //					// Compute normal velocity
            //					//vn1 = b2Dot(dv1, normal);
            //					vn1 = dv1X * normalX + dv1Y * normalY;
            //					//vn2 = b2Dot(dv2, normal);
            //					vn2 = dv2X * normalX + dv2Y * normalY;
            //
            //					//b2Settings.b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
            //					//b2Settings.b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
            //#endif
            break;
          }

          // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
          break;
        }
      }

      // b2Vec2s in AS3 are copied by reference. The originals are
      // references to the same things here and there is no need to
      // copy them back, unlike in C++ land where b2Vec2s are
      // copied by value.
      /*bodyA->m_linearVelocity = vA;
			bodyB->m_linearVelocity = vB;*/
      bodyA.m_angularVelocity = wA;
      bodyB.m_angularVelocity = wB;
    }
  }

  void FinalizeVelocityConstraints() {
    for (int i = 0; i < m_constraintCount; ++i) {
      b2ContactConstraint c = m_constraints[i];
      b2Manifold m = c.manifold;

      for (int j = 0; j < c.pointCount; ++j) {
        b2ManifoldPoint point1 = m.m_points[j];
        b2ContactConstraintPoint point2 = c.points[j];
        point1.m_normalImpulse = point2.normalImpulse;
        point1.m_tangentImpulse = point2.tangentImpulse;
      }
    }
  }

//#if 1
// Sequential solver
//	  bool SolvePositionConstraints(double baumgarte){
//		double minSeparation = 0.0;
//
//		b2Mat22 tMat;
//		b2Vec2 tVec;
//
//		for (int i = 0; i < m_constraintCount; ++i)
//		{
//			b2ContactConstraint c = m_constraints[ i ];
//			b2Body bodyA = c.bodyA;
//			b2Body bodyB = c.bodyB;
//			b2Vec2 bA_sweep_c = bodyA.m_sweep.c;
//			double bA_sweep_a = bodyA.m_sweep.a;
//			b2Vec2 bB_sweep_c = bodyB.m_sweep.c;
//			double bB_sweep_a = bodyB.m_sweep.a;
//
//			double invMassa = bodyA.m_mass * bodyA.m_invMass;
//			double invIa = bodyA.m_mass * bodyA.m_invI;
//			double invMassb = bodyB.m_mass * bodyB.m_invMass;
//			double invIb = bodyB.m_mass * bodyB.m_invI;
//			//b2Vec2 normal = new b2Vec2(c.normal.x, c.normal.y);
//			double normalX = c.normal.x;
//			double normalY = c.normal.y;
//
//			// Solver normal constraints
//			int tCount = c.pointCount;
//			for (int j = 0; j < tCount; ++j)
//			{
//				b2ContactConstraintPoint ccp = c.points[ j ];
//
//				//r1 = b2Mul(bodyA->m_xf.R, ccp->localAnchor1 - bodyA->GetLocalCenter());
//				tMat = bodyA.m_xf.R;
//				tVec = bodyA.m_sweep.localCenter;
//				double r1X = ccp.localAnchor1.x - tVec.x;
//				double r1Y = ccp.localAnchor1.y - tVec.y;
//				tX =  (tMat.col1.x * r1X + tMat.col2.x * r1Y);
//				r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
//				r1X = tX;
//
//				//r2 = b2Mul(bodyB->m_xf.R, ccp->localAnchor2 - bodyB->GetLocalCenter());
//				tMat = bodyB.m_xf.R;
//				tVec = bodyB.m_sweep.localCenter;
//				double r2X = ccp.localAnchor2.x - tVec.x;
//				double r2Y = ccp.localAnchor2.y - tVec.y;
//				double tX =  (tMat.col1.x * r2X + tMat.col2.x * r2Y);
//				r2Y = 			 (tMat.col1.y * r2X + tMat.col2.y * r2Y);
//				r2X = tX;
//
//				//b2Vec2 p1 = bodyA->m_sweep.c + r1;
//				double p1X = b1_sweep_c.x + r1X;
//				double p1Y = b1_sweep_c.y + r1Y;
//
//				//b2Vec2 p2 = bodyB->m_sweep.c + r2;
//				double p2X = b2_sweep_c.x + r2X;
//				double p2Y = b2_sweep_c.y + r2Y;
//
//				//b2Vec2 dp = b2Math.SubtractVV(p2, p1);
//				double dpX = p2X - p1X;
//				double dpY = p2Y - p1Y;
//
//				// Approximate the current separation.
//				//double separation = b2Math.b2Dot(dp, normal) + ccp.separation;
//				double separation = (dpX*normalX + dpY*normalY) + ccp.separation;
//
//				// Track max constraint error.
//				minSeparation = b2Math.b2Min(minSeparation, separation);
//
//				// Prevent large corrections and allow slop.
//				double C =  b2Math.b2Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0.0);
//
//				// Compute normal impulse
//				double dImpulse = -ccp.equalizedMass * C;
//
//				//b2Vec2 P = b2Math.MulFV( dImpulse, normal );
//				double PX = dImpulse * normalX;
//				double PY = dImpulse * normalY;
//
//				//bodyA.m_position.Subtract( b2Math.MulFV( invMass1, impulse ) );
//				b1_sweep_c.x -= invMass1 * PX;
//				b1_sweep_c.y -= invMass1 * PY;
//				b1_sweep_a -= invI1 * (r1X * PY - r1Y * PX);//b2Math.b2CrossVV(r1, P);
//				bodyA.m_sweep.a = b1_sweep_a;
//				bodyA.SynchronizeTransform();
//
//				//bodyB.m_position.Add( b2Math.MulFV( invMass2, P ) );
//				b2_sweep_c.x += invMass2 * PX;
//				b2_sweep_c.y += invMass2 * PY;
//				b2_sweep_a += invI2 * (r2X * PY - r2Y * PX);//b2Math.b2CrossVV(r2, P);
//				bodyB.m_sweep.a = b2_sweep_a;
//				bodyB.SynchronizeTransform();
//			}
//			// Update body rotations
//			//bodyA.m_sweep.a = b1_sweep_a;
//			//bodyB.m_sweep.a = b2_sweep_a;
//		}
//
//		// We can't expect minSpeparation >= -b2_linearSlop because we don't
//		//.add the separation above -b2_linearSlop.
//		return minSeparation >= -1.5 * b2Settings.b2_linearSlop;
//	}
//#else
  // Sequential solver.
  static b2PositionSolverManifold s_psm = new b2PositionSolverManifold();
  bool SolvePositionConstraints(double baumgarte) {
    double minSeparation = 0.0;

    for (int i = 0; i < m_constraintCount; i++) {
      b2ContactConstraint c = m_constraints[i];
      b2Body bodyA = c.bodyA;
      b2Body bodyB = c.bodyB;

      double invMassA = bodyA.m_mass * bodyA.m_invMass;
      double invIA = bodyA.m_mass * bodyA.m_invI;
      double invMassB = bodyB.m_mass * bodyB.m_invMass;
      double invIB = bodyB.m_mass * bodyB.m_invI;

      s_psm.Initialize(c);
      b2Vec2 normal = s_psm.m_normal;

      // Solve normal constraints
      for (int j = 0; j < c.pointCount; j++) {
        b2ContactConstraintPoint ccp = c.points[j];

        b2Vec2 point = s_psm.m_points[j];
        double separation = s_psm.m_separations[j];

        double rAX = point.x - bodyA.m_sweep.c.x;
        double rAY = point.y - bodyA.m_sweep.c.y;
        double rBX = point.x - bodyB.m_sweep.c.x;
        double rBY = point.y - bodyB.m_sweep.c.y;

        // Track max constraint error.
        minSeparation = minSeparation < separation ? minSeparation : separation;

        // Prevent large corrections and allow slop.
        double C =
            b2Math.Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0.0);

        // Compute normal impulse
        double impulse = -ccp.equalizedMass * C;

        double PX = impulse * normal.x;
        double PY = impulse * normal.y;

        //bodyA.m_sweep.c -= invMassA * P;
        bodyA.m_sweep.c.x -= invMassA * PX;
        bodyA.m_sweep.c.y -= invMassA * PY;
        //bodyA.m_sweep.a -= invIA * b2Cross(rA, P);
        bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
        bodyA.SynchronizeTransform();

        //bodyB.m_sweep.c += invMassB * P;
        bodyB.m_sweep.c.x += invMassB * PX;
        bodyB.m_sweep.c.y += invMassB * PY;
        //bodyB.m_sweep.a += invIB * b2Cross(rB, P);
        bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
        bodyB.SynchronizeTransform();
      }
    }

    // We can't expect minSpeparation >= -b2_linearSlop because we don't
    //.add the separation above -b2_linearSlop.
    return minSeparation > -1.5 * b2Settings.b2_linearSlop;
  }

//#endif
  b2TimeStep m_step = new b2TimeStep();
  dynamic m_allocator;
  List<b2ContactConstraint> m_constraints = new List<b2ContactConstraint>();
  int m_constraintCount = 0;
}
