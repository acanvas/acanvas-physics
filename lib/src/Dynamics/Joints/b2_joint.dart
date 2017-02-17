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
* The base joint class. Joints are used to constraint two bodies together in
* various fashions. Some joints also feature limits and motors.
* @see b2JointDef
*/
class b2Joint {
  /**
	* Get the type of the concrete joint.
	*/
  int GetType() {
    return m_type;
  }

  /**
	* Get the anchor point on bodyA in world coordinates.
	*/
  b2Vec2 GetAnchorA() {
    return null;
  }

  /**
	* Get the anchor point on bodyB in world coordinates.
	*/
  b2Vec2 GetAnchorB() {
    return null;
  }

  /**
	* Get the reaction force on body2 at the joint anchor in Newtons.
	*/
  b2Vec2 GetReactionForce(double inv_dt) {
    return null;
  }

  /**
	* Get the reaction torque on body2 in N*m.
	*/
  double GetReactionTorque(double inv_dt) {
    return 0.0;
  }

  /**
	* Get the first body attached to this joint.
	*/
  b2Body GetBodyA() {
    return m_bodyA;
  }

  /**
	* Get the second body attached to this joint.
	*/
  b2Body GetBodyB() {
    return m_bodyB;
  }

  /**
	* Get the next joint the world joint list.
	*/
  b2Joint GetNext() {
    return m_next;
  }

  /**
	* Get the user data pointer.
	*/
  dynamic GetUserData() {
    return m_userData;
  }

  /**
	* Set the user data pointer.
	*/
  void SetUserData(dynamic data) {
    m_userData = data;
  }

  /**
	 * Short-cut function to determine if either body is inactive.
	 * @return
	 */
  bool IsActive() {
    return m_bodyA.IsActive() && m_bodyB.IsActive();
  }

  //--------------- Internals Below -------------------

  static b2Joint Create(b2JointDef def, dynamic allocator) {
    b2Joint joint = null;

    switch (def.type) {
      case e_distanceJoint:
        {
          //void* mem = allocator->Allocate(sizeof(b2DistanceJoint));
          joint = new b2DistanceJoint(def as b2DistanceJointDef);
        }
        break;

      case e_mouseJoint:
        {
          //void* mem = allocator->Allocate(sizeof(b2MouseJoint));
          joint = new b2MouseJoint(def as b2MouseJointDef);
        }
        break;

      case e_prismaticJoint:
        {
          //void* mem = allocator->Allocate(sizeof(b2PrismaticJoint));
          joint = new b2PrismaticJoint(def as b2PrismaticJointDef);
        }
        break;

      case e_revoluteJoint:
        {
          //void* mem = allocator->Allocate(sizeof(b2RevoluteJoint));
          joint = new b2RevoluteJoint(def as b2RevoluteJointDef);
        }
        break;

      case e_pulleyJoint:
        {
          //void* mem = allocator->Allocate(sizeof(b2PulleyJoint));
          joint = new b2PulleyJoint(def as b2PulleyJointDef);
        }
        break;

      case e_gearJoint:
        {
          //void* mem = allocator->Allocate(sizeof(b2GearJoint));
          joint = new b2GearJoint(def as b2GearJointDef);
        }
        break;

      case e_lineJoint:
        {
          //void* mem = allocator->Allocate(sizeof(b2LineJoint));
          joint = new b2LineJoint(def as b2LineJointDef);
        }
        break;

      case e_weldJoint:
        {
          //void* mem = allocator->Allocate(sizeof(b2WeldJoint));
          joint = new b2WeldJoint(def as b2WeldJointDef);
        }
        break;

      case e_frictionJoint:
        {
          //void* mem = allocator->Allocate(sizeof(b2FrictionJoint));
          joint = new b2FrictionJoint(def as b2FrictionJointDef);
        }
        break;

      default:
        //b2Settings.b2Assert(false);
        break;
    }

    return joint;
  }

  static void Destroy(b2Joint joint, dynamic allocator) {
    /*joint->~b2Joint();
		switch (joint.m_type)
		{
		case e_distanceJoint:
			allocator->Free(joint, sizeof(b2DistanceJoint));
			break;
		
		case e_mouseJoint:
			allocator->Free(joint, sizeof(b2MouseJoint));
			break;
		
		case e_prismaticJoint:
			allocator->Free(joint, sizeof(b2PrismaticJoint));
			break;
		
		case e_revoluteJoint:
			allocator->Free(joint, sizeof(b2RevoluteJoint));
			break;
		
		case e_pulleyJoint:
			allocator->Free(joint, sizeof(b2PulleyJoint));
			break;
		
		case e_gearJoint:
			allocator->Free(joint, sizeof(b2GearJoint));
			break;
		
		case e_lineJoint:
			allocator->Free(joint, sizeof(b2LineJoint));
			break;
			
		case e_weldJoint:
			allocator->Free(joint, sizeof(b2WeldJoint));
			break;
			
		case e_frictionJoint:
			allocator->Free(joint, sizeof(b2FrictionJoint));
			break;
		
		default:
			b2Assert(false);
			break;
		}*/
  }

  /** @ */
  b2Joint(b2JointDef def) {
    b2Settings.b2Assert(def.bodyA != def.bodyB);
    m_type = def.type;
    m_prev = null;
    m_next = null;
    m_bodyA = def.bodyA;
    m_bodyB = def.bodyB;
    m_collideConnected = def.collideConnected;
    m_islandFlag = false;
    m_userData = def.userData;
  }
  //virtual ~b2Joint() {}

  void InitVelocityConstraints(b2TimeStep step) {}
  void SolveVelocityConstraints(b2TimeStep step) {}
  void FinalizeVelocityConstraints() {}

  // This returns true if the position errors are within tolerance.
  bool SolvePositionConstraints(double baumgarte) {
    return false;
  }

  int m_type = 0;
  b2Joint m_prev;
  b2Joint m_next;
  b2JointEdge m_edgeA = new b2JointEdge();
  b2JointEdge m_edgeB = new b2JointEdge();
  b2Body m_bodyA;
  b2Body m_bodyB;

  bool m_islandFlag;
  bool m_collideConnected;

  dynamic m_userData;

  // Cache here per time step to reduce cache misses.
  b2Vec2 m_localCenterA = new b2Vec2();
  b2Vec2 m_localCenterB = new b2Vec2();
  double m_invMassA = 0.0;
  double m_invMassB = 0.0;
  double m_invIA = 0.0;
  double m_invIB = 0.0;

  // EdoubleS

  // edouble b2JointType
  static const int e_unknownJoint = 0;
  static const int e_revoluteJoint = 1;
  static const int e_prismaticJoint = 2;
  static const int e_distanceJoint = 3;
  static const int e_pulleyJoint = 4;
  static const int e_mouseJoint = 5;
  static const int e_gearJoint = 6;
  static const int e_lineJoint = 7;
  static const int e_weldJoint = 8;
  static const int e_frictionJoint = 9;

  // edouble b2LimitState
  static const int e_inactiveLimit = 0;
  static const int e_atLowerLimit = 1;
  static const int e_atUpperLimit = 2;
  static const int e_equalLimits = 3;
}
