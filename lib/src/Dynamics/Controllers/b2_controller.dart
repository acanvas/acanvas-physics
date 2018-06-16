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
 * Base class for controllers. Controllers are a convience for encapsulating common
 * per-step functionality.
 */
class b2Controller {
  void Step(b2TimeStep step) {}

  void Draw(b2DebugDraw debugDraw) {}

  void AddBody(b2Body body) {
    b2ControllerEdge edge = new b2ControllerEdge();
    edge.controller = this;
    edge.body = body;
    //
    edge.nextBody = m_bodyList;
    edge.prevBody = null;
    m_bodyList = edge;
    if (edge.nextBody != null) edge.nextBody.prevBody = edge;
    m_bodyCount++;
    //
    edge.nextController = body.m_controllerList;
    edge.prevController = null;
    body.m_controllerList = edge;
    if (edge.nextController != null) edge.nextController.prevController = edge;
    body.m_controllerCount++;
  }

  void RemoveBody(b2Body body) {
    b2ControllerEdge edge = body.m_controllerList;
    while (edge != null && edge.controller != this) edge = edge.nextController;

    //Attempted to remove a body that was not attached?
    //b2Settings.b2Assert(bEdge != null);

    if (edge.prevBody != null) edge.prevBody.nextBody = edge.nextBody;
    if (edge.nextBody != null) edge.nextBody.prevBody = edge.prevBody;
    if (edge.nextController != null) edge.nextController.prevController = edge.prevController;
    if (edge.prevController != null) edge.prevController.nextController = edge.nextController;
    if (m_bodyList == edge) m_bodyList = edge.nextBody;
    if (body.m_controllerList == edge) body.m_controllerList = edge.nextController;
    body.m_controllerCount--;
    m_bodyCount--;
    //b2Settings.b2Assert(body.m_controllerCount >= 0);
    //b2Settings.b2Assert(m_bodyCount >= 0);
  }

  void Clear() {
    while (m_bodyList != null) RemoveBody(m_bodyList.body);
  }

  b2Controller GetNext() {
    return m_next;
  }

  b2World GetWorld() {
    return m_world;
  }

  b2ControllerEdge GetBodyList() {
    return m_bodyList;
  }

  b2Controller m_next;
  b2Controller m_prev;

  b2ControllerEdge m_bodyList;
  int m_bodyCount = 0;

  b2World m_world;
}
