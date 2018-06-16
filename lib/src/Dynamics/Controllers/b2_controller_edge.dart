part of acanvas_physics;

class b2ControllerEdge {
  /** provides quick access to other end of this edge */
  b2Controller controller;
  /** the body */
  b2Body body;
  /** the previous controller edge in the controllers's body list */
  b2ControllerEdge prevBody;
  /** the next controller edge in the controllers's body list */
  b2ControllerEdge nextBody;
  /** the previous controller edge in the body's controller list */
  b2ControllerEdge prevController;
  /** the next controller edge in the body's controller list */
  b2ControllerEdge nextController;
}
