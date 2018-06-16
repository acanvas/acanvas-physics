//========================================
//======================================//
//						-=ANTHEM=-
//	file: frameLimiter.as
//
//	copyright: Matthew Bush 2007
//
//	notes: limits framerate
//
//======================================//
//========================================

//========================================
// frame limiter
//========================================

part of acanvas_physics;

class FRateLimiter {
  //===============
  // limit frame function
  //===============
  static void limitFrame(int maxFPS) {
    int fTime = (1000 / maxFPS).round();

    while ((newT - oldT).abs() < fTime) {
      newT = watch.elapsedMilliseconds;
    }
    oldT = watch.elapsedMilliseconds;
  }

  //===============
  // member vars
  //===============
  static Stopwatch watch = new Stopwatch()..start();
  static int oldT = watch.elapsedMilliseconds;
  static int newT = oldT;
}
