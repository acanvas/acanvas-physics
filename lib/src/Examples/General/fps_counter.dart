//========================================
//======================================//
//						-=ANTHEM=-
//	file: .as
//
//	copyright: Matthew Bush 2007
//
//	notes:
//
//======================================//
//========================================

//========================================
// FPS COUNTER CLASS
//========================================
part of rockdot_physics;

class FpsCounter extends Sprite {
  //===============
  //  variables
  //===============
  TextField textBox;
  TextField textBox2;
  TextField textBox3;
  int mfpsCount = 0;
  int mfpsCount2 = 0;
  int avgCount = 30;
  int avgCount2 = 30;
  int oldT = 0;
  Stopwatch watch;

  //===============
  // constructor
  //===============
  FpsCounter(this.watch) {
    // create text field
    textBox = new TextField();
    textBox.text = "...";
    textBox.textColor = 0xaa1144;
//			textBox.selectable = false;

    textBox2 = new TextField();
    textBox2.text = "...";
    textBox2.width = 150;
    textBox2.textColor = 0xaa1144;
//			textBox2.selectable = false;
    textBox2.y = 15;

    textBox3 = new TextField();
    textBox3.text = "...";
    textBox3.textColor = 0xaa1144;
//			textBox3.selectable = false;
    textBox3.y = 30;

    // set initial lastTime
    oldT = watch.elapsedMilliseconds;

    addChild(textBox);
    addChild(textBox2);
    addChild(textBox3);
  }

  //===============
  // update function
  //===============
  void update() {
    int newT = watch.elapsedMilliseconds;
    int f1 = newT - oldT;
    mfpsCount += f1;
    if (avgCount < 1) {
      textBox.text = ((1000 / (mfpsCount / 30)).round().toString() + " fps average");
      avgCount = 30;
      mfpsCount = 0;
    }
    avgCount--;
    oldT = watch.elapsedMilliseconds;

    //textBox3.text = (System.totalMemory/(1024*1024)).round().toString() + " MB used"
  }

  void updatePhys(int oldT2) {
    int newT = watch.elapsedMilliseconds;
    int f1 = (newT - oldT2);
    mfpsCount2 += f1 + 1;
    if (avgCount2 < 1) {
      textBox2.text = ("Physics step: " +
          (mfpsCount2 / 30).toString() +
          " ms (" +
          (1000 / (mfpsCount2 / 30)).round().toString() +
          " fps)");
      avgCount2 = 30;
      mfpsCount2 = 0;
    }
    avgCount2--;
  }

  //===============
  // updateend function
  //===============
  void updateEnd() {
    // wrong
    /*int newT = /*getTimer()*/ (stage.juggler.elapsedTime*1000);
			int f1 = newT-oldT;
			mfpsCount2 += f1;
			if (avgCount2 < 1){
				textBox2.text = String((1000/(mfpsCount2/30))+" fps uncapped").round();
				avgCount2 = 30;
				mfpsCount2 = 0;
			}
			avgCount2--;*/
  }
}
