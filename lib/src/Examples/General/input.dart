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
// Input class
//========================================
 part of rockdot_physics;
	
	 class Input{
		
	    //===============
     // member data
     //===============
     // key text array
     static  List ascii;
     static  List keyState;
     static  List keyArr;
     
     static  List keyBuffer;
     static  int bufferSize = 0;
     
     // last key pressed
     static  int lastKey = 0;
     static  double timeSinceLastKey = 0.0;
     
     // mouse states
     static  bool mouseDown = false;
     static  bool mouseReleased = false;
     static  bool mouseOver = false;
     static  double mouseX = 0.0;
     static  double mouseY = 0.0;
     static  double mouseOffsetX = 0.0;
     static  double mouseOffsetY = 0.0;
     static  double mouseDragX = 0.0;
     static  double mouseDragY = 0.0;
     static  Sprite mouse = new Sprite();
     
     // stage
     static  Sprite m_stageMc;
		
		//===============
		// constructor
		//===============
	 Input(Sprite stageMc){
			
			m_stageMc = stageMc;
			
			// init ascii array
			ascii = new List(223);
			fillAscii();
			
			// init key state array
			keyState = new List(222);
			keyArr = new List();
			for (int i = 0; i < 222; i++){
				keyState[i] = 0;
				if (ascii[i] != null){
					keyArr.add(i);
				}
			}
			
			// buffer
			bufferSize = 5;
			keyBuffer = new List(bufferSize);
			for (int j = 0; j < bufferSize; j++){
				keyBuffer[j] = [0,0];
			}
			
			// add key listeners
			stageMc.stage.addEventListener(KeyboardEvent.KEY_DOWN, keyPress, useCapture: false, priority: 0);
			stageMc.stage.addEventListener(KeyboardEvent.KEY_UP, keyRelease, useCapture: false, priority: 0);		
			
			// mouse listeners
			stageMc.stage.addEventListener(MouseEvent.MOUSE_DOWN, mousePress, useCapture: false, priority: 0);
			stageMc.stage.addEventListener(MouseEvent.CLICK, mouseRelease, useCapture: false, priority: 0);
			stageMc.stage.addEventListener(MouseEvent.MOUSE_MOVE, mouseMove, useCapture: false, priority: 0);
			stageMc.stage.addEventListener(Event.MOUSE_LEAVE, mouseLeave, useCapture: false, priority: 0);
			
			mouse.graphics.moveTo(0,0);
			mouse.graphics.lineTo(0,0.1);
			mouse.graphics.strokeColor(0xFF000000, 0.1);
			
		}
		
		
		
		//===============
		// update
		//===============
		static   void update(){
			
			// update used keys
			for (int i = 0; i < keyArr.length; i++){
				if (keyState[keyArr[i]] != 0){
					keyState[keyArr[i]]++;
				}
			}
			
			// update buffer
			for (int j = 0; j < bufferSize; j++){
				keyBuffer[j][1]++;
			}
			
			// end mouse release
			mouseReleased = false;
			mouseOver = false;
			
		}
		
		
		
		//===============
		// mousePress listener
		//===============
		  void mousePress(MouseEvent e){
			mouseDown = true;
			mouseDragX = 0.0;
			mouseDragY = 0.0;
		}
		
		
		
		//===============
		// mousePress listener
		//===============
		  void mouseRelease(MouseEvent e){
			mouseDown = false;
			mouseReleased = true;
		}
		
		
		
		//===============
		// mousePress listener
		//===============
		  void mouseLeave(Event e){
			mouseReleased = mouseDown;
			mouseDown = false;
		}
		
		
		
		//===============
		// mouseMove listener
		//===============
		  void mouseMove(MouseEvent e){
			
			// Fix mouse release not being registered from mouse going off stage
			if (mouseDown != e.buttonDown){
				mouseDown = e.buttonDown;
				mouseReleased = !e.buttonDown;
				mouseDragX = 0.0;
				mouseDragY = 0.0;
			}
			
			mouseX = e.stageX - m_stageMc.x;
			mouseY = e.stageY - m_stageMc.y;
			// Store offset
			mouseOffsetX = mouseX - mouse.x;
			mouseOffsetY = mouseY - mouse.y;
			// Update drag
			if( mouseDown){
				mouseDragX += mouseOffsetX;
				mouseDragY += mouseOffsetY;
			}
			mouse.x = mouseX;
			mouse.y = mouseY;
		}
		
		
		
		//===============
		// getKeyHold
		//===============
		static   int getKeyHold(int k){
			return /*Math.*/max(0, keyState[k]);
		}
		
		
		//===============
		// isKeyDown
		//===============
		static   bool isKeyDown(int k){
			return (keyState[k] > 0);
		}
		
		
		
		//===============
		//  isKeyPressed
		//===============
		static   bool isKeyPressed(int k){
			timeSinceLastKey = 0.0;
			return (keyState[k] == 1);
		}
		
		
		
		//===============
		//  isKeyReleased
		//===============
		static   bool isKeyReleased(int k){
			return (keyState[k] == -1);
		}
		
		
		
		//===============
		// isKeyInBuffer
		//===============
		static   bool isKeyInBuffer(int k,int i,int t){
			return (keyBuffer[i][0] == k && keyBuffer[i][1] <= t);
		}
		
		
		
		//===============
		// keyPress function
		//===============
		  void keyPress(KeyboardEvent e){
			
			//sprint ( e.keyCode + " : " + ascii[e.keyCode] );
			
			// set keyState
			keyState[e.keyCode] = /*Math.*/max(keyState[e.keyCode], 1);
			
			// last key (for key config)
			lastKey = e.keyCode;
			
		}
		
		//===============
		// keyRelease function
		//===============
		  void keyRelease(KeyboardEvent e){
			keyState[e.keyCode] = -1;
			
			// add to key buffer
			for (int i = bufferSize-1; i > 0 ; i--){
				keyBuffer[i] = keyBuffer[i - 1];
			}
			keyBuffer[0] = [e.keyCode, 0];
		}
		
		
		
		//===============
		// get key string
		//===============
		static   String getKeyString(int k){
			return ascii[k];
		}
		
		
		//===============
		// set up ascii text
		//===============
		  void fillAscii(){
			ascii[65] = "A";
			ascii[66] = "B";
			ascii[67] = "C";
			ascii[68] = "D";
			ascii[69] = "E";
			ascii[70] = "F";
			ascii[71] = "G";
			ascii[72] = "H";
			ascii[73] = "I";
			ascii[74] = "J";
			ascii[75] = "K";
			ascii[76] = "L";
			ascii[77] = "M";
			ascii[78] = "N";
			ascii[79] = "O";
			ascii[80] = "P";
			ascii[81] = "Q";
			ascii[82] = "R";
			ascii[83] = "S";
			ascii[84] = "T";
			ascii[85] = "U";
			ascii[86] = "V";
			ascii[87] = "W";
			ascii[88] = "X";
			ascii[89] = "Y";
			ascii[90] = "Z";
			ascii[48] = "0";
			ascii[49] = "1";
			ascii[50] = "2";
			ascii[51] = "3";
			ascii[52] = "4";
			ascii[53] = "5";
			ascii[54] = "6";
			ascii[55] = "7";
			ascii[56] = "8";
			ascii[57] = "9";
			ascii[32] = "Spacebar";
			ascii[17] = "Ctrl";
			ascii[16] = "Shift";
			ascii[192] = "~";
			ascii[38] = "up";
			ascii[40] = "down";
			ascii[37] = "left";
			ascii[39] = "right";
			ascii[96] = "doublepad 0";
			ascii[97] = "doublepad 1";
			ascii[98] = "doublepad 2";
			ascii[99] = "doublepad 3";
			ascii[100] = "doublepad 4";
			ascii[101] = "doublepad 5";
			ascii[102] = "doublepad 6";
			ascii[103] = "doublepad 7";
			ascii[104] = "doublepad 8";
			ascii[105] = "doublepad 9";
			ascii[111] = "doublepad /";
			ascii[106] = "doublepad *";
			ascii[109] = "doublepad -";
			ascii[107] = "doublepad +";
			ascii[110] = "doublepad .";
			ascii[45] = "Insert";
			ascii[46] = "Delete";
			ascii[33] = "Page Up";
			ascii[34] = "Page Down";
			ascii[35] = "End";
			ascii[36] = "Home";
			ascii[112] = "F1";
			ascii[113] = "F2";
			ascii[114] = "F3";
			ascii[115] = "F4";
			ascii[116] = "F5";
			ascii[117] = "F6";
			ascii[118] = "F7";
			ascii[119] = "F8";
			ascii[188] = ",";
			ascii[190] = ".";
			ascii[186] = ";";
			ascii[222] = "'";
			ascii[219] = "[";
			ascii[221] = "]";
			ascii[189] = "-";
			ascii[187] = "+";
			ascii[220] = "\\";
			ascii[191] = "/";
			ascii[9] = "TAB";
			ascii[8] = "Backspace";
			//ascii[27] = "ESC";
		}
		

	}
	
	
