part of stagexl_box2d;	
	// Typees used in this example
	
	 class HelloWorld extends Sprite{
	   

     b2World m_world;
     int m_velocityIterations = 10;
     int m_positionIterations = 10;
     double m_timeStep = 1.0/30.0;
	 
     HelloWorld(){
			
			
			// Define the gravity vector
			b2Vec2 gravity = new b2Vec2(0.0, 10.0);
			
			// Allow bodies to sleep
			bool doSleep = true;
			
			// Construct a world object
			m_world = new b2World( gravity, doSleep);
			
			// set debug draw
			b2DebugDraw debugDraw = new b2DebugDraw();
			debugDraw.SetSprite(this);
			debugDraw.SetDrawScale(30.0);
			debugDraw.SetFillAlpha(0.3);
			debugDraw.SetLineThickness(1.0);
			debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
			m_world.SetDebugDraw(debugDraw);
			m_world.DrawDebugData();
			
			
			
			// Vars used to create bodies
			b2Body body;
			b2BodyDef bodyDef;
			b2PolygonShape boxShape;
			b2CircleShape circleShape;
			
			
			
			// Add ground body
			bodyDef = new b2BodyDef();
			//bodyDef.position.Set(15, 19);
			bodyDef.position.Set(10, 12);
			//bodyDef.angle = 0.1;
			boxShape = new b2PolygonShape();
			boxShape.SetAsBox(30, 3);
			b2FixtureDef fixtureDef = new b2FixtureDef();
			fixtureDef.shape = boxShape;
			fixtureDef.friction = 0.3;
			fixtureDef.density = 0; // static bodies require zero density
			// Add sprite to body userData
			bodyDef.userData = new PhysGround();
			bodyDef.userData.width = 30 * 2 * 30; 
			bodyDef.userData.height = 30 * 2 * 3; 
			addChild(bodyDef.userData);
			body = m_world.CreateBody(bodyDef);
			body.CreateFixture(fixtureDef);
			
			// Add some objects
			for (int i = 1; i < 10; i++){
				bodyDef = new b2BodyDef();
				bodyDef.position.x = new Random().nextDouble() * 15 + 5;
				bodyDef.position.y = new Random().nextDouble() * 10;
				double rX = new Random().nextDouble() + 0.5;
				double rY = new Random().nextDouble() + 0.5;
				// Box
				if (new Random().nextDouble() < 0.5){
					boxShape = new b2PolygonShape();
					boxShape.SetAsBox(rX, rY);
					fixtureDef.shape = boxShape;
					fixtureDef.density = 1.0;
					fixtureDef.friction = 0.5;
					fixtureDef.restitution = 0.2;
					bodyDef.userData = new PhysBox();
					bodyDef.userData.width = rX * 2 * 30; 
					bodyDef.userData.height = rY * 2 * 30; 
					body = m_world.CreateBody(bodyDef);
					body.CreateFixture(fixtureDef);
				} 
				// Circle
				else {
					circleShape = new b2CircleShape(rX);
					fixtureDef.shape = circleShape;
					fixtureDef.density = 1.0;
					fixtureDef.friction = 0.5;
					fixtureDef.restitution = 0.2;
					bodyDef.userData = new PhysCircle();
					bodyDef.userData.width = rX * 2 * 30; 
					bodyDef.userData.height = rX * 2 * 30; 
					body = m_world.CreateBody(bodyDef);
					body.CreateFixture(fixtureDef);
				}
				addChild(bodyDef.userData);
			}
			
// Add event for main loop
   addEventListener(Event.ENTER_FRAME, Update, useCapture: false, priority: 0);
   
			
		}
		
		  void Update(Event e){
			
			m_world.Step(m_timeStep, m_velocityIterations, m_positionIterations);
			
			// Go through body list and update sprite positions/rotations
			for (b2Body bb = m_world.GetBodyList(); bb; bb = bb.GetNext()){
				if (bb.GetUserData() is Sprite){
					Sprite sprite = bb.GetUserData() as Sprite;
					sprite.x = bb.GetPosition().x * 30;
					sprite.y = bb.GetPosition().y * 30;
					sprite.rotation = bb.GetAngle() * (180/PI);
				}
			}
			
		}
		
		
	}

