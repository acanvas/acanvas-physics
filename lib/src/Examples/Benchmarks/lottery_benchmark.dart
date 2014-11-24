/*
* Copyright (2009 as c) Adam Newgas http://www.boristhebrave.com
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

	 class LotteryBenchmark implements IBenchmark
	{
		 double angularVelocity = 1.0;
		  String Name()
		{
			return "Lottery";
		}
		
		  void Init(b2World world)
		{
			world.SetGravity(new b2Vec2(0.0, 10.0));
			
			b2BodyDef groundDef = new b2BodyDef();
			groundDef.type = b2Body.b2_kinematicBody;
			groundDef.angularVelocity = angularVelocity;
			groundDef.position.Set(5.0, 5.0);
			b2PolygonShape ground = new b2PolygonShape();
			b2Body body;
			body = world.CreateBody(groundDef);
			ground.SetAsOrientedBox(.5, 5.0, new b2Vec2(-4.5, 0.0));
			body.CreateFixture2(ground);
			ground.SetAsOrientedBox(5.0, .5, new b2Vec2(0.0, -4.5));
			body.CreateFixture2(ground);
			ground.SetAsOrientedBox(5.0, .5, new b2Vec2(0.0, 4.5));
			body.CreateFixture2(ground);
			ground.SetAsOrientedBox(.5, 5.0, new b2Vec2(4.5, 0.0));
			body.CreateFixture2(ground);
			
			b2BodyDef bodyDef = new b2BodyDef();
			bodyDef.type = b2Body.b2_dynamicBody;
			b2CircleShape circle = new b2CircleShape(.5);
			for (int i = 0; i < 5; i++)
			{
				for (int j = 0; j < 5; j++)
				{
					bodyDef.position.Set(i + 1.0, j + 1.0);
					bodyDef.userData = i * 5 + j + 1;
					body = world.CreateBody(bodyDef);
					body.CreateFixture2(circle, 1.0);
				}
			}
			
			world.SetContactFilter(new LotteryContactFilter());
			world.SetContactListener(new LotteryContactListener());
		}
		
		  void Update()
		{
			
		}
	}


class LotteryContactFilter extends b2ContactFilter
{
	@override 
		  bool ShouldCollide(b2Fixture fixtureA,b2Fixture fixtureB) 
	{
		dynamic udA = fixtureA.GetBody().GetUserData();
		dynamic udB = fixtureB.GetBody().GetUserData();
		if (udA && udB)
		{
			return ((udA - udB) % 3 != 0);
		}
			
		return true;
	}
}

class LotteryContactListener extends b2ContactListener
{
	@override 
		  void PreSolve(b2Contact contact,b2Manifold oldManifold) 
	{
		
		dynamic udA = contact.GetFixtureA().GetBody().GetUserData();
		dynamic udB = contact.GetFixtureB().GetBody().GetUserData();
		if(udA && udB && ((udA - udB) % 2 == 0))
			contact.SetEnabled(false);
	}
}