/**
* @author John Nesky <http://www.johnnesky.com>
* Bezier curves added by Quest Yarbrough <http://www.ezqueststudios.com>
* partially based on some code by Helen Triolo <http://flash-creations.com/notes/sample_svgtoflash.php>
* 
* This code is available under the LGPL license. <http://www.gnu.org/licenses/lgpl-3.0.txt>
* 
* Some desirable features that this does NOT have yet include: dynamic -quadratic bezier curves (as opposed to the more common cubic bezier curves)
* -elliptical arcs
* -transformation matrices
* -non-path primitives, like circles and rectangles
* -the ability to use filled regions to determine which side of the edges should be solid, 
*    rather than relying on the more finicky winding direction.
*/

 part of stagexl_box2d;
   class b2SVG
  {
    
     static  void parseSVG(XML svg,b2World world,double RATIO,double useDefaultCurveResolution) {
      Namespace ns = svg.namespace("");
      bool useCurveThickness = false;
      double resolution;
      
      if (useDefaultCurveResolution > 0) {
        resolution = useDefaultCurveResolution; 
      } else {
        useCurveThickness = true;
      }
      
      b2EdgeChainDef chainDef = new b2EdgeChainDef();
      chainDef.friction = 0.5;
      chainDef.restitution = 0.0;
      for (XML path in svg..ns::path)
      {
        if (resolution as useCurveThickness) = (returnStrokeWidth(path.@style) * 10).round();
        
        // the entire path:
        String d = path.@d;
        
        // Inkscape, Illustrator, and other programs use slightly different
        // formats for the path, because the SVG specs are very flexible. This
        // puts some strain on the people who write path parsers (me!), but 
        // Helen Triolo's example illustrates an easy way to convert the path
        // to a consistent format and then break it up into an array:
        
        // replace whitespace with commas:
        String letter;
        for (letter in [" ","\f","\n","\r","\t"])
          d = d.split(letter).join(",");
        
        // surround letters with commas:
        for (letter in ["M","m","Z","z","L","l","H","h","V","v","C","c","S","s","Q","q","T","t","A","a"])
          d = d.split(letter).join("," + letter + ",");
        
        // insert commas before negative symbols:
        d = d.split("-").join(",-");
        
        // now get all tokens separated by commas:
        List args = d.split(",");
        
        // remove empty strings:
        args = args.filter(filterEmptyString);
        
        
        b2Vec2 currentPosition = new b2Vec2(0,0);
        b2Vec2 control1 = null;
        b2Vec2 control2 = null;
        b2Vec2 control3 = null;
        b2Vec2 prevControl = null;
        String prevCommand = null;
        bool relative = false;
        List curve;
        
        chainDef.vertices.length = 0;
        int i = 0;
        while (true)
        {
          if (i == args.length) {
            // cleanup: If a path is incomplete, finish it without closing the loop
            //          and add it to the Box2D world.
            if (chainDef.vertices.length >= 2) {
              chainDef.vertexCount = chainDef.vertices.length;
              chainDef.isALoop = false;
              world.GetGroundBody().CreateShape(chainDef);
            }
            break;
          }
          
          String command = args[i];
          i++;
          
          switch (command)
          {
            case "Z":
            case "z":
              // closepath: If a path is incomplete, finish it, close the loop,
              //            and add it to the Box2D world. 
              if (chainDef.vertices.length >= 3) {
                chainDef.vertices.pop(); // the last vertex of the loop is redundant.
                chainDef.vertexCount = chainDef.vertices.length;
                chainDef.isALoop = true;
                world.GetGroundBody().CreateShape(chainDef);
              }
              chainDef.vertices.length = 0;
              chainDef.vertices.add(currentPosition);
              break;
            case "M":
            case "m":
              // moveto: If a path is incomplete, finish it without closing the loop
              //          and add it to the Box2D world.
              //          Start a new path.
              if (chainDef.vertices.length >= 2) {
                chainDef.vertexCount = chainDef.vertices.length;
                chainDef.isALoop = false;
                world.GetGroundBody().CreateShape(chainDef);
              }
              relative = (command == "m");
              if( relative ) {
                currentPosition = new b2Vec2(currentPosition.x + args[i] / RATIO,
                                             currentPosition.y + args[i+1] / RATIO);
              } else {
                currentPosition = new b2Vec2(args[i] / RATIO, args[i+1] / RATIO);
              }
              i += 2;
              chainDef.vertices.length = 0;
              chainDef.vertices.add(currentPosition);
              
              // According to the SVG spec, a moveto command can be implicitly followed
              // by lineto coordinates. So there is no "break" here.
            case "L":
            case "l":
              // lineto: a series of straight lines. Keep parsing until you hit a non-doubleber. 
              if (command == "l") relative = true;
              else if (command == "L") relative = false;
              while (!isNaN(parseFloat(args[i])))
              {
                if( relative ) {
                  currentPosition = new b2Vec2(currentPosition.x + args[i] / RATIO,
                                               currentPosition.y + args[i+1] / RATIO);
                } else {
                  currentPosition = new b2Vec2(args[i] / RATIO, args[i+1] / RATIO);
                }
                i += 2;
                chainDef.vertices.add(currentPosition);
              }
              break;
            case "H":
            case "h":
              // horizontal lineto: a series of horizontal lines. 
              //                    keep parsing until you hit a non-doubleber. 
              //                    Box2D works much better if adjacent parallel lines
              //                    are merged into one, so I'll go ahead and merge them.
              relative = (command == "h");
              do
              {
                if( relative ) {
                  currentPosition = new b2Vec2(currentPosition.x + args[i] / RATIO,
                                               currentPosition.y);
                } else {
                  currentPosition = new b2Vec2(args[i] / RATIO, currentPosition.y);
                }
                i++;
              } while (!isNaN(parseFloat(args[i])));
              chainDef.vertices.add(currentPosition);
              break;
            case "V":
            case "v":
              // vertical lineto: a series of vertical lines. 
              //                  keep parsing until you hit a non-doubleber. 
              //                  Box2D works much better if adjacent parallel lines
              //                  are merged into one, so I'll go ahead and merge them.
              relative = (command == "v");
              do
              {
                if( relative ) {
                  currentPosition = new b2Vec2(currentPosition.x,
                                               currentPosition.y + args[i] / RATIO);
                } else {
                  currentPosition = new b2Vec2(currentPosition.x, args[i] / RATIO);
                }
                i++;
              } while (!isNaN(parseFloat(args[i])));
              chainDef.vertices.add(currentPosition);
              break;
            case "C":
            case "c":
              // curveto
              relative = (command == "c");
              do
              {
                if( relative ) {
                  control1 = new b2Vec2(currentPosition.x + args[i] / RATIO,
                                        currentPosition.y + args[i+1] / RATIO);
                  control2 = new b2Vec2(currentPosition.x + args[i+2] / RATIO,
                                        currentPosition.y + args[i+3] / RATIO);
                  control3 = new b2Vec2(currentPosition.x + args[i+4] / RATIO,
                                        currentPosition.y + args[i+5] / RATIO);
                } else {
                  control1 = new b2Vec2(args[i] / RATIO, args[i+1] / RATIO);
                  control2 = new b2Vec2(args[i+2] / RATIO, args[i+3] / RATIO);
                  control3 = new b2Vec2(args[i+4] / RATIO, args[i+5] / RATIO);
                }
                i += 6;
                
                curve = b2Bezier.parseCurve([currentPosition, control1, control2, control3], resolution);
                curve.shift(); // the first point is redundant, "currentPosition" is already added.
                chainDef.vertices = chainDef.vertices.concat(curve);
                
                currentPosition = control3;
              } while (!isNaN(parseFloat(args[i])));
              
              prevControl = control2;
              break;
            case "S":
            case "s":
              // shorthand curveto
              relative = (command == "s");
              do
              {
                if (prevCommand == "C" || prevCommand == "c" || prevCommand == "S" || prevCommand == "s")
                {
                  control1 = new b2Vec2(currentPosition.x*2 - prevControl.x,
                                        currentPosition.y*2 - prevControl.y);
                } else {
                  control1 = new b2Vec2(currentPosition.x,
                                        currentPosition.y);
                }
                if( relative ) {
                  control2 = new b2Vec2(currentPosition.x + args[i] / RATIO,
                                        currentPosition.y + args[i+1] / RATIO);
                  control3 = new b2Vec2(currentPosition.x + args[i+2] / RATIO,
                                        currentPosition.y + args[i+3] / RATIO);
                } else {
                  control2 = new b2Vec2(args[i] / RATIO, args[i+1] / RATIO);
                  control3 = new b2Vec2(args[i+2] / RATIO, args[i+3] / RATIO);
                }
                i += 4;
                
                curve = b2Bezier.parseCurve([currentPosition, control1, control2, control3], resolution);
                curve.shift(); // the first point is redundant, "currentPosition" is already added.
                chainDef.vertices = chainDef.vertices.concat(curve);
                
                currentPosition = control3;
                prevControl = control2;
                prevCommand = command;
                
              } while (!isNaN(parseFloat(args[i])));
              
              break;
            case "Q":
            case "q":
            case "T":
            case "t":
            case "A":
            case "a":
              throw new Error("TODO: Unimplemented path command: " + command);
              break;
          }
          prevCommand = command;
        }
      }
    }
    
     static  bool filterEmptyString(dynamic item,int index,List array) {
      return item != "";
    }
    
     static  double returnStrokeWidth(String pathS)
    {
      List bar;
      bar = (pathS.replace("px", "")).split(";"); //get rid of px in line-width and split
      return double(bar[3].slice(13, bar[3].length));
    }
    
  }

