import 'dart:html' as html;
import 'package:stagexl/stagexl.dart';
import 'package:acanvas_physics/acanvas_physics.dart';

void main() {
  html.CanvasElement stageEl = html.querySelector('#stage');

  /* Startup StageXL */
  var options = new StageOptions();
  options.renderEngine = RenderEngine.WebGL;
  options.backgroundColor = 0xFF292C2C;
  Stage stage = new Stage(stageEl, options: options, width: 640, height: 320);

  stage.scaleMode = StageScaleMode.NO_SCALE;
  stage.align = StageAlign.TOP_LEFT;
  stage.focus = stage;

  RenderLoop renderLoop = new RenderLoop();
  renderLoop.addStage(stage);

  //TODO Examples are a direct conversion from ActionScript and don't work. Please use acanvas generator to see examples.
  //stage.addChild( new Main() );
}
