import 'dart:html' as html;
import 'package:stagexl/stagexl.dart';
import 'package:rockdot_box2d/rockdot_box2d.dart';

void main() {
  html.CanvasElement stageEl = html.querySelector('#stage');

  /* Startup StageXL */
  Stage stage = new Stage(stageEl, webGL: false, color: 0xFF292C2C, width: 640, height: 320);

  stage.scaleMode = StageScaleMode.NO_SCALE;
  stage.align = StageAlign.TOP_LEFT;
  stage.focus = stage;

  RenderLoop renderLoop = new RenderLoop();
  renderLoop.addStage(stage);
  
  stage.addChild( new Main() );
}
