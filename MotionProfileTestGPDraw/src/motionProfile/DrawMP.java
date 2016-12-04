package motionProfile;

import gpdraw.*;

public class DrawMP {

	
	MotionProfile mp = new MotionProfile(0, 0, 0, 0.0, 0.05);
	
	public DrawMP(){
	}
	
	public void run(){
		double i = 0.1;
		SketchPad paper = new SketchPad(1000, 1000);
		DrawingTool pen = new DrawingTool(paper);
		pen.move(0, 0);
		mp.configureNewProfile(-5);
		while(!mp.isFinishedTrajectory()){
			i += (Math.random() * 0.2);
			pen.up();
			pen.move(i - 150, (mp.calculate(i) * 10000) - 150);
			pen.down();
			pen.drawCircle(0.01);
		}
		System.out.println(i);
	}
	
}
