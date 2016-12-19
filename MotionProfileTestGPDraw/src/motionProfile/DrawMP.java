package motionProfile;

import gpdraw.*;

public class DrawMP {

	
	MotionProfile mp = new MotionProfile(0, 0, 0, 0.0, 0.05);
	SplineProfile sp = new SplineProfile(0, 0, 0, 0.05, 0.05);
	
	public DrawMP(){
	}
	
	public void run(){
		double i = 0.1;
		SketchPad paper = new SketchPad(1000, 1000);
		DrawingTool pen = new DrawingTool(paper);
		pen.move(0, 0);
		sp.configureNewProfile(50, 4, Math.toRadians(145), 20);
		while(!sp.isFinishedTrajectory()){
			i += (Math.random() * 0.2);
			sp.calculate(i);
			
			pen.up();
			pen.move(i - 250, (sp.getRightOutput() * 10000) - 150);
			//System.out.println(sp.getRightOutput());
			pen.down();
			pen.drawCircle(0.01);
			
			pen.up();
			pen.move(i - 250, (sp.getLeftOutput() * 10000) + 150);
			pen.down();
			pen.drawCircle(0.01);
			
		}
		System.out.println(i);
	}
	
}
