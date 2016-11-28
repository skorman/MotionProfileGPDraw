package motionProfile;

public class Main {

	static DrawMP draw;
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		draw = new DrawMP();
		
		draw.mp.setScaleFactor(0.25);
		
		draw.run();
		
		draw.mp.setScaleFactor(0.5);	
		
		draw.run();
		
		draw.mp.setScaleFactor(0.8);
		
		draw.run();
	}

}
