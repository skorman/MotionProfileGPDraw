package motionProfile;

public class MotionProfile {
	
	private double Kp, Ki, Kd, Ka, Kv, goal, cruiseVel,
	maxAcc = 0.0005, cruiseVelScaleFactor = 0.75;
	private double lastTime;
	private Segment currentSegment = new Segment(0, 0, 0);
	private Segment nextSegment = new Segment(0, 0, 0);
	
	public MotionProfile(){
	}
	
	public void setScaleFactor(double factor){
		this.cruiseVelScaleFactor = factor;
	}
	
	public MotionProfile(double Kp, double Ki, double Kd, double Ka,
			double Kv){
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		this.Ka = Ka;
		this.Kv = Kv;
	}
	
	private enum MotionState{
		ACCELERATING, CRUISING, DECELERATING, END
	}
	
	private MotionState state = MotionState.END;
	
	private void setState(MotionState newState){
		state = newState;
	}
	
	private MotionState getState(){
		return state;
	}

	private static class Segment{
		public double pos = 0, vel = 0, acc = 0;
		
		public Segment(double pos, double vel, double acc){
			this.pos = pos;
			this.vel = vel;
			this.acc = acc;
		}
	}
	
	public void configureNewProfile(double distance){
		this.goal = distance;
		this.maxAcc = 0.0005;
		if(distance < 0){
			this.maxAcc *= -1;
		}
		this.cruiseVel = getCruiseVel(this.goal);
		if(distance < 0){
			this.cruiseVel *= -1;
		}
		this.currentSegment = new Segment(0, 0, 0);
		setState(MotionState.ACCELERATING);
		lastTime = 0.1;
	}
	
	public void configureNewProfile(double Kp, double Ki, double Kd, double Ka,
			double Kv, double distance){
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		this.Ka = Ka;
		this.Kv = Kv;
		this.goal = distance;
		this.cruiseVel = getCruiseVel(this.goal);
		this.maxAcc = 0.000005;
		this.currentSegment = new Segment(0, 0, 0);
		this.cruiseVelScaleFactor = 0.05;
		setState(MotionState.ACCELERATING);
		lastTime = 0.1;
	}
	
	public void setProfileConstants(double Kp, double Ki, double Kd, double Ka,
			double Kv){
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		this.Ka = Ka;
		this.Kv = Kv;
	}
	
	private double getCruiseVel(double distance){
		double halfDist = distance / 2;
		double maxVelOverHalfDistance = Math.sqrt(2 * halfDist * maxAcc);
		double vel = Math.min(maxVelOverHalfDistance * cruiseVelScaleFactor, 0.565); //0.565
		return vel;
	}
	
	/*
	public void initializeTime(){
		lastTime = Timer.getFPGATimestamp();
	}
	*/
	public double getVel(){
		return currentSegment.vel;
	}
	
	public double calculate(double time){
		double dt;
		double currentTime = time;
		dt = currentTime - lastTime;
		lastTime = currentTime;
		double currentVel = currentSegment.vel;
		double distanceToGo = goal - currentSegment.pos;
		
		double t_to_cruise = (cruiseVel - currentVel) / maxAcc; //time to accelerate to cruise speed
		double x_to_cruise = currentVel * t_to_cruise + .5 * maxAcc * t_to_cruise * t_to_cruise; //distance to get to cruise speed
		
		double t_to_zero = Math.abs(cruiseVel / maxAcc); //time to get to zero speed from cruise speed
		double x_to_zero = currentVel * t_to_zero - .5 * maxAcc * t_to_zero * t_to_zero; //distance to get to zero speed
		double cruiseX;
		if(goal > 0){
			cruiseX  = Math.max(0, distanceToGo - x_to_cruise - x_to_zero);
		}
		else{
			cruiseX  = Math.min(0, distanceToGo - x_to_cruise - x_to_zero);
		}
		double cruiseT = Math.abs(cruiseX / cruiseVel);
		
		if (getState() == MotionState.ACCELERATING){
			if (t_to_cruise < dt){
				setState(MotionState.CRUISING);
			}
		}
		
		if(getState() == MotionState.CRUISING){
			if(cruiseT < dt){
				setState(MotionState.DECELERATING);
			}
		}
		
		if(getState() == MotionState.DECELERATING){
			if(t_to_cruise + cruiseT + t_to_zero < dt){
				setState(MotionState.END);
			}
		}
		
		if(getState() == MotionState.ACCELERATING){
			nextSegment.pos = currentVel * dt + .5 * maxAcc * dt * dt;
			nextSegment.vel = currentVel + dt * maxAcc;
			nextSegment.acc = maxAcc;
		}
		else if(getState() == MotionState.CRUISING){
			nextSegment.pos = x_to_cruise + cruiseVel * dt;
			nextSegment.vel = cruiseVel;
			nextSegment.acc = 0;
		}
		else if(getState() == MotionState.DECELERATING){
			nextSegment.pos = currentVel * dt - 0.5 *maxAcc * dt * dt;
			nextSegment.vel = currentVel - maxAcc * dt;
			nextSegment.acc = -maxAcc;
		}
		else{
			nextSegment.pos = 0;
			nextSegment.vel = 0;
			nextSegment.acc = 0;
		}
		
		currentSegment.pos += nextSegment.pos;
		currentSegment.vel = nextSegment.vel;
		currentSegment.acc = nextSegment.acc;
				
		double output = Kv * currentSegment.vel + Ka * currentSegment.acc;
		return output;
	}
	
	public boolean isFinishedTrajectory() {
        return Math.abs(currentSegment.pos - goal) < 0.05
                && Math.abs(currentSegment.vel) < 0.01;
    }
}
