package motionProfile;

public class SplineProfile {

	private double Kp, Ki, Kd, Ka, Kv, goal, cruiseVel, 
	splineCruiseVel, maxAcc, cruiseVelScaleFactor, splineRadius,
	splineAngle;
	private double lastTime;
	public Segment currentSegment = new Segment(0, 0, 0);
	public Segment nextSegment = new Segment(0, 0, 0);
	public Segment currentOuterSegment = new Segment(0, 0, 0);
	public Segment currentInnerSegment = new Segment(0, 0, 0);
	public Segment nextOuterSegment = new Segment(0, 0, 0);
	public Segment nextInnerSegment = new Segment(0, 0, 0);
	private double maxSplineVel;
	private double width = 6;
	private double rightOutput = 0, leftOutput = 0;
	
	public SplineProfile(double Kp, double Ki, double Kd, double Ka,
			double Kv){
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		this.Ka = Ka;
		this.Kv = Kv;
		currentSegment = new Segment(0, 0, 0);
		nextSegment = new Segment(0, 0, 0);
		currentOuterSegment = new Segment(0, 0, 0);
		currentInnerSegment = new Segment(0, 0, 0);
		nextOuterSegment = new Segment(0, 0, 0);
		nextInnerSegment = new Segment(0, 0, 0);
	}
	
	private static class Segment{
		public double pos, vel, acc;
		
		public Segment(double pos, double vel, double acc){
			this.pos = pos;
			this.vel = vel;
			this.acc = acc;
		}
	}
	
	
	public void configureNewProfile(double distance, double radius, double angle, double pointToAvoid){
		this.goal = pointToAvoid - distance;
		this.maxAcc = 0.04;
		this.cruiseVelScaleFactor = 0.3;
		if(distance < 0){
			this.maxAcc *= -1;
		}
		this.cruiseVel = getCruiseVel(this.goal);
		if(distance < 0){
			this.cruiseVel *= -1;
		}
		this.splineCruiseVel = getSplineCruiseVel(this.goal);
		this.currentSegment = new Segment(0, 0, 0);
		this.nextSegment = new Segment(0, 0, 0);
		this.currentOuterSegment = new Segment(0, 0, 0);
		this.currentInnerSegment = new Segment(0, 0, 0);
		this.nextOuterSegment = new Segment(0, 0, 0);
		this.nextInnerSegment = new Segment(0, 0, 0);
		//setState(MotionState.ACCELERATING);
		//lastTime = Timer.getFPGATimestamp();
	}
	
	private enum MotionState{
		ACCELERATING, CRUISING, DECELERATING, SPLINE, END
	}
	
	private MotionState state = MotionState.END;
	
	private void setState(MotionState newState){
		state = newState;
	}
	
	private MotionState getState(){
		return state;
	}
	
	private double getCruiseVel(double distance){
		double halfDist = distance / 2;
		double maxVelOverHalfDistance = Math.sqrt(2 * halfDist * maxAcc);
		return Math.min(maxVelOverHalfDistance * cruiseVelScaleFactor, Constants.maxCruiseSpeed);
	}
	
	private double getSplineCruiseVel(double distance){
		if(distance > 0) return Math.min(this.cruiseVel, maxSplineVel);
		return Math.max(this.cruiseVel, maxSplineVel);
	}
	
	public void calculate(){
		double dt;
		double currentTime = Timer.getFPGATimestamp();
		dt = currentTime - lastTime;
		lastTime = currentTime;
		
		if(getState() == MotionState.SPLINE){
			splineCalculate(dt);
		}
		
		double currentVel = currentSegment.vel;
		double distanceToGo = goal - currentSegment.pos;
		
		double t_to_cruise = (cruiseVel - currentVel) / maxAcc; //time to accelerate to cruise speed
		double x_to_cruise = currentVel * t_to_cruise + .5 * maxAcc * t_to_cruise * t_to_cruise; //distance to get to cruise speed
		
		double t_to_spline = Math.abs((cruiseVel - splineCruiseVel) / maxAcc); //time to get to zero speed from cruise speed
		double x_to_spline = currentVel * t_to_spline - .5 * maxAcc * t_to_spline * t_to_spline; //distance to get to zero speed
		
		double cruiseX;
		if(goal > 0){
			cruiseX  = Math.max(0, distanceToGo - x_to_cruise - x_to_spline);
		}
		else{
			cruiseX  = Math.min(0, distanceToGo - x_to_cruise - x_to_spline);
		}
		double cruiseT = Math.abs(cruiseX / cruiseVel);
		
		if (getState() == MotionState.ACCELERATING){
			if (t_to_cruise < dt){
				setState(MotionState.CRUISING);
			}
		}
		
		if(getState() == MotionState.CRUISING){
			if(t_to_cruise + cruiseT < dt){
				setState(MotionState.DECELERATING);
			}
		}
		
		if(getState() == MotionState.DECELERATING){
			if(t_to_cruise + cruiseT + t_to_spline < dt){
				setState(MotionState.SPLINE);
			}
		}
		
		if(getState() == MotionState.ACCELERATING){
			nextSegment.pos = currentVel * dt + .5 * maxAcc * dt * dt;
			nextSegment.vel = currentVel + dt * maxAcc;
			nextSegment.acc = maxAcc;
		}
		else if(getState() == MotionState.CRUISING){
			nextSegment.pos = cruiseVel * dt;
			nextSegment.vel = cruiseVel;
			nextSegment.acc = 0;
		}
		else if(getState() == MotionState.DECELERATING){
			nextSegment.pos = currentVel * dt - 0.5 * maxAcc * dt * dt;
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
		
		currentOuterSegment.pos += nextSegment.pos;
		currentOuterSegment.vel = currentSegment.vel;
		currentOuterSegment.acc = currentSegment.acc;
		
		currentInnerSegment.pos += nextSegment.pos;
		currentInnerSegment.vel = currentSegment.vel;
		currentInnerSegment.acc = currentSegment.acc;
		
		double outerOutput = Kv * currentOuterSegment.vel + Ka * currentOuterSegment.acc;
		double innerOutput = Kv * currentInnerSegment.vel + Ka * currentInnerSegment.acc;
		
		if(splineAngle >= 0) {
			rightOutput = outerOutput;
			leftOutput = innerOutput;
		}
		else{
			rightOutput = innerOutput;
			leftOutput = outerOutput;
		}
	}
	
	public double getRightOutput(){
		return rightOutput;
	}
	
	public double getLeftOutput(){
		return leftOutput;
	}
	
	private void splineCalculate(double delta_t){
		double dt = delta_t;
		
		double currentInnerVel = currentInnerSegment.vel;
		double outerCircDistance = splineAngle * splineRadius;
		
		nextOuterSegment.pos = splineCruiseVel * dt;
		nextOuterSegment.vel = splineCruiseVel;
		nextOuterSegment.acc = 0;
		
		double innerRadius = splineRadius - width;
		double angle = nextOuterSegment.pos / splineRadius;
		
		nextInnerSegment.pos = angle * innerRadius;
		double displacement = nextInnerSegment.pos;
		nextInnerSegment.vel = (displacement / dt * 2) - currentInnerVel;
		double finalVel = nextInnerSegment.vel;
		nextInnerSegment.acc = (finalVel - currentInnerVel) / dt;
		
		currentOuterSegment.pos += nextOuterSegment.pos;
		currentOuterSegment.vel = nextOuterSegment.vel;
		currentOuterSegment.acc = nextOuterSegment.acc;
		
		currentInnerSegment.pos += nextInnerSegment.pos;
		currentInnerSegment.vel = nextInnerSegment.vel;
		currentInnerSegment.acc = nextInnerSegment.acc;
	
		double x_to_goal = outerCircDistance + currentSegment.pos - displacement;
		double t_to_goal = Math.abs(x_to_goal / splineCruiseVel);
		
		if(t_to_goal < dt){
			setState(MotionState.ACCELERATING);
		}
	}
}
