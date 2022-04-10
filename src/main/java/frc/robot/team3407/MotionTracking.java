package frc.robot.team3407;

import edu.wpi.first.math.filter.LinearFilter;


public class MotionTracking {
	
	public static class LinearMotionVector {

		public double position = 0, velocity = 0, acceleration = 0;
		
		public LinearMotionVector() {}
		public LinearMotionVector(double p, double v, double a) {
			this.position = p; 
			this.velocity = v; 
			this.acceleration = a;
		}
		public LinearMotionVector(LinearMotionVector other) {
			this.position = other.position; 
			this.velocity = other.velocity; 
			this.acceleration = other.acceleration;
		}
		
		public void copyFrom(LinearMotionVector other) { 
			this.position = other.position; 
			this.velocity = other.velocity; 
			this.acceleration = other.acceleration; 
		}
	
	}

	public static class LinearMotionIntegrator {
		
		private LinearMotionVector vect = new LinearMotionVector();
		private final LinearFilter filter;
		private final double vel_threshold, acc_threshold;

		public LinearMotionIntegrator(int filter_len) {
			this.filter = LinearFilter.movingAverage(filter_len);
			this.vel_threshold = 0.0;
			this.acc_threshold = 0.0;
		}
		public LinearMotionIntegrator(int filter_len, double vel_thresh, double acc_thresh) {
			this.filter = LinearFilter.movingAverage(filter_len);
			this.vel_threshold = vel_thresh;
			this.acc_threshold = acc_thresh;
		}

		public void update(double delta_t) {

		}
		public void update(double acc, double delta_t) {
			acc = this.filter.calculate(acc);
			if(Math.abs(this.vect.acceleration - acc) < this.acc_threshold) {
				acc = 0;
			}
			//position += new distance->{average velocity->{old velocity + delta velocity->{average acceleration * delta time} / 2} * delta_t}
			this.vect.position += (this.vect.velocity * 2.0 + (this.vect.acceleration + acc) / 2.0 * delta_t) / 2.0 * delta_t;
			this.vect.velocity += (this.vect.acceleration + acc) / 2.0 * delta_t;
			this.vect.acceleration = acc;

			if(Math.abs(this.vect.velocity) <= this.vel_threshold) {
				this.vect.velocity = 0;
			}
		}
		public void updateVel(double vel, double delta_t) {
			//System.out.println("DEBUG: " + vel);
			this.vect.position += (this.vect.velocity + vel)/2.0*delta_t;
			this.vect.acceleration = (vel - this.vect.velocity)/delta_t;
			this.vect.velocity = vel;
		}
		public LinearMotionVector get() { 
			return new LinearMotionVector(this.vect); 
		}
		public void set(LinearMotionVector vect) { 
			this.vect.copyFrom(vect); 
		}
		public double getPosition() { 
			return this.vect.position; 
		}
		public double getVelocity() { 
			return this.vect.velocity; 
		}
		public double getAcceleration() { 
			return this.vect.acceleration; 
		}

	}

}
