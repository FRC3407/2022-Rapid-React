package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;

public class Positions {
	private final ADIS16470_IMU gyro_imu;
	private final Accelerometer gyro_rio;

	public Positions() {
		this.gyro_imu = new ADIS16470_IMU();
		this.gyro_rio = new BuiltInAccelerometer();
	}
	public Positions(IMUAxis yaw_axis) {
		this.gyro_imu = new ADIS16470_IMU(yaw_axis, Port.kOnboardCS0, CalibrationTime._4s);
		this.gyro_rio = new BuiltInAccelerometer();
	}
	public Positions(CalibrationTime calibration_time) {
		this.gyro_imu = new ADIS16470_IMU(IMUAxis.kZ, Port.kOnboardCS0, calibration_time);
		this.gyro_rio = new BuiltInAccelerometer();
	}
	public Positions(BuiltInAccelerometer.Range rio_range_gs) {
		this.gyro_imu = new ADIS16470_IMU();
		this.gyro_rio = new BuiltInAccelerometer(rio_range_gs);
	}
	public Positions(CalibrationTime calibration_time, IMUAxis yaw_axis) {
		this.gyro_imu = new ADIS16470_IMU(yaw_axis, Port.kOnboardCS0, calibration_time);
		this.gyro_rio = new BuiltInAccelerometer();
	}
	public Positions(CalibrationTime calibration_time, IMUAxis yaw_axis, BuiltInAccelerometer.Range rio_range_gs) {
		this.gyro_imu = new ADIS16470_IMU(yaw_axis, Port.kOnboardCS0, calibration_time);
		this.gyro_rio = new BuiltInAccelerometer(rio_range_gs);
	}

	public ADIS16470_IMU getIMU() {
		return this.gyro_imu;
	}
	public Accelerometer getBuiltIn() {
		return this.gyro_rio;
	}

	public double getAngle() {	// in degrees
		return this.gyro_imu.getAngle();
	}
	public double getTurnRate() {	// in degrees per second
		return this.gyro_imu.getRate();
	}



	public static class LinearMotionVector {
		public double position = 0, velocity = 0, acceleration = 0;
		
		public LinearMotionVector() {}
		public LinearMotionVector(double p, double v, double a) {position = p; velocity = v; acceleration = a;}
		public LinearMotionVector(LinearMotionVector other) {position = other.position; velocity = other.velocity; acceleration = other.acceleration;}
		public void copy(LinearMotionVector other) { this.position = other.position; this.velocity = other.velocity; this.acceleration = other.acceleration; }
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
			this.vect.position += (this.vect.velocity + ((this.vect.acceleration + acc)/2.0*delta_t))*delta_t;
			this.vect.velocity += (this.vect.acceleration + acc)/2.0*delta_t;
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
		public LinearMotionVector get() { return new LinearMotionVector(this.vect); }
		public void set(LinearMotionVector vect) { this.vect.copy(vect); }
		public double getPosition() { return this.vect.position; }
		public double getVelocity() { return this.vect.velocity; }
		public double getAcceleration() { return this.vect.acceleration; }
	}

}