package frc.robot.team3407;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.SPI;


// For some reason the ADIS16470_IMU driver does not implement Gyro, even though it already has the correct method names
public class ADIS16470 extends ADIS16470_IMU implements Gyro, Accelerometer {

	public ADIS16470() { super(); }
	public ADIS16470(IMUAxis yaw_axis, SPI.Port port, CalibrationTime cal_time) {
		super(yaw_axis, port, cal_time);
	}

	@Override public double getAngle() {
		return super.getAngle() * -1;	// convert from CCW positive to CW positive
	}
	@Override public double getRate() {
		return super.getRate() * -1;	// convert from CCW positive to CW positive
	}

	public double getX() {
		return super.getAccelX();
	}
	public double getY() {
		return super.getAccelY();
	}
	public double getZ() {
		return super.getAccelZ();
	}
	public void setRange(Range r) {

	}


}