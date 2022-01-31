package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SPI;


// For some reason the ADIS16470_IMU driver does not implement Gyro, even though it already has the correct method names
public class ADIS16470_Gyro extends ADIS16470_IMU implements Gyro {
	
	public ADIS16470_Gyro() { super(); }
	public ADIS16470_Gyro(IMUAxis yaw_axis, SPI.Port port, CalibrationTime cal_time) {
		super(yaw_axis, port, cal_time);
	}

}