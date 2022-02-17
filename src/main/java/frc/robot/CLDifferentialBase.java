package frc.robot;

import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.common.drive.Types.*;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;


// 'Closed-Loop' Differential drivebase
public class CLDifferentialBase extends DriveBase {	// not completely safe yet as an extension of DriveBase (managing control)

	public static void configDefault(BaseMotorController... motors) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].configFactoryDefault();
		}
	}

	private final Gyro 
		gyro = new ADIS16470();

	private final WPI_TalonSRX
		left, right;

	private final DifferentialDriveOdometry 
		odometry;

	public CLDifferentialBase(DriveMap_2<WPI_TalonSRX> map) {
		super(map);

		this.left = map.left;
		this.right = map.right;
		// apply configs...
		configDefault(this.left, this.right);
		this.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

		this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d());
	}
	public CLDifferentialBase(DriveMap_4<WPI_TalonSRX> map) {
		super(map);

		this.left = map.front_left;
		this.right = map.front_right;
		// apply configs...
		configDefault(this.left, this.right, map.back_left, map.back_right);
		this.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		map.back_left.follow(this.left);
		map.back_right.follow(this.right);
		map.back_left.setInverted(InvertType.FollowMaster);
		map.back_right.setInverted(InvertType.FollowMaster);

		this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d());
	}

	@Override public void periodic() {
		this.odometry.update(
			this.gyro.getRotation2d(),
			this.left.getSelectedSensorPosition(),	// need to convert from "raw units"
			this.right.getSelectedSensorPosition()
		);
	}

	// methods for getting each encoder speed/position in actual units
	public double getRawLeftPosition() {
		return this.left.getSelectedSensorPosition();
	}
	public double getRawRightPosition() {
		return this.right.getSelectedSensorPosition();
	}
	public double getRawLeftVelocity() {
		return this.left.getSelectedSensorVelocity();
	}
	public double getRawRightVelocity() {
		return this.right.getSelectedSensorVelocity();
	}
	// methods for total distance

	public double getLeftVelocity() {	// in meters per second
		return this.getRawLeftVelocity() * Constants.srx_mag_rawunits_to_meters * 10;
	}
	public double getRightVelocity() {	// in meters per second
		return this.getRawRightVelocity() * Constants.srx_mag_rawunits_to_meters * 10;
	}

	public double getAngle() {		// in degrees
		return this.gyro.getAngle();
	}
	public double getHeading() {
		return this.gyro.getRotation2d().getDegrees();
	}
	public double getTurnRate() {	// in degrees per second
		return -this.gyro.getRate();
	}
	public Rotation2d getRotation() {
		return this.gyro.getRotation2d();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(this.getLeftVelocity(), this.getRightVelocity());
	}

	public void resetOdometry(Pose2d p) {
		this.resetEncoders();
		this.odometry.resetPosition(p, this.getRotation());
	}
	public void resetEncoders() {
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
	}
	public void zeroHeading() {
		this.gyro.reset();
	}

	
	
	
}