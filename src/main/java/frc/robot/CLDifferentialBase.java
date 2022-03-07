package frc.robot;

import frc.robot.modules.common.LambdaCommand;
import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.common.drive.Types.*;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.*;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;


// 'Closed-Loop' Differential drivebase
public class CLDifferentialBase extends DriveBase {

	/**
	 * Contains all parameters required for closed-loop drivebase control. All accessable variables use meters for units of length,
	 * but any unit type can be used to construct as long as a conversion constant is supplied and the same units are used for all params.
	 */
	public static class CLParams {

		public final double track_width_meters;
		public final double wheel_diameter_meters;

		public final double static_voltage;					// "kS" (voltage) -> "voltage to overcome static friction"
		public final double volt_seconds_per_meter;			// "kV" (volts * seconds / meters) -> "voltage to hold a velocity"
		public final double volt_seconds_sqrd_per_meter;	// "kA" (volts * seconds^2 / meters) -> "voltage to hold an acceleration"
		public final double volt_seconds_per_meter_gain;	// "kP" (voltage * seconds / meters) -> "voltage applied for detected velocity error"

		public final double max_velocity_meters_per_sec;			// meters / second
		public final double max_acceleration_meters_per_sec_sqrd;	// meters / second^2

		public final double units2meters;

		/** Make sure all parameters use the same units of length! */
		public CLParams(
			double track_width, double wheel_diam,
			double kS, double kV, double kA, double kP,
			double max_vel, double max_acc,
			double units2meters
		) {
			this.track_width_meters = track_width * units2meters;
			this.wheel_diameter_meters = wheel_diam * units2meters;
			this.static_voltage = kS;
			this.volt_seconds_per_meter = kV / units2meters;
			this.volt_seconds_sqrd_per_meter = kA / units2meters;
			this.volt_seconds_per_meter_gain = kP / units2meters;
			this.max_velocity_meters_per_sec = max_vel * units2meters;
			this.max_acceleration_meters_per_sec_sqrd = max_acc * units2meters;
			this.units2meters = units2meters;
		}
		/** Make sure all parameters use meters for units of length! */
		public CLParams(
			double track_width, double wheel_diam,
			double kS, double kV, double kA, double kP,
			double max_vel, double max_acc
		) {
			this(track_width, wheel_diam, kS, kV, kA, kP, max_vel, max_acc, 1.0);
		}

		public SimpleMotorFeedforward getFeedforward() {
			return new SimpleMotorFeedforward(
				this.kS(),
				this.kV(),
				this.kA()
			);
		}

		public double kS() { return this.static_voltage; }
		public double kV() { return this.volt_seconds_per_meter; }
		public double kA() { return this.volt_seconds_sqrd_per_meter; }
		public double kP() { return this.volt_seconds_per_meter_gain; }

		public double meters2units() { return 1.0 / this.units2meters; }


	}

	public static void configDefault(BaseMotorController... motors) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].configFactoryDefault();
		}
	}





	private final Gyro gyro;
	private final WPI_TalonSRX left, right;		// front left and right left respectively for 4-motor systems

	public final CLParams params;

	private final DifferentialDriveOdometry odometry;
	private final DifferentialDriveKinematics kinematics;
	private final DifferentialDriveVoltageConstraint voltage_constraint;
	private final SimpleMotorFeedforward feedforward;

	public CLDifferentialBase(DriveMap_2<WPI_TalonSRX> map, Gyro gy, CLParams params) {
		super(map);
		this.gyro = gy;

		this.left = map.left;
		this.right = map.right;
		this.params = params;

		// apply configs...
		configDefault(this.left, this.right);
		this.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

		this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d());
		this.kinematics = new DifferentialDriveKinematics(this.params.track_width_meters);
		this.feedforward = params.getFeedforward();
		this.voltage_constraint = new DifferentialDriveVoltageConstraint(this.feedforward, this.kinematics, 10);
	}
	public CLDifferentialBase(DriveMap_4<WPI_TalonSRX> map, Gyro gy, CLParams params) {
		super(map);
		this.gyro = gy;

		this.left = map.front_left;
		this.right = map.front_right;
		this.params = params;

		// apply configs...
		configDefault(this.left, this.right, map.back_left, map.back_right);
		this.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		map.back_left.follow(this.left);
		map.back_right.follow(this.right);
		map.back_left.setInverted(InvertType.FollowMaster);
		map.back_right.setInverted(InvertType.FollowMaster);

		this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d());
		this.kinematics = new DifferentialDriveKinematics(this.params.track_width_meters);
		this.feedforward = params.getFeedforward();
		this.voltage_constraint = new DifferentialDriveVoltageConstraint(this.feedforward, this.kinematics, 10);
	}

	@Override public void periodic() {
		this.odometry.update(
			this.gyro.getRotation2d(),
			this.getLeftPositionMeters(),
			this.getRightPositionMeters()
		);
		// update robot field pose if we use that on the dashboard
	}

	public TrajectoryConfig getTrajectoryConfig() {
		return new TrajectoryConfig(
			this.params.max_velocity_meters_per_sec,
			this.params.max_acceleration_meters_per_sec_sqrd
		).setKinematics(this.kinematics).addConstraint(this.voltage_constraint);
	}
	public Command followTrajectory(Trajectory t) {
		return new SequentialCommandGroup(
			new LambdaCommand(()->this.resetOdometry(t.getInitialPose())),
			new RamseteCommand(
				t,
				this::getPose,
				new RamseteController(Constants.ramsete_B, Constants.ramsete_Zeta),
				this.feedforward,
				this.kinematics,
				this::getWheelSpeeds,
				new PIDController(this.params.kP(), 0, 0),
				new PIDController(this.params.kP(), 0, 0),
				this::setDriveVoltage,
				this
			)
		);
	}

	public Pose2d getPose() {	// in meters
		return this.odometry.getPoseMeters();
	}
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(this.getLeftVelocity(), this.getRightVelocity());
	}
	public void resetOdometry(Pose2d p) {
		this.resetEncoders();
		this.odometry.resetPosition(p, this.getRotation());
	}

	public void setDriveVoltage(double l, double r) {
		this.left.setVoltage(l);
		this.right.setVoltage(r);
		super.getDrive().feed();
	}

	public void resetEncoders() {
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
	}

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

	public double getLeftPositionMeters() {
		return this.getRawLeftPosition() / Constants.srx_mag_units_per_revolution * this.params.wheel_diameter_meters;
	}
	public double getRightPositionMeters() {
		return this.getRawRightPosition() / Constants.srx_mag_units_per_revolution * this.params.wheel_diameter_meters;
	}
	public double getLeftVelocity() {	// in meters per second
		return this.getRawLeftVelocity() * 10 / Constants.srx_mag_units_per_revolution * this.params.wheel_diameter_meters;
	}
	public double getRightVelocity() {	// in meters per second
		return this.getRawRightVelocity() * 10 / Constants.srx_mag_units_per_revolution * this.params.wheel_diameter_meters;
	}

	public void zeroHeading() {
		this.gyro.reset();
	}

	public double getContinuousAngle() {	// in degrees
		return this.gyro.getAngle();
	}
	public double getHeading() {			// from -180 to 180
		return this.gyro.getRotation2d().getDegrees();
	}
	public double getTurnRate() {	// in degrees per second
		return -this.gyro.getRate();
	}
	public Rotation2d getRotation() {
		return this.gyro.getRotation2d();
	}


}