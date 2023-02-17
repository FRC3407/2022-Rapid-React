package frc.robot;

import java.util.function.DoubleSupplier;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;

import frc.robot.team3407.drive.DriveBase;
import frc.robot.team3407.drive.Types.*;


/**
 * Extends {@link DriveBase} and represents a differential drivebase with srx-mag encoders. Constructors require a motor map of the type 
 * {@link WPI_TalonSRX} in order to interface with the encoders. All 'super' functionality should still be possible, and a drivebase can be 
 * created with either 2 or 4 motors. Note that it is expected that the encoders are plugged into the front motorcontrollers for each side. 
 * This class allows for trajectory-following functionality provided that the supplied constants ("clparams") are accurate. Additionally,
 * the position of the robot can be viewed in the dashboard from the "Robot Position" table under this subsystem's sendable table. 
 */
public class ClosedLoopDifferentialDrive extends DriveBase {

	/**
	 * Contains all parameters required for closed-loop drivebase control. All accessable variables use meters for units of length,
	 * but any unit type can be used to construct as long as a conversion constant is supplied and the same units are used for all params.
	 */
	public static class CLDriveParams {

		public final double track_width_meters;
		public final double wheel_diameter_meters;

		public final double static_voltage;					// "kS" (voltage) -> "voltage to overcome static friction"
		public final double volt_seconds_per_meter;			// "kV" (volts * seconds / meters) -> "voltage to hold a velocity"
		public final double volt_seconds_sqrd_per_meter;	// "kA" (volts * seconds^2 / meters) -> "voltage to hold an acceleration"
		public final double volt_seconds_per_meter_gain;	// "kP" (voltage * seconds / meters) -> "voltage applied for detected velocity error"

		public final double max_voltage;						// maximum voltage that can be supplied to motors
		public final double max_velocity_meters_per_sec;			// meters / second
		public final double max_acceleration_meters_per_sec_sqrd;	// meters / second^2

		public final double units2meters;

		/** Make sure all parameters use the same units of length! */
		public CLDriveParams(
			double track_width, double wheel_diam,
			double kS, double kV, double kA, double kP,
			double max_volts, double max_vel, double max_acc,
			double units2meters
		) {
			this.track_width_meters = track_width * units2meters;
			this.wheel_diameter_meters = wheel_diam * units2meters;
			this.static_voltage = kS;
			this.volt_seconds_per_meter = kV / units2meters;
			this.volt_seconds_sqrd_per_meter = kA / units2meters;
			this.volt_seconds_per_meter_gain = kP / units2meters;
			this.max_voltage = max_volts;
			this.max_velocity_meters_per_sec = max_vel * units2meters;
			this.max_acceleration_meters_per_sec_sqrd = max_acc * units2meters;
			this.units2meters = units2meters;
		}
		/** Make sure all parameters use meters for units of length! */
		public CLDriveParams(
			double track_width, double wheel_diam,
			double kS, double kV, double kA, double kP,
			double max_volts, double max_vel, double max_acc
		) {
			this(track_width, wheel_diam, kS, kV, kA, kP, max_volts, max_vel, max_acc, 1.0);
		}

		public SimpleMotorFeedforward getFeedforward() {
			return new SimpleMotorFeedforward(
				this.kS(),
				this.kV(),
				this.kA()
			);
		}
		public PIDController getFeedbackController() {
			return new PIDController(this.kP(), 0.0, 0.0);
		}
		public ProfiledPIDController getProfiledFeedbackController() {
			return new ProfiledPIDController(
				this.kP(), 0.0, 0.0,
				new TrapezoidProfile.Constraints(
					this.max_velocity_meters_per_sec,
					this.max_acceleration_meters_per_sec_sqrd
				)
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





	public final Gyro gyro;
	private final WPI_TalonSRX left, right;		// front left and front right respectively for 4-motor systems

	public final CLDriveParams params;

	private final DifferentialDriveOdometry odometry;
	private final DifferentialDriveKinematics kinematics;
	private final SimpleMotorFeedforward feedforward;

	//private Transform2d position_offset = new Transform2d();
	private Pose2d position_offset = new Pose2d();
	private final Field2d map = new Field2d();

	public ClosedLoopDifferentialDrive(DriveMap_2<WPI_TalonSRX> map, Gyro gy, CLDriveParams params) { this(map, gy, params, Inversions.NEITHER); }
	public ClosedLoopDifferentialDrive(DriveMap_4<WPI_TalonSRX> map, Gyro gy, CLDriveParams params) { this(map, gy, params, Inversions.NEITHER); }
	public ClosedLoopDifferentialDrive(DriveMap_2<WPI_TalonSRX> map, Gyro gy, CLDriveParams params, Inversions ei) {
		super(map);
		this.gyro = gy;
		this.params = params;

		this.left = map.left;
		this.right = map.right;

		this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d());
		this.kinematics = new DifferentialDriveKinematics(this.params.track_width_meters);
		this.feedforward = params.getFeedforward();

		// apply configs...
		configDefault(this.left, this.right);
		this.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
		this.left.setSensorPhase(ei.left);
		this.right.setSensorPhase(ei.right);

		SmartDashboard.putData("Robot Location", this.map);
	}
	public ClosedLoopDifferentialDrive(DriveMap_4<WPI_TalonSRX> map, Gyro gy, CLDriveParams params, Inversions ei) {
		super(map.differentialDowncast());	// motorcontroller groups mess everything up, so only let the super control the front motors, the back can be set via the phoenix api
		this.gyro = gy;
		this.params = params;

		this.left = map.front_left;
		this.right = map.front_right;

		this.odometry = new DifferentialDriveOdometry(this.gyro.getRotation2d());
		this.kinematics = new DifferentialDriveKinematics(this.params.track_width_meters);
		this.feedforward = params.getFeedforward();

		// apply configs...
		configDefault(this.left, this.right, map.back_left, map.back_right);
		this.left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
		this.left.setSensorPhase(ei.left);
		this.right.setSensorPhase(ei.right);

		map.back_left.follow(this.left);	// since we didn't pass these motors to the super, we can just set them to follow the front ones and nothing else will touch them
		map.back_right.follow(this.right);
		map.back_left.setInverted(InvertType.FollowMaster);
		map.back_right.setInverted(InvertType.FollowMaster);

		SmartDashboard.putData("Robot Location", this.map);
	}

	@Override public void periodic() {
		this.odometry.update(
			this.gyro.getRotation2d(),
			this.getLeftPositionMeters(),
			this.getRightPositionMeters()
		);
		this.map.setRobotPose(this.getTotalPose());
	}

	@Override public void initSendable(SendableBuilder b) {
		super.initSendable(b);
		b.addDoubleProperty("Left Distance", ()->this.getLeftPositionMeters(), null);
		b.addDoubleProperty("Right Distance", ()->this.getRightPositionMeters(), null);
		b.addDoubleProperty("Left Velocity", ()->this.getLeftVelocity(), null);
		b.addDoubleProperty("Right Velocity", ()->this.getRightVelocity(), null);
		b.addDoubleProperty("Rotation (Total)", ()->this.getContinuousAngle(), null);
		b.addDoubleArrayProperty("Output Voltage [L, R]",
            ()->{ return new double[]{
                this.left.getMotorOutputVoltage(),
                this.right.getMotorOutputVoltage()
            }; }, null);
        b.addDoubleArrayProperty("Output Current [L, R]",
            ()->{ return new double[]{
                this.left.getStatorCurrent(),
                this.right.getStatorCurrent()
            }; }, null);
        b.addDoubleArrayProperty("Controller Temps [L, R]",
            ()->{ return new double[]{
                this.left.getTemperature(),
                this.right.getTemperature()
            }; }, null);
	}

	public TankDriveVelocity tankDriveVelocity(DoubleSupplier lvel, DoubleSupplier rvel) {
		return new TankDriveVelocity(this, lvel, rvel);
	}
	public TankDriveVelocity_P tankDriveVelocityProfiled(DoubleSupplier lvel, DoubleSupplier rvel) {
		return new TankDriveVelocity_P(this, lvel, rvel);
	}
	public FollowTrajectory followTrajectory(Trajectory t) {
		return new FollowTrajectory(this, t);
	}
	public FollowTrajectory followTrajectory(Path json_path) {
		return new FollowTrajectory(this, json_path);
	}
	public FollowTrajectory followSingleTrajectory(Trajectory t) {	// stops when complete
		return new FollowTrajectory(this, t, true);
	}
	public FollowTrajectory followSingleTrajectory(Path json_path) {	// stops when complete
		return new FollowTrajectory(this, json_path, true);
	}
	public GoTo autoPosition(Pose2d p) {
		return new GoTo(this, p);
	}

	// "Setters" -> require command key
	public void resetOdometry(Pose2d p, CLDriveCommand c) {
		this.resetEncoders(c);
		this.position_offset = this.getTotalPose();
		this.odometry.resetPosition(p, this.getRotation());
	}
	public void setDriveVoltage(double lv, double rv, CLDriveCommand c) {
		this.left.setVoltage(lv);
		this.right.setVoltage(rv);
		super.getDrive().feed();
	}
	public void resetEncoders(CLDriveCommand c) {
		this.left.setSelectedSensorPosition(0.0);
		this.right.setSelectedSensorPosition(0.0);
	}
	public void zeroHeading(CLDriveCommand c) {
		this.gyro.reset();
	}

	public void setInitial(Pose2d init) {	// set the initial position based on where the robot is located on the field - used primarily for accurate dashboard view
		if(this.position_offset.equals(new Pose2d())) {
			this.position_offset = init;
		}
	}


	public Pose2d getCurrentPose() {	// in meters
		return this.odometry.getPoseMeters();
	}
	public Pose2d getTotalPose() {	// in meters
		//return this.odometry.getPoseMeters().plus(this.position_offset);
		Pose2d current = this.getCurrentPose();
		return this.position_offset.plus(new Transform2d(current.getTranslation(), current.getRotation()));
	}
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(this.getLeftVelocity(), this.getRightVelocity());
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
		return this.getRawLeftPosition()				// output is in encoder units...
			/ Constants.srx_mag_units_per_revolution		// to get total rotations
			* this.params.wheel_diameter_meters * Math.PI;	// to get total distance
			//* this.encoder_invrt.leftSign();				// to get total distance in the correct direction
	}
	public double getRightPositionMeters() {
		return this.getRawRightPosition()				// ^^^
			/ Constants.srx_mag_units_per_revolution
			* this.params.wheel_diameter_meters * Math.PI;
			//* this.encoder_invrt.rightSign();
	}
	public double getLeftVelocity() {	// in meters per second
		return this.getRawLeftVelocity()				// output is in encoder units per 100 ms
			* 10											// to get encoder units per second
			/ Constants.srx_mag_units_per_revolution		// to get rotations per second
			* this.params.wheel_diameter_meters * Math.PI;	// to get meters per second
			//* this.encoder_invrt.leftSign();				// to get meters per second in the correct direction
	}
	public double getRightVelocity() {	// in meters per second
		return this.getRawRightVelocity()				// ^^^
			* 10
			/ Constants.srx_mag_units_per_revolution
			* this.params.wheel_diameter_meters * Math.PI;
			//* this.encoder_invrt.rightSign();
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

	public DifferentialDriveVoltageConstraint getVoltageConstraint() {
		return new DifferentialDriveVoltageConstraint(
			this.feedforward, this.kinematics, this.params.max_voltage
		);
	}
	public TrajectoryConfig getTrajectoryConfig() {
		return new TrajectoryConfig(
			this.params.max_velocity_meters_per_sec,
			this.params.max_acceleration_meters_per_sec_sqrd
		).setKinematics(
			this.kinematics
		).addConstraint(
			this.getVoltageConstraint()
		);
	}



	public static abstract class CLDriveCommand extends DriveCommandBase {

		protected final ClosedLoopDifferentialDrive drivebase_cl;

		protected CLDriveCommand(ClosedLoopDifferentialDrive db) {
			super(db);
			this.drivebase_cl = db;
		}

		protected void resetOdometry(Pose2d p) {
			this.drivebase_cl.resetOdometry(p, this);
		}
		protected void setDriveVoltage(double lv, double rv) {
			this.drivebase_cl.setDriveVoltage(lv, rv, this);
		}
		protected void resetEncoders() {
			this.drivebase_cl.resetEncoders(this);
		}
		protected void zeroHeading() {
			this.drivebase_cl.zeroHeading(this);
		}


	}

	public static class TankDriveVelocity extends CLDriveCommand {

		protected final DoubleSupplier left, right;
		private final PIDController left_fb, right_fb;

		public TankDriveVelocity(
			ClosedLoopDifferentialDrive db, DoubleSupplier l, DoubleSupplier r
		) {
			super(db);
			this.left = l;
			this.right = r;
			this.left_fb = db.params.getFeedbackController();
			this.right_fb = db.params.getFeedbackController();
		}

		protected void applyVelocity(double lv, double rv) {
			double
				lc = super.drivebase_cl.getLeftVelocity(),  // the actual velocity
				rc = super.drivebase_cl.getRightVelocity();
			super.setDriveVoltage(
				this.drivebase_cl.feedforward.calculate(lv) +  // the calculated feedforward
					this.left_fb.calculate(lc, lv),   		// add the feedback adjustment
				this.drivebase_cl.feedforward.calculate(rv) +
					this.right_fb.calculate(rc, rv)
			);
		}

		@Override
		public void initialize() {
			this.left_fb.reset();
			this.right_fb.reset();
		}
		@Override
		public void execute() {
			this.applyVelocity(
				this.left.getAsDouble(),
				this.right.getAsDouble()
			);
		}
		@Override
		public void end(boolean interrupted) {
			super.setDriveVoltage(0.0, 0.0);
		}
		@Override
		public boolean isFinished() {
			return false;
		}


	}
	public static class TankDriveVelocity_P extends CLDriveCommand {

		private final DoubleSupplier left, right;
		private final ProfiledPIDController left_fb, right_fb;

		public TankDriveVelocity_P(
			ClosedLoopDifferentialDrive db, DoubleSupplier l, DoubleSupplier r
		) {
			super(db);
			this.left = l;
			this.right = r;
			this.left_fb = db.params.getProfiledFeedbackController();
			this.right_fb = db.params.getProfiledFeedbackController();
		}

		@Override
		public void initialize() {
			this.left_fb.reset(0);
			this.right_fb.reset(0);
		}
		@Override
		public void execute() {
			double
				lt = this.left.getAsDouble(),	// the target velocity from the left input --> METERS PER SECOND
				rt = this.right.getAsDouble(),	// ^^^ for the right side
				lc = super.drivebase_cl.getLeftVelocity(),  // the actual velocity
				rc = super.drivebase_cl.getRightVelocity();
			super.setDriveVoltage(
				super.drivebase_cl.feedforward.calculate(lt) +  // the calculated feedforward
					this.left_fb.calculate(lc, lt),   		// add the feedback adjustment
				super.drivebase_cl.feedforward.calculate(rt) +
					this.right_fb.calculate(rc, rt)
			);
		}
		@Override
		public void end(boolean interrupted) {
			super.setDriveVoltage(0.0, 0.0);
		}
		@Override
		public boolean isFinished() {
			return false;
		}


	}

	/**
	 * Extends CLDriveCommand and wraps a RamseteCommand (and prints status)
	 */
	public static class FollowTrajectory extends CLDriveCommand {

		private final Trajectory trajectory;
		private final RamseteCommand controller;
		private final boolean stop;

		public FollowTrajectory(ClosedLoopDifferentialDrive db, Trajectory t) { this(db, t, false); }
		public FollowTrajectory(ClosedLoopDifferentialDrive db, Path json_path) { this(db, json_path, false); }
		public FollowTrajectory(ClosedLoopDifferentialDrive db, Trajectory t, boolean s) {
			super(db);
			this.trajectory = t;
			this.stop = s;
			this.controller = new RamseteCommand(
				this.trajectory,
				super.drivebase_cl::getCurrentPose,
				new RamseteController(Constants.ramsete_B, Constants.ramsete_Zeta),
				super.drivebase_cl.feedforward,
				super.drivebase_cl.kinematics,
				super.drivebase_cl::getWheelSpeeds,
				new PIDController(super.drivebase_cl.params.kP(), 0, 0),
				new PIDController(super.drivebase_cl.params.kP(), 0, 0),
				super::setDriveVoltage
			);
		}
		public FollowTrajectory(ClosedLoopDifferentialDrive db, Path json_path, boolean s) {	// accepts a path to a pathweaver json (deployed with robot program)
			super(db);
			this.stop = s;
			Trajectory temp;
			try {
				temp = TrajectoryUtil.fromPathweaverJson(json_path);
			} catch(Exception e) {
				System.err.println("FAILED TO READ TRAJECTORY: " + json_path.toString() + " -> " + e.getMessage());
				temp = new Trajectory(Arrays.asList(new Trajectory.State()));	// do-nothing trajectory as placeholder
			}
			this.trajectory = temp;
			this.controller = new RamseteCommand(
				this.trajectory,
				super.drivebase_cl::getCurrentPose,
				new RamseteController(Constants.ramsete_B, Constants.ramsete_Zeta),
				super.drivebase_cl.feedforward,
				super.drivebase_cl.kinematics,
				super.drivebase_cl::getWheelSpeeds,
				new PIDController(super.drivebase_cl.params.kP(), 0, 0),
				new PIDController(super.drivebase_cl.params.kP(), 0, 0),
				super::setDriveVoltage
			);
		}

		@Override public void initialize() {
			super.resetOdometry(this.trajectory.getInitialPose());
			this.controller.initialize();
			System.out.println("FollowTrajectory: Running...");
		}
		@Override public void execute() {
			this.controller.execute();
		}
		@Override public void end(boolean i) {
			this.controller.end(i);
			if(this.stop) {
				super.setDriveVoltage(0, 0);
			}
			System.out.println("FollowTrajectory: " + (i ? "Terminated." : "Completed."));
		}
		@Override public boolean isFinished() {
			return this.controller.isFinished();
		}

		public ParallelCommandGroup alongWithFromPercentage(Command c, double p) {	// starts the given command when the trajectory is the given percentage finished
			return new ParallelCommandGroup(
				this,
				new SequentialCommandGroup(
					new WaitCommand(this.trajectory.getTotalTimeSeconds() * p),
					c
				)
			);
		}
		public ParallelDeadlineGroup alongWithFromPercentageDeadline(Command c, double p) {	// ^^^ but the given command is cancelled when the trajectory ends
			return new ParallelDeadlineGroup(
				this,
				new SequentialCommandGroup(
					new WaitCommand(this.trajectory.getTotalTimeSeconds() * p),
					c
				)
			);
		}
		public ParallelCommandGroup alongWithUntilPercentage(Command c, double p) {	// starts the given command immediately and ends it when the given percentage is reached
			return new ParallelCommandGroup(
				this,
				new ParallelRaceGroup(
					c,
					new WaitCommand(this.trajectory.getTotalTimeSeconds() * p)
				)
			);
		}


	}

	public static class GoTo extends CLDriveCommand {

		private final Pose2d destination;
		private Trajectory generated = null;
		private RamseteCommand controller = null;
		private Thread builder = null;

		public GoTo(ClosedLoopDifferentialDrive db, Pose2d gt) {
			super(db);
			this.destination = gt;
		}

		@Override public void initialize() {
			this.generated = null;
			this.controller = null;
			this.builder = new Thread(()->{
				System.out.println("GoTo (Pose): Generating Trajectory...");
				this.generated = TrajectoryGenerator.generateTrajectory(
					List.of(super.drivebase_cl.getTotalPose(), this.destination),
					super.drivebase_cl.getTrajectoryConfig()
				);
				this.controller = new RamseteCommand(
					this.generated,
					super.drivebase_cl::getCurrentPose,
					new RamseteController(Constants.ramsete_B, Constants.ramsete_Zeta),
					super.drivebase_cl.feedforward,
					super.drivebase_cl.kinematics,
					super.drivebase_cl::getWheelSpeeds,
					new PIDController(super.drivebase_cl.params.kP(), 0, 0),
					new PIDController(super.drivebase_cl.params.kP(), 0, 0),
					super::setDriveVoltage
				);
				this.controller.initialize();
				System.out.println("GoTo (Pose): Trajectory Built. Running...");
			});
			this.builder.start();
		}
		@Override public void execute() {
			if(this.controller != null) {
				this.controller.execute();
			} else {
				super.setDriveVoltage(0, 0);
			}
		}
		@Override public void end(boolean i) {
			if(this.controller != null) {
				this.controller.end(i);
			}
			// super.setDriveVoltage(0, 0);
			System.out.println("GoTo (Pose): " + (i ? "Terminated." : "Completed."));
		}
		@Override public boolean isFinished() {
			return this.controller != null && this.controller.isFinished();
		}


	}

	// public Command followTrajectory(Trajectory t) {
	// 	return new SequentialCommandGroup(
	// 		new LambdaCommand(()->this.resetOdometry(t.getInitialPose())),	// may need to "transform" to initial pose location
	// 		new RamseteCommand(
	// 			t,
	// 			this::getPose,
	// 			new RamseteController(Constants.ramsete_B, Constants.ramsete_Zeta),
	// 			this.feedforward,
	// 			this.kinematics,
	// 			this::getWheelSpeeds,
	// 			new PIDController(this.params.kP(), 0, 0),
	// 			new PIDController(this.params.kP(), 0, 0),
	// 			this::setDriveVoltage,
	// 			this
	// 		)
	// 	);
	// }


}