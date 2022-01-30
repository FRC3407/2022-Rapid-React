package frc.robot.outdated;
// package frc.robot;

// //import frc.robot.Containers;

// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.button.Button;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
// import edu.wpi.first.wpilibj.DigitalInput;

// // This class has been changed to represent/contain all robot hardware, ex. drivetrain, arms, sensors, etc...
// public class Robot {
	
// 	public static class DriveBase extends SubsystemBase {	// extend DifferentialDrive?

// 		private final MotorController left, right;
// 		private final DifferentialDrive drive;
		
// 		private final Constants.DriveModes mode;
// 		private final Constants.DBS settings;

// 		public DriveBase(Constants.DB2 map, Constants.DBS settings) {
// 			this.left = new PWMVictorSPX(map.left);
// 			this.right = new PWMVictorSPX(map.right);

// 			this.left.setInverted(map.invert.left);
// 			this.right.setInverted(map.invert.right);

// 			this.drive = new DifferentialDrive(this.left, this.right);
			
// 			if(settings.mode_options != null) {
// 				this.mode = new Constants.DriveModes(settings.default_mode, settings.mode_options);
// 			} else {
// 				this.mode = new Constants.DriveModes(settings.default_mode, new Constants.DriveMode[]{settings.default_mode});
// 			}
// 			this.settings = settings;

// 			this.setDefaultCommand(new Idling(this));
// 		}
// 		public DriveBase(Constants.DB4 map, Constants.DBS settings) {
// 			this.left = new MotorControllerGroup(new PWMVictorSPX(map.front_left), new PWMVictorSPX(map.back_left));
// 			this.right = new MotorControllerGroup(new PWMVictorSPX(map.front_right), new PWMVictorSPX(map.back_right));

// 			this.left.setInverted(map.invert.left);
// 			this.right.setInverted(map.invert.right);

// 			this.drive = new DifferentialDrive(this.left, this.right);

// 			if(settings.mode_options != null) {
// 				this.mode = new Constants.DriveModes(settings.default_mode, settings.mode_options);
// 			} else {
// 				this.mode = new Constants.DriveModes(settings.default_mode, new Constants.DriveMode[]{settings.default_mode});
// 			}
// 			this.settings = settings;

// 			this.setDefaultCommand(new Idling(this));
// 		}

// 		public void setLeft(double speed) {
// 			this.left.set(speed);
// 		}
// 		public void setRight(double speed) {
// 			this.right.set(speed);
// 		}
// 		public double getLeft() {
// 			return this.left.get();
// 		}
// 		public double getRight() {
// 			return this.right.get();
// 		}

// 		public void sureStop() {
// 			this.drive.stopMotor();
// 		}
// 		public void zero() {
// 			this.left.set(0.0);
// 			this.right.set(0.0);
// 		}

// 		public Constants.DriveMode getMode() {
// 			return this.mode.get();
// 		}
// 		public void setMode(Constants.DriveMode m) {
// 			this.mode.set(m);
// 		}
// 		public void incrementMode() {
// 			this.mode.increment();
// 		}
// 		public void incrementMode(int v) {
// 			this.mode.increment(v);
// 		}
// 		public void decrementMode() {
// 			this.mode.decrement();
// 		}
// 		public void decrementMode(int v) {
// 			this.mode.decrement(v);
// 		}

// 		public void tankDrive(double lspeed, double rspeed) {
// 			this.drive.tankDrive(lspeed, rspeed, this.settings.default_squaring);
// 		}
// 		public void arcadeDrive(double speed, double rotation) {
// 			this.drive.arcadeDrive(speed, rotation, this.settings.default_squaring);
// 		}
// 		public void curvatureDrive(double speed, double rotation, boolean qturn) {
// 			this.drive.curvatureDrive(speed, rotation, qturn);
// 		}
// 		public void raceDrive(double forward, double backward, double rotation) {
// 			this.drive.arcadeDrive(forward-backward, rotation, this.settings.default_squaring);
// 		}
// 		public void triggerDrive(double lspeed, double rspeed) {
// 			this.drive.tankDrive(lspeed, rspeed, this.settings.default_squaring);
// 		}
// 		public void triggerDrive(double lspeed, double rspeed, boolean forward) {
// 			byte mult = forward ? (byte)1 : (byte)-1;
// 			this.drive.tankDrive(lspeed*mult, rspeed*mult, this.settings.default_squaring);
// 		}
// 		public void modeDrive(Input controls) {
// 			switch(this.mode.get()) {
// 				case TANK:
// 					this.tankDrive(controls.getAnalogLY(), controls.getAnalogRY());
// 					break;
// 				case ARCADE:
// 					this.arcadeDrive(controls.getAnalogRX(), controls.getAnalogRY());
// 					break;
// 				case RACE:
// 					System.out.println("Input systems currently do not support 'RACE DRIVE'");
// 					//this.raceDrive(0, 0, 0);
// 					break;
// 				case TRIGGER:
// 					System.out.println("Input systems currently do not support 'TRIGGER DRIVE'");
// 					//this.triggerDrive(0, 0);
// 					break;
// 				// case CURVATURE:
// 				// 	this.curvatureDrive(0, 0, false);
// 				// 	break;
// 				default:
// 					System.out.println("Drivemode detection error - motors have not been updated");
// 			}
// 		}

// 		private static class Idling extends CommandBase {
// 			private final DriveBase drivebase;
// 			public Idling(DriveBase db) {
// 				this.drivebase = db;
// 				addRequirements(this.drivebase);
// 			}
// 			@Override public void initialize() {System.out.println("Drive motors idling...");}
// 			@Override public void execute() {drivebase.zero();}
// 			@Override public void end(boolean i) {System.out.println("Drive motors stopped idling.");}
// 			@Override public boolean isFinished() {return false;}
// 		}
// 		public static class Decelerate extends CommandBase {
// 			private final DriveBase drivebase;
// 			private double s_left, s_right;
// 			private final double gradient;
// 			public Decelerate(DriveBase db, Constants.Deceleration d) {		// << Add a constant for the gradient - this should be 0.96 to 0.99, but 0.98 should be best
// 				this.drivebase = db;
// 				this.gradient = d.value;
// 				addRequirements(this.drivebase);
// 			}
// 			@Override public void initialize() {}
// 			@Override public void execute() {
// 				s_left *= this.gradient;
// 				s_right *= this.gradient;
// 				this.drivebase.tankDrive(s_left, s_right);
// 			}
// 			@Override public void end(boolean i) {this.drivebase.zero();}
// 			@Override public boolean isFinished() {return (s_left < 0.1) && (s_right < 0.1);}
// 		}
// 	}

// 	public static class Dumper extends SubsystemBase {	// MAKE CONSTANTS: dumping speed, dumping time(or get another limit switch)

// 		private final PWMVictorSPX bucket;
// 		private final DigitalInput limit;
// 		private final Dump c_dump;
// 		private final Reset c_reset;

// 		public Dumper(int b, int l) {
// 			this.bucket = new PWMVictorSPX(b);
// 			this.limit = new DigitalInput(l);

// 			this.c_dump = new Dump(this);
// 			this.c_reset = new Reset(this);
// 		}

// 		public void dump() {
// 			CommandScheduler.getInstance().schedule(this.c_dump.withTimeout(2));	// <-- get actual precise time or add another limit switch
// 		}
// 		public void reset() {
// 			CommandScheduler.getInstance().schedule(this.c_reset);
// 		}
// 		public void bindDump(Button bind) {
// 			bind.whenPressed(this.c_dump.withTimeout(2)); 	// <-- get actual precise time or add another limit switch
// 		}
// 		public void bindReset(Button bind) {
// 			bind.whenPressed(this.c_reset);
// 		}

// 		private boolean getLimit() {
// 			return this.limit.get();
// 		}
// 		private void setBucket(double speed) {
// 			this.bucket.set(speed);
// 		}

// 		private static class Dump extends CommandBase {
// 			private final Dumper dumper;
// 			public Dump(Dumper d) {
// 				this.dumper = d;
// 				addRequirements(this.dumper);
// 			}
// 			@Override
// 			public void execute() {this.dumper.setBucket(0.3);}
// 			@Override
// 			public void end(boolean i) {this.dumper.setBucket(0);}
// 			@Override
// 			public boolean isFinished() {return false;}
// 		}
// 		private static class Reset extends CommandBase {
// 			private final Dumper dumper;
// 			public Reset(Dumper d) {
// 				this.dumper = d;
// 				addRequirements(this.dumper);
// 			}
// 			@Override
// 			public void execute() {if(!this.dumper.getLimit()) {this.dumper.setBucket(-0.3);}}
// 			@Override
// 			public void end(boolean i) {this.dumper.setBucket(0);}
// 			@Override
// 			public boolean isFinished() {return this.dumper.getLimit();}
// 		}

// 	}
	

// }



// /**
//  * The VM is configured to automatically run this class, and to call the functions corresponding to
//  * each mode, as described in the TimedRobot documentation. If you change the name of this class or
//  * the package after creating this project, you must also update the build.gradle file in the
//  * project.
//  */
// // public class Robot extends TimedRobot {
// // 	private Command m_autonomousCommand;

// // 	private RobotContainer m_robotContainer;

// // 	private Input main_in = new Input(1);
// // 	/**
// // 	 * This function is run when the robot is first started up and should be used for any
// // 	 * initialization code.
// // 	 */
// // 	@Override
// // 	public void robotInit() {
// // 		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
// // 		// autonomous chooser on the dashboard.
// // 		m_robotContainer = new RobotContainer();
// // 	}

// // 	/**
// // 	 * This function is called every robot packet, no matter the mode. Use this for items like
// // 	 * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
// // 	 *
// // 	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
// // 	 * SmartDashboard integrated updating.
// // 	 */
// // 	@Override
// // 	public void robotPeriodic() {
// // 		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
// // 		// commands, running already-scheduled commands, removing finished or interrupted commands,
// // 		// and running subsystem periodic() methods.  This must be called from the robot's periodic
// // 		// block in order for anything in the Command-based framework to work.
// // 		CommandScheduler.getInstance().run();
// // 	}

// // 	/** This function is called once each time the robot enters Disabled mode. */
// // 	@Override
// // 	public void disabledInit() {}

// // 	@Override
// // 	public void disabledPeriodic() {}

// // 	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
// // 	@Override
// // 	public void autonomousInit() {
// // 		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

// // 		// schedule the autonomous command (example)
// // 		if (m_autonomousCommand != null) {
// // 		m_autonomousCommand.schedule();
// // 		}
// // 	}

// // 	/** This function is called periodically during autonomous. */
// // 	@Override
// // 	public void autonomousPeriodic() {}

// // 	@Override
// // 	public void teleopInit() {
// // 		// This makes sure that the autonomous stops running when
// // 		// teleop starts running. If you want the autonomous to
// // 		// continue until interrupted by another command, remove
// // 		// this line or comment it out.
// // 		if (m_autonomousCommand != null) {
// // 		m_autonomousCommand.cancel();
// // 		}
// // 	}

// // 	/** This function is called periodically during operator control. */
// // 	@Override
// // 	public void teleopPeriodic() {}

// // 	@Override
// // 	public void testInit() {
// // 		// Cancels all running commands at the start of test mode.
// // 		CommandScheduler.getInstance().cancelAll();
// // 	}

// // 	/** This function is called periodically during test mode. */
// // 	@Override
// // 	public void testPeriodic() {}
// // }