package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.*;

import frc.robot.team3407.drive.Motors;
import frc.robot.team3407.drive.Motors.MotorSupplier;
import frc.robot.vision.java.VisionServer;


/**
 * Contains all subsystems that manipulate cargo and commands to control them. All 'setter' methods for cargo subsystems require
 * a command of that subsystem type as a key, which makes sure that commandscheduler requirements are always respected.
 * ___________________________________________________________________
 * Table of Contents: >>>>>>>>>>>>>>>>>>>>>>>>>
 * {@link IntakeSubsystem} / {@link IntakeSubsystem.IntakeCommand}
 * {@link TransferSubsystem} / {@link TransferSubsystem.TransferCommand}
 * {@link ShooterSubsystem} / {@link ShooterSubsystem.ShooterCommand}
 * 
 * {@link BasicIntake} / {@link ManagedIntake} / 
 * {@link BasicTransfer} / {@link AutomaticTransfer}
 * {@link BasicShoot} / {@link ManagedShoot} / {@link ShootOne} / {@link ShootAll}
 * {@link BasicShootCL} / {@link ManagedShootCL} / {@link VisionShoot} /
 * {@link VisionShootOne} / {@link VisionShootAll}
 */
public final class CargoSystem {

	/**
	 * Represents an intake. Accessible by extending {@link IntakeCommand}.
	 */
	public static class IntakeSubsystem implements Subsystem {

		private final MotorController motor;

		public IntakeSubsystem(int p) { this(new PWMVictorSPX(p)); }
		public IntakeSubsystem(MotorController m) { this.motor = m; }
		public<M extends MotorController> IntakeSubsystem(int p, MotorSupplier<M> t) { this(t.create(p)); }
		public<M extends MotorController> IntakeSubsystem(int p, boolean invt, MotorSupplier<M> t) {
			this(t.create(p));
			this.motor.setInverted(invt);
		}

		public void set(double s, IntakeCommand c) {
			this.motor.set(s);
		}
		public void setVoltage(double v, IntakeCommand c) {
			this.motor.setVoltage(v);
		}
		public void stop(IntakeCommand c) {
			this.motor.stopMotor();
		}
		public double getLast() {
			return this.motor.get();
		}

		/**
		 * Extending this class gives access to 'setters' for an {@link IntakeSubsystem}
		 */
		public static abstract class IntakeCommand extends CommandBase {

			protected final IntakeSubsystem intake_sys;
			protected IntakeCommand(IntakeSubsystem i) {
				this.intake_sys = i;
				super.addRequirements(i);
			}
			@Override public void end(boolean i) { this.intake_sys.stop(this); }

			protected final void set(double s) {
				this.intake_sys.set(s, this);
			}
			protected final void setVoltage(double v) {
				this.intake_sys.setVoltage(v, this);
			}
			protected final void stop() {
				this.intake_sys.stop(this);
			}


		}


	}
	/**
	 * Represents the transfer motor(s). Accessible by extending {@link TransferCommand}.
	 */
	public static class TransferSubsystem implements Subsystem {

		public static abstract class IndexerSystem implements Subsystem {

			protected int count = 0;
			protected boolean
				front_state = false, back_state = false,
				front_state_last = false, back_state_last = false;

			public IndexerSystem() {
				if(hasFeedback()) {
					CommandScheduler.getInstance().registerSubsystem(this);
				}
			}

			@Override public void periodic() {
				this.front_state_last = this.front_state;
				this.back_state_last = this.back_state;
				this.front_state = this.getFrontRaw();
				this.back_state = this.getBackRaw();
				if(this.isFrontRisingEdge()) {
					this.count++;
				}
				if(this.isBackFallingEdge()) {
					this.count--;
				}
			}

			/** Override this method to return true when the sensors are valid */
			public boolean hasFeedback() { return false; }
			/** Override this method to return true if the implementation supports color detection */
			public boolean hasColorDetection() { return false; }
			/** Override this method so that indexing is functional */
			public boolean getFrontRaw() { return false; }
			/** Override this method so that indexing is functional */
			public boolean getBackRaw() { return false; }

			public final boolean isFrontRisingEdge() {
				return this.front_state && !this.front_state_last;
			}
			public final boolean isFrontFallingEdge() {
				return !this.front_state && this.front_state_last;
			}
			public final boolean isBackRisingEdge() {
				return this.back_state && !this.back_state_last;
			}
			public final boolean isBackFallingEdge() {
				return !this.back_state && this.back_state_last;
			}
			public final int getCount() {
				return this.count;
			}

			public Alliance getFrontCargoAlliance() {
				return Alliance.Invalid;
			}
			public Alliance getBackCargoAlliance() {
				return Alliance.Invalid;
			}
			public Alliance getLastBackDetected() {
				return Alliance.Invalid;
			}


		}
		public static class DigitalInputIndexer extends IndexerSystem {

			private final DigitalInput front, back;

			public DigitalInputIndexer(int f, int b) {
				this.front = new DigitalInput(f);
				this.back = new DigitalInput(b);
			}
			public DigitalInputIndexer(DigitalInput f, DigitalInput b) {
				this.front = f;
				this.back = b;
			}

			@Override public boolean hasFeedback() { return this.front != null && this.back != null; }
			@Override public boolean getFrontRaw() { return this.front != null ? this.front.get() : false; }
			@Override public boolean getBackRaw() { return this.back != null ? this.back.get() : false; }


		}
		public static class ColorSensorIndexer extends IndexerSystem {

			private static final Color ncolor = Constants.nothing_colormatch;
			private static final NetworkTable status_table = NetworkTableInstance.getDefault().getTable("ColorSensor Indexer");

			private final ColorSensorV3 front, back;
			private final ColorMatch matcher = new ColorMatch();
			private final Color red, blue;
			private ColorMatchResult f_color, b_color;
			private Color last_leaving = null;

			public ColorSensorIndexer(ColorSensorV3 f, ColorSensorV3 b, Color red, Color blue, Color n) {
				this.front = f;
				this.back = b;
				this.red = red;
				this.blue = blue;
				this.matcher.addColorMatch(red);
				this.matcher.addColorMatch(blue);
				this.matcher.addColorMatch(n);
				CommandScheduler.getInstance().registerSubsystem(this);
			}
			public ColorSensorIndexer(I2C.Port f, I2C.Port b, Color red, Color blue, Color n) {
				this(new ColorSensorV3(f), new ColorSensorV3(b), red, blue, n);
			}

			@Override public void periodic() {
				this.f_color = this.matcher.matchClosestColor(this.front != null ? this.front.getColor() : ncolor);
				this.b_color = this.matcher.matchClosestColor(this.back != null ? this.back.getColor() : ncolor);
				//System.out.println(this.f_color.color == this.red);
				super.periodic();
				if(this.getBackRaw()) {
					this.last_leaving = this.b_color.color;
				}
				status_table.getEntry("Last Back Alliance").setString(this.last_leaving == this.blue ? "Blue" : this.last_leaving == this.red ? "Red" : "Null");
			}
			@Override public boolean hasFeedback() {
				return this.front != null || this.back != null;
				//return true;
			}
			@Override public boolean hasColorDetection() {
				return this.hasFeedback();
			}
			@Override public boolean getFrontRaw() {
				//System.out.println("Front is null: " + this.f_color == null);
				return this.f_color != null && (this.f_color.color == this.red || this.f_color.color == this.blue);
			}
			@Override public boolean getBackRaw() {
				//System.out.println("Back is null: " + this.b_color == null);
				return this.b_color != null && (this.b_color.color == this.red || this.b_color.color == this.blue);
			}

			@Override public Alliance getFrontCargoAlliance() {
				return this.f_color.color == this.red ? Alliance.Red : this.f_color.color == this.blue ? Alliance.Blue : Alliance.Invalid;
			}
			@Override public Alliance getBackCargoAlliance() {
				return this.b_color.color == this.red ? Alliance.Red : this.b_color.color == this.blue ? Alliance.Blue : Alliance.Invalid;
			}
			@Override public Alliance getLastBackDetected() {
				return this.last_leaving == this.red ? Alliance.Red : this.last_leaving == this.blue ? Alliance.Blue : Alliance.Invalid; 
			}


		}

		private final MotorControllerGroup motors;
		private final IndexerSystem indexer;
		private int shooter_override_mult = 1;

		public TransferSubsystem(MotorController... ms) { this(null, null, ms); }
		public TransferSubsystem(DigitalInput i, DigitalInput o, MotorController... ms) {
			this.motors = new MotorControllerGroup(ms);
			this.indexer = new DigitalInputIndexer(i, o);
		}
		public TransferSubsystem(ColorSensorV3 f, ColorSensorV3 b, Color red, Color blue, Color n, MotorController... ms) {
			this.motors = new MotorControllerGroup(ms);
			this.indexer = new ColorSensorIndexer(f, b, red, blue, n);
		}
		public TransferSubsystem(int... ps) {
			this(null, null, Motors.pwm_victorspx, ps);
		}
		public TransferSubsystem(int i, int o, int... ps) {
			this(new DigitalInput(i), new DigitalInput(o), Motors.pwm_victorspx, ps);
		}
		public TransferSubsystem(I2C.Port f, I2C.Port b, Color red, Color blue, Color n, int... ps) {
			this(new ColorSensorV3(f), new ColorSensorV3(b), red, blue, n, Motors.pwm_victorspx, ps);
		}
		public<M extends MotorController> TransferSubsystem(MotorSupplier<M> t, int... ps) {
			this(null, null, t, ps);
		}
		public<M extends MotorController> TransferSubsystem(int i, int o, MotorSupplier<M> t, int... ps) {
			this(new DigitalInput(i), new DigitalInput(o), t, ps);
		}
		public<M extends MotorController> TransferSubsystem(I2C.Port f, I2C.Port b, Color red, Color blue, Color n, MotorSupplier<M> t, int... ps) {
			this(new ColorSensorV3(f), new ColorSensorV3(b), red, blue, n, t, ps);
		}
		public<M extends MotorController> TransferSubsystem(DigitalInput i, DigitalInput o, MotorSupplier<M> t, int... ps) {
			MotorController[] temp = new MotorController[ps.length];
			for(int k = 0; k < ps.length; k++) {
				temp[k] = t.create(ps[k]);
			}
			this.motors = new MotorControllerGroup(temp);
			this.indexer = new DigitalInputIndexer(i, o);
		}
		public<M extends MotorController> TransferSubsystem(ColorSensorV3 f, ColorSensorV3 b, Color red, Color blue, Color n, MotorSupplier<M> t, int... ps) {
			MotorController[] temp = new MotorController[ps.length];
			for(int k = 0; k < ps.length; k++) {
				temp[k] = t.create(ps[k]);
			}
			this.motors = new MotorControllerGroup(temp);
			this.indexer = new ColorSensorIndexer(f, b, red, blue, n);
		}

		public void startAutomaticTransfer(double s) {
			this.setDefaultCommand(new AutomaticTransfer(this, s));
		}

		public void set(double s, TransferCommand c) {
			this.motors.set(s * this.shooter_override_mult);
		}
		public void setVoltage(double v, TransferCommand c) {
			this.motors.setVoltage(v * this.shooter_override_mult);
		}
		public void stop(TransferCommand c) {
			this.motors.stopMotor();
		}
		public void setShooterOverride(boolean v, ShooterSubsystem.ShooterCommand c) {	// TRUE to enable override, FALSE to disable
			this.shooter_override_mult = v ? 0 : 1;
		}

		public boolean hasFeedback() {
			return this.indexer.hasFeedback();
		}
		public boolean hasColorDetection() {
			return this.indexer.hasColorDetection();
		}
		public boolean getCurrentInput() {
			return this.indexer.getFrontRaw();
		}
		public boolean getCurrentOutput() {
			return this.indexer.getBackRaw();
		}
		public boolean isInputRisingEdge() {
			return this.indexer.isFrontRisingEdge();
		}
		public boolean isInputFallingEdge() {
			return this.indexer.isFrontFallingEdge();
		}
		public boolean isOutputRisingEdge() {
			return this.indexer.isBackRisingEdge();
		}
		public boolean isOutputFallingEdge() {
			return this.indexer.isBackFallingEdge();
		}
		public int getCargoCount() {
			return this.indexer.getCount();
		}
		public Alliance getFrontCargoAlliance() {
			return this.indexer.getFrontCargoAlliance();
		}
		public Alliance getBackCargoAlliance() {
			return this.indexer.getBackCargoAlliance();
		}
		public Alliance getLastBackDetected() {
			return this.indexer.getLastBackDetected();
		}

		/**
		 * Provides access to 'setters' for a {@link TransferSubsystem}.
		 */
		public static class TransferCommand extends CommandBase {

			protected final TransferSubsystem transfer_sys;
			protected TransferCommand(TransferSubsystem t) {
				this.transfer_sys = t;
				super.addRequirements(t);
			}
			@Override public void end(boolean i) { this.transfer_sys.stop(this); }

			protected final void set(double s) {
				this.transfer_sys.set(s, this);
			}
			protected final void setVoltage(double v) {
				this.transfer_sys.setVoltage(v, this);
			}
			protected final void stop() {
				this.transfer_sys.stop(this);
			}


		}


	}
	/**
	 * Represents a shooter (and feed motor). Handles basic pwm motors along with the FalconFX, and can utilize 1 or 2 for the main shooter.
	 * Accessible by extening {@link ShooterCommand}. 
	 */
	public static class ShooterSubsystem implements Subsystem {

		private final WPI_TalonFX main, secondary;
		private final MotorController feed, base_main;
		private double vlimit = Double.MAX_VALUE;

		private void configureFalcons() {
			if(this.main != null) {
				this.main.configFactoryDefault();
				this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
				this.main.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
				if(this.secondary != null) {
					this.secondary.configFactoryDefault();
					this.secondary.follow(this.main);
					this.secondary.setInverted(InvertType.OpposeMaster);
				}
			}
		}

		public ShooterSubsystem(int m) {
			this(new WPI_TalonFX(m));
		}
		public<Mf extends MotorController> ShooterSubsystem(int m, MotorSupplier<Mf> t, int f) {
			this(new WPI_TalonFX(m), t.create(f));
		}
		public<Mf extends MotorController> ShooterSubsystem(int m, int s, MotorSupplier<Mf> t, int f) {
			this(new WPI_TalonFX(m), new WPI_TalonFX(s), t.create(f));
		}
		public<Ms extends MotorController> ShooterSubsystem(MotorSupplier<Ms> t, int m) {
			this(t.create(m));
		}
		public<Mf extends MotorController, Ms extends MotorController> ShooterSubsystem(MotorSupplier<Ms> ts, int m, MotorSupplier<Mf> tf, int f) {
			this(ts.create(m), tf.create(f));
		}
		public<Mf extends MotorController, Ms extends MotorController> ShooterSubsystem(MotorSupplier<Ms> ts, int m, int s, MotorSupplier<Mf> tf, int f) {
			this(ts.create(m), ts.create(s), tf.create(f));
		}
		public ShooterSubsystem(WPI_TalonFX m) { this(m, null, null); }
		public ShooterSubsystem(WPI_TalonFX m, MotorController f) { this(m, null, f); }
		public ShooterSubsystem(WPI_TalonFX m, WPI_TalonFX s, MotorController f) {
			this.main = m;
			this.secondary = s;
			this.feed = f;
			if(!(m == null || s == null)) {
				this.base_main = new MotorControllerGroup(m, s);
			} else {
				this.base_main = m;
			}
			this.configureFalcons();

		}
		public ShooterSubsystem(MotorController m) { this(m, null, null); }
		public ShooterSubsystem(MotorController m, MotorController f) { this(m, null, f); }
		public ShooterSubsystem(MotorController m, MotorController s, MotorController f) {
			this.main = null;
			this.secondary = null;
			this.feed = f;
			if(!(m == null || s == null)) {
				this.base_main = new MotorControllerGroup(m, s);
			} else {
				this.base_main = m;
			}
		}

		/** In VOLTAGE not PERCENT OUTPUT */
		public void rateLimit(double voltage_rate) {
			this.vlimit = voltage_rate;
			if(this.main != null) {
				this.main.configOpenloopRamp(voltage_rate / 12);
				this.main.configClosedloopRamp(voltage_rate / 12);
			}
			if(this.secondary != null) {
				this.secondary.configOpenloopRamp(voltage_rate / 12);
				this.secondary.configClosedloopRamp(voltage_rate / 12);
			}
		}

		public void setFeed(double s, ShooterCommand c) {
			if(this.feed != null) {
				this.feed.set(s);
			}
		}
		public void setFeedVoltage(double v, ShooterCommand c) {
			if(this.feed != null) {
				this.feed.setVoltage(v);
			}
		}
		public void stopFeed(ShooterCommand c) {
			if(this.feed != null) {
				this.feed.stopMotor();
			}
		}
		public void setShooter(double s, ShooterCommand c) {
			if(this.base_main != null) {
				this.base_main.set(s);
			} else if(this.main != null) {
				this.main.set(ControlMode.PercentOutput, s);
			}
		}
		public void setShooterVoltage(double v, ShooterCommand c) {
			if(this.base_main != null) {
				this.base_main.setVoltage(v);
			} else if(this.main != null) {
				this.main.setVoltage(v);
			}
		}
		public void setShooterRawVelocity(double vr, ShooterCommand c) {
			if(this.main != null) {
				this.main.set(ControlMode.Velocity, vr);
			}
		}
		public void setShooterVelocity(double rpm, ShooterCommand c) {
			this.setShooterRawVelocity(rpm / 600 * Constants.falcon_units_per_revolution, c);
		}
		public void stopShooter(ShooterCommand c) {
			if(this.base_main != null) {
				this.base_main.stopMotor();
			} else if(this.main != null) {
				this.main.stopMotor();
			}
		}
		public double getLastShooterSpeed() {
			if(this.base_main != null) {
				return this.base_main.get();
			} else if(this.main != null) {
				return this.main.get();
			}
			return 0.0;
		}
		public double getShooterRawPosition() {
			if(this.main != null) {
				return this.main.getSelectedSensorPosition();
			}
			return 0.0;
		}
		public double getShooterRawVelocity() {
			if(this.main != null) {
				return this.main.getSelectedSensorVelocity();
			}
			return 0.0;
		}
		public double getShooterVelocity() {	// in rpm
			return this.getShooterRawVelocity() * 600 / Constants.falcon_units_per_revolution;
		}
		public boolean hasTalonShooter() {
			return this.main != null;
		}
		public boolean hasFeedMotor() {
			return this.feed != null;
		}

		/**
		 * Provides access to 'setters' for a {@link ShooterSubsystem}
		 */
		public static abstract class ShooterCommand extends CommandBase {

			protected final ShooterSubsystem shooter_sys;
			protected SlewRateLimiter rate_limit;

			protected ShooterCommand(ShooterSubsystem s) {
				this.shooter_sys = s;
				super.addRequirements(s);
			}
			@Override public void end(boolean i) {
				this.shooter_sys.stopFeed(this);
				this.shooter_sys.stopShooter(this);
			}

			/** Make sure this method is called on initialization */
			protected void initRateLimit() {
				this.rate_limit = new SlewRateLimiter(this.shooter_sys.vlimit, 0);
			}
			/** Applies rate limit, returns the limited value */
			protected double setShooterVoltage(double v) {
				double limited = this.rate_limit.calculate(v);
				this.shooter_sys.setShooterVoltage(limited, this);
				return limited;
			}

			// add shooter access methods


		}


	}








	public final IntakeSubsystem intake;
	public final TransferSubsystem transfer;
	public final ShooterSubsystem shooter;

	public CargoSystem(IntakeSubsystem i, TransferSubsystem t, ShooterSubsystem s) {
		this.intake = i;
		this.transfer = t;
		this.shooter = s;
	}

	public void startAutomaticTransfer(double s) { this.transfer.startAutomaticTransfer(s); }

	public DeployIntake deployIntake() { return new DeployIntake(this.intake); }
	public BasicIntake basicIntake(double volts) { return new BasicIntake(this.intake, volts); }
	public BasicIntake basicIntake(DoubleSupplier volts) { return new BasicIntake(this.intake, volts); }
	public ManagedIntake managedIntake(double volts) { return new ManagedIntake(this.intake, this.transfer, volts); }
	public ManagedIntake managedIntake(DoubleSupplier volts) { return new ManagedIntake(this.intake, this.transfer, volts); }
	public BasicTransfer basicTransfer(double volts) { return new BasicTransfer(this.transfer, volts); }
	public BasicTransfer basicTransfer(DoubleSupplier volts) { return new BasicTransfer(this.transfer, volts); }

	public BasicShoot basicShoot(BooleanSupplier ftrigger, double fvolts, double svolts) { return new BasicShoot(this.shooter, ftrigger, fvolts, svolts); }
	public BasicShoot basicShoot(BooleanSupplier ftrigger, DoubleSupplier fvolts, DoubleSupplier svolts) { return new BasicShoot(this.shooter, ftrigger, fvolts, svolts); }
	public ManagedShoot managedShoot(BooleanSupplier ftrigger, double fvolts, double svolts) { return new ManagedShoot(this.shooter, this.transfer, ftrigger, fvolts, svolts); }
	public ManagedShoot managedShoot(BooleanSupplier ftrigger, DoubleSupplier fvolts, DoubleSupplier svolts) { return new ManagedShoot(this.shooter, this.transfer, ftrigger, fvolts, svolts); }
	public ShootOne shootSingle(double fvolts, double svolts, double tvolts) { return new ShootOne(this.shooter, this.transfer, fvolts, svolts, tvolts); }
	public ShootAll shootAll(double fvolts, double svolts, double tvolts) { return new ShootAll(this.shooter, this.transfer, fvolts, svolts, tvolts); }
	public BasicShootCL basicShootVelocity(BooleanSupplier ftrigger, double fvolts, double sv) { return new BasicShootCL(this.shooter, ftrigger, fvolts, sv); }
	public BasicShootCL basicShootVelocity(BooleanSupplier ftrigger, DoubleSupplier fvolts, DoubleSupplier sv) { return new BasicShootCL(this.shooter, ftrigger, fvolts, sv); }
	public ManagedShootCL managedShootVelocity(BooleanSupplier ftrigger, double fvolts, double sv) { return new ManagedShootCL(this.shooter, this.transfer, ftrigger, fvolts, sv); }
	public ManagedShootCL managedShootVelocity(BooleanSupplier ftrigger, DoubleSupplier fvolts, DoubleSupplier sv) { return new ManagedShootCL(this.shooter, this.transfer, ftrigger, fvolts, sv); }
	public VisionShoot visionShoot(BooleanSupplier ftrigger, double fvolts, VisionServer.Conversion in2vlt) { return new VisionShoot(this.shooter, this.transfer, ftrigger, fvolts, in2vlt); }
	public VisionShootOne visionShootSingle(double fvolts, double tvolts, VisionServer.Conversion in2vlt) { return new VisionShootOne(this.shooter, this.transfer, fvolts, tvolts, in2vlt); }
	public VisionShootAll visionShootAll(double fvolts, double tvolts, VisionServer.Conversion in2vlt) { return new VisionShootAll(this.shooter, this.transfer, fvolts, tvolts, in2vlt); }








// Intake Commands >>


	/**
	 * Spins the intake slowly for a short duration (under the assumption that pinwheel-type hardware is used to push the intake out)
	 */
	public static class DeployIntake extends IntakeSubsystem.IntakeCommand {

		public static final double
			voltage = Constants.intake_deploy_voltage,
			duration = Constants.intake_deploy_time;
		private final  Timer time = new Timer();

		private DeployIntake(IntakeSubsystem i) { super(i); }

		@Override public void initialize() {
			System.out.println(getClass().getSimpleName() + ": Running...");
			this.time.reset();
			this.time.start();
		}
		@Override public void execute() {
			super.setVoltage(voltage);
		}
		@Override public void end(boolean i) {
			this.time.stop();
			super.stop();
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			return this.time.hasElapsed(duration);
		}


	}
	/**
	 * Basic control over the intake with either constant voltage or variable if constructed with a {@link DoubleSupplier}.
	 */
	public static class BasicIntake extends IntakeSubsystem.IntakeCommand {

		private final DoubleSupplier voltage;

		private BasicIntake(IntakeSubsystem i, double volts) { this(i, ()->volts); }
		private BasicIntake(IntakeSubsystem i, DoubleSupplier volts) {
			super(i);
			this.voltage = volts;
		}

		@Override public void initialize() {
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			super.setVoltage(this.voltage.getAsDouble());
		}
		@Override public void end(boolean i) {
			super.stop();
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() { return false; }


	}
	/**
	 * Basic intake control but only when the transfer system has space for more cargo. If the transfer system does not contain limit switches,
	 * this command resembles {@link BasicIntake}. 
	 */
	public static class ManagedIntake extends BasicIntake {

		private final TransferSubsystem transfer_sys;

		private ManagedIntake(IntakeSubsystem i, TransferSubsystem t, double volts) { this(i, t, ()->volts); }
		private ManagedIntake(IntakeSubsystem i, TransferSubsystem t, DoubleSupplier volts) {
			super(i, volts);
			this.transfer_sys = t;
		}

		@Override public void initialize() {
			if(!this.transfer_sys.hasFeedback()) {
				System.out.println(getClass().getSimpleName() + ": No feedback sensors detected. Defaulting to BasicIntake.");
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			if(this.transfer_sys.getCargoCount() < 2) {
				super.execute();
			} else {
				super.setVoltage(0);
			}
		}
		@Override public boolean isFinished() { return false; }


	}






// Transfer Commands >>


	/**
	 * Basic control over the transfer motor(s).
	 */
	public static class BasicTransfer extends TransferSubsystem.TransferCommand {

		private final DoubleSupplier voltage;

		private BasicTransfer(TransferSubsystem t, double volts) { this(t, ()->volts); }
		private BasicTransfer(TransferSubsystem t, DoubleSupplier volts) {
			super(t);
			this.voltage = volts;
		}

		@Override public void initialize() {
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			super.setVoltage(this.voltage.getAsDouble());
		}
		@Override public void end(boolean i) {
			super.stop();
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() { return false; }


	}
	/**
	 * This should be used as the default command for a {@link TransferSubsystem}. Automatically moves the transfer belts when the front limit
	 * switch is triggered and stops when it is released (thus automatically cycling cargo). Does nothing if the maximum capacity has been reached
	 * or no limit switches are detected. 
	 */
	private static class AutomaticTransfer extends BasicTransfer {

		private AutomaticTransfer(TransferSubsystem t, double volts) { super(t, volts); }
		private AutomaticTransfer(TransferSubsystem t, DoubleSupplier volts) { super(t, volts); }

		@Override public void initialize() {
			if(!super.transfer_sys.hasFeedback()) {
				System.out.println(getClass().getSimpleName() + ": No feedback sensors detected. Automatic transfer disabled.");
			} else {
				System.out.println(getClass().getSimpleName() + ": Running...");
			}
		}
		@Override public void execute() {
			//System.out.println("F: " + super.transfer_sys.getCurrentInput() + "\tB: " + super.transfer_sys.getCurrentOutput());
			if(super.transfer_sys.getCurrentInput() && !super.transfer_sys.getCurrentOutput()/* && !super.transfer_sys.getCurrentOutput() && super.transfer_sys.getCargoCount() <= 2*/) {
				//System.out.println("running transfer");
				super.execute();
			} else {
				super.set(0);
			}
		}
		@Override public boolean isFinished() {
			//return !super.transfer_sys.hasFeedback();
			return false; 	// "default commands should not end"
		}


	}





// Basic Shoot Commands >>


	/**
	 * Provides basic shooter control for a {@link ShooterSubsystem}.
	 */
	public static class BasicShoot extends ShooterSubsystem.ShooterCommand {

		protected final DoubleSupplier shooter_voltage, feed_voltage;
		protected final BooleanSupplier feed_actuate;

		private BasicShoot(ShooterSubsystem s, BooleanSupplier ftrigger, double fvolts, double svolts) { this(s, ftrigger, ()->fvolts, ()->svolts); }
		private BasicShoot(ShooterSubsystem s, BooleanSupplier ftrigger, DoubleSupplier fvolts, DoubleSupplier svolts) {
			super(s);
			this.shooter_voltage = svolts;
			this.feed_voltage = fvolts;
			this.feed_actuate = ftrigger;
		}

		@Override public void initialize() {
			super.initRateLimit();
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			super.setShooterVoltage(this.shooter_voltage.getAsDouble());
			if(this.feed_actuate.getAsBoolean()) {
				super.shooter_sys.setFeedVoltage(this.feed_voltage.getAsDouble(), this);
			} else {
				super.shooter_sys.setFeed(0, this);
			}
		}
		@Override public void end(boolean i) {
			this.shooter_sys.stopShooter(this);
			this.shooter_sys.stopFeed(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() { return false; }


	}
	/**
	 * Provides basic shooter control but only when there is cargo available in the transfer system. If no limit switches are present, acts
	 * the same as {@link BasicShoot}. 
	 */
	public static class ManagedShoot extends BasicShoot {	// basicshoot except that this only runs when there is no feedback system or when there is and cargo is present in the transfer system

		private final TransferSubsystem transfer_sys;

		private ManagedShoot(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier ftrigger, double fvolts, double svolts) {
			super(s, ftrigger, ()->fvolts, 
				t.hasColorDetection() ? 
					()->{
						if(DriverStation.getAlliance() == t.getLastBackDetected()) {
							return svolts;
						} else {
							return svolts / 2.0;
						}
					} : ()->svolts
			);
			this.transfer_sys = t;
		}
		private ManagedShoot(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier ftrigger, DoubleSupplier fvolts, DoubleSupplier svolts) {
			super(s, ftrigger, fvolts, 
				t.hasColorDetection() ? 
				()->{
					if(DriverStation.getAlliance() == t.getLastBackDetected()) {
						return svolts.getAsDouble();
					} else {
						return svolts.getAsDouble() / 2.0;
					}
				} : svolts
			);
			this.transfer_sys = t;
		}

		@Override protected double setShooterVoltage(double v) {
			double l = super.setShooterVoltage(v);
			this.transfer_sys.setShooterOverride(l != v, this);
			return l;
		}

		@Override public void initialize() {
			if(!this.transfer_sys.hasFeedback()) {
				System.out.println(getClass().getSimpleName() + ": No feedback sensors detected. Defaulting to BasicShoot.");
			}
			super.initialize();
		}
		@Override public void execute() {
			// maybe change the super implementation to somehow use the overloaded 'setShooterVoltage' instead of this reimplemntation
			this.setShooterVoltage(this.shooter_voltage.getAsDouble());
			if(this.feed_actuate.getAsBoolean()) {
				super.shooter_sys.setFeedVoltage(this.feed_voltage.getAsDouble(), this);
			} else {
				super.shooter_sys.setFeed(0, this);
			}
		}
		// @Override public boolean isFinished() {
		// 	return this.transfer_sys.hasFeedback() && this.transfer_sys.getCargoCount() <= 0;
		// }


	}
	/**
	 * Spins the shooter and feed motor until the last limit switch is detected to have a falling edge (cargo just left).
	 */
// UPDATE THIS TO EXTEND MANAGEDSHOOT
	public static class ShootOne extends ShooterSubsystem.ShooterCommand {	// shoots a single cargo with the given speed - transfer sensors required

		protected final TransferSubsystem transfer_sys;
		private final BasicTransfer transfer_command;
		private final double feed_voltage, shoot_voltage;

		private ShootOne(ShooterSubsystem s, TransferSubsystem t, double fvolts, double svolts, double tvolts) {
			super(s);
			this.transfer_sys = t;
			this.transfer_command = new BasicTransfer(t, tvolts);	// command for setting transfer belt to 'tvolts' during the duration of this command
			this.feed_voltage = fvolts;
			this.shoot_voltage = svolts;

			super.addRequirements(t);
		}

		@Override public void initialize() {
			this.transfer_command.initialize();
			super.initRateLimit();
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			this.transfer_command.execute();
			super.setShooterVoltage(this.shoot_voltage);		// spin up shooter
			super.shooter_sys.setFeedVoltage(this.transfer_sys.getCurrentOutput() ? this.feed_voltage : 0, this);	// set feed if output limit triggered
		}
		@Override public void end(boolean i) {
			this.transfer_command.end(i);
			super.shooter_sys.stopFeed(this);
			super.shooter_sys.stopShooter(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			return !this.transfer_sys.hasFeedback() ||	// no feedback devices detected, or...
				(this.transfer_sys.hasFeedback() && this.transfer_sys.isOutputFallingEdge());	// cargo leaving transfer system
		}


	}
	/**
	 * Spins the shooter and feed motor until all cargo has been ejected (as detected by the transfer system limit switches). 
	 */
	public static class ShootAll extends ShootOne {		// shoots all cargo in the transfer system - transfer sensors required

		private ShootAll(ShooterSubsystem s, TransferSubsystem t, double fvolts, double svolts, double tvolts) { super(s, t, fvolts, svolts, tvolts); }

		@Override public boolean isFinished() {
			return !super.transfer_sys.hasFeedback() ||	// no feedback devices detected, or...
				(super.transfer_sys.hasFeedback() && super.transfer_sys.getCargoCount() <= 0);	// no cargo left to shoot
		}


	}





// Velcity-CL Shoot Commands >>


	/**
	 * Shooter control with velocity. Requires the shooter motor(s) to be TalonFX's. The PID loop for setting the velocity is contained within
	 * the phoenix api. Additionally, the feed motor is not allowed to be spun until the target velocity is reached. If the shooter motor(s) are
	 * not TalonFX's, then this command does nothing. 
	 */
	public static class BasicShootCL extends BasicShoot {	// meant for manual control but with closed-loop velocity

		private BasicShootCL(ShooterSubsystem s, BooleanSupplier ftrigger, double fvolts, double s_rpm) { super(s, ftrigger, fvolts, s_rpm); }
		private BasicShootCL(ShooterSubsystem s, BooleanSupplier ftrigger, DoubleSupplier fvolts, DoubleSupplier s_rpm) { super(s, ftrigger, fvolts, s_rpm); }

		@Override public void initialize() {
			if(!super.shooter_sys.hasTalonShooter()) {
				System.out.println(getClass().getSimpleName() + ": TalonFX not detected. Shooter disabled.");
				return;
			}
			super.initialize();
		}
		@Override public void execute() {
			if(super.shooter_sys.hasTalonShooter()) {	// if no talon, do nothing, else set to given speed
				super.shooter_sys.setShooterVelocity(super.shooter_voltage.getAsDouble(), this);	// "shooter voltage" actually hold the rpm because of the overloaded constructor
				if(super.feed_actuate.getAsBoolean() && super.shooter_sys.getShooterVelocity() >= super.shooter_voltage.getAsDouble()) {
					super.shooter_sys.setFeedVoltage(super.feed_voltage.getAsDouble(), this);
				} else {
					super.shooter_sys.setFeed(0, this);
				}
			}
		}


	}
	/**
	 * Shooter control with velocity, but only when cargo is present in the transfer system (as detected by limit switches). Requires the shooter motor(s) to be TalonFX's. The PID loop for setting the velocity is contained within
	 * the phoenix api. Additionally, the feed motor is not allowed to be spun until the target velocity is reached. If the shooter motor(s) are
	 * not TalonFX's, then this command does nothing. 
	 */
// UPDATE THIS TO EXTEND MANAGEDSHOOT (for clrsense ramping handler)
	public static class ManagedShootCL extends BasicShootCL {

		private final TransferSubsystem transfer_sys;

		private ManagedShootCL(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier ftrigger, double fvolts, double s_rpm) { this(s, t, ftrigger, ()->fvolts, ()->s_rpm); }
		private ManagedShootCL(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier ftrigger, DoubleSupplier fvolts, DoubleSupplier s_rpm) {
			super(s, ftrigger, fvolts, s_rpm);
			this.transfer_sys = t;
		}

		@Override public void initialize() {
			if(!this.transfer_sys.hasFeedback()) {
				System.out.println(getClass().getSimpleName() + ": No feedback sensors detected. Defaulting to BasicShootCL.");
			}
			super.initialize();
		}
		// @Override public void execute() {
		// 	if(super.shooter_sys.hasTalonShooter()) {	// if no talon, do nothing, else set to given speed
		// 		super.shooter_sys.setShooterVelocity(super.sspeed.getAsDouble() / 10, this);	// divide by ten to get units/100ms
		// 		if(super.feed.getAsBoolean() && super.shooter_sys.getShooterRawVelocity() >= super.sspeed.getAsDouble() / 10) {
		// 			super.shooter_sys.setFeed(super.fspeed.getAsDouble(), this);
		// 		} else {
		// 			super.shooter_sys.setFeed(0, this);
		// 		}
		// 	}
		// }
		// @Override public void end(boolean i) {
		// 	this.shooter_sys.stopShooter(this);
		// 	this.shooter_sys.stopFeed(this);
		// 	System.out.println(i ? "ManagedShootCL: Terminated." : "ManagedShootCL: Completed.");
		// }
		@Override public boolean isFinished() {
			return this.transfer_sys.hasFeedback() && this.transfer_sys.getCargoCount() <= 0;
		}


	}
	// public static class ShootOneCL extends ShootOne {

	// }
	// public static class ShootAllCL extends ShootAll {

	// }





// Vision Shoot Commands >>


	/**
	 * Shooting speed is determined based on how far away the hub is (detected by vision). A user-defined p-control is used to set the 
	 * voltage at any given distance. Once the shooter is spun up the operator can trigger the feed to shoot. 
	 */
	public static class VisionShoot extends ManagedShoot {

		private final VisionServer.Conversion inches2voltage;
		private VisionServer.TargetData position = null;
		private double last_voltage = 0.0;
		private boolean failed = false;

		private VisionShoot(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier ftrigger, double fvolts, VisionServer.Conversion in2vlt) {
			super(s, t, ftrigger, fvolts, 0);
			this.inches2voltage = in2vlt;
		}

		@Override public void initialize() {
			Constants.vision_hub.run();
			if(!RapidReactVision.verifyHubPipelineActive()) {
				System.out.println(getClass().getSimpleName() + ": Failed to set UpperHub pipeline.");
				this.failed = true;
				return;
			}
			super.initRateLimit();
			System.out.println(getClass().getSimpleName() + ": Running...");
			this.failed = false;
		}
		@Override public void execute() {
			if(!super.transfer_sys.hasFeedback() || super.transfer_sys.getCargoCount() > 0) {	// if sensors not detected or cargo in transfer system
				this.position = RapidReactVision.getHubPosition();
				if(this.position != null) {
					this.last_voltage = this.inches2voltage.convert(this.position.distance);	// update calculated voltage
					if(super.transfer_sys.hasColorDetection() && DriverStation.getAlliance() != super.transfer_sys.getLastBackDetected()) {
						this.last_voltage /= 2;
					}
				}
				super.setShooterVoltage(this.last_voltage);		// set shooter speed
				super.shooter_sys.setFeedVoltage(super.feed_actuate.getAsBoolean() ? super.feed_voltage.getAsDouble() : 0, this);	// set feed if booleansupplier allows
			} else {	// otherwise stop motors
				super.shooter_sys.setShooter(0, this);
				super.shooter_sys.setFeed(0, this);
			}
		}
		@Override public boolean isFinished() {
			if(this.position != null) {
				return super.isFinished();
			}
			return this.failed;
		}


	}
	/**
	 * Automatically determines the speed to shoot at based on the distance from the hub (from user-supplied p-control). Feeds unil
	 * exactly one cargo has been ejected as determined by the transfer system limit switches. 
	 */
	public static class VisionShootOne extends ShootOne {

		private final VisionServer.Conversion inches2voltage;
		private VisionServer.TargetData position = null;
		private double last_voltage = 0.0;
		private boolean failed = false;

		private VisionShootOne(ShooterSubsystem s, TransferSubsystem t, double fvolts, double tvolts, VisionServer.Conversion in2vlt) {
			super(s, t, fvolts, 0, tvolts);
			this.inches2voltage = in2vlt;

			super.addRequirements(t);
		}

		@Override public void initialize() {
			Constants.vision_hub.run();
			if(!RapidReactVision.verifyHubPipelineActive()) {
				System.out.println(getClass().getSimpleName() + ": Failed to set UpperHub pipeline.");
				this.failed = true;
				return;
			}
			super.transfer_command.initialize();
			super.initRateLimit();
			System.out.println(getClass().getSimpleName() + ": Running...");
			this.failed = false;
		}
		@Override public void execute() {
			super.transfer_command.execute();
			this.position = RapidReactVision.getHubPosition();
			if(this.position != null) {
				this.last_voltage = this.inches2voltage.convert(this.position.distance);
			}
			super.setShooterVoltage(this.last_voltage);
			super.shooter_sys.setFeedVoltage(super.transfer_sys.getCurrentOutput() ? super.feed_voltage : 0, this);
		}
		@Override public boolean isFinished() {
			if(this.position != null) {
				return super.isFinished();
			}
			return this.failed;
		}


	}
	/**
	 * Automatically determines the speed to shoot at based on the distance from the hub (from user-supplied p-control). Feeds unil
	 * all cargo has been ejected as determined by the transfer system limit switches. 
	 */
	public static class VisionShootAll extends VisionShootOne {

		private VisionShootAll(ShooterSubsystem s, TransferSubsystem t, double fvolts, double tvolts, VisionServer.Conversion in2vlt) { super(s, t, fvolts, tvolts, in2vlt); }

		@Override public boolean isFinished() {
			if(super.position != null) {
				return !super.transfer_sys.hasFeedback() ||	// no feedback devices detected, or...
					(super.transfer_sys.hasFeedback() && super.transfer_sys.getCargoCount() <= 0);	// no cargo left to shoot
			}
			return super.failed;
		}


	}
	// public static class VisionShootOneCL extends ShootOne {

	// }
	// public static class VisionShootAllCL extends ShootAll {

	// }


}