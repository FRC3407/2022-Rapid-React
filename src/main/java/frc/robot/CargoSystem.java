package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.modules.common.drive.Motors;
import frc.robot.modules.common.drive.Motors.MotorSupplier;
import frc.robot.modules.vision.java.VisionServer;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.*;


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

		private final MotorControllerGroup motors;
		private final DigitalInput input, output;
		private int cargo_cnt = 0;
		private boolean
			now_input = false, now_output = false,
			last_input = false, last_output = false;

		public TransferSubsystem(MotorController... ms) { this(null, null, ms); }
		public TransferSubsystem(DigitalInput i, DigitalInput o, MotorController... ms) {
			this.motors = new MotorControllerGroup(ms);
			this.input = i;
			this.output = o;
			if(i != null && o != null) {
				this.register();
			}
		}
		public TransferSubsystem(int... ps) {
			this(null, null, Motors.pwm_victorspx, ps);
		}
		public TransferSubsystem(int i, int o, int... ps) {
			this(new DigitalInput(i), new DigitalInput(o), Motors.pwm_victorspx, ps);
			this.register();
		}
		public<M extends MotorController> TransferSubsystem(MotorSupplier<M> t, int... ps) {
			this(null, null, t, ps);
		}
		public<M extends MotorController> TransferSubsystem(int i, int o, MotorSupplier<M> t, int... ps) {
			this(new DigitalInput(i), new DigitalInput(o), t, ps);
			this.register();
		}
		public<M extends MotorController> TransferSubsystem(DigitalInput i, DigitalInput o, MotorSupplier<M> t, int... ps) {
			MotorController[] temp = new MotorController[ps.length];
			for(int k = 0; k < ps.length; k++) {
				temp[k] = t.create(ps[k]);
			}
			this.motors = new MotorControllerGroup(temp);
			this.input = i;
			this.output = o;
		}

		@Override public void periodic() {
			if(this.input != null && this.output != null) {	// should be safe because of check within constructors -> uncomment if issue
				this.last_input = this.now_input;
				this.last_output = this.now_output;
				this.now_input = this.input.get();
				this.now_output = this.output.get();
				if(this.isInputFallingEdge()) { this.cargo_cnt++; }
				if(this.isOutputFallingEdge()) { this.cargo_cnt--; }
			}
		}

		public void startAutomaticTransfer(double s) {
			this.setDefaultCommand(new AutomaticTransfer(this, s));
		}

		public void set(double s, TransferCommand c) {
			this.motors.set(s);
		}
		public void setVoltage(double v, TransferCommand c) {
			this.motors.setVoltage(v);
		}
		public void stop(TransferCommand c) {
			this.motors.stopMotor();
		}
		public boolean hasFeedback() {
			return this.input != null && this.output != null;
		}
		public boolean getCurrentInput() {
			if(this.input != null) {
				return this.input.get();
			}
			return false;
		}
		public boolean getCurrentOutput() {
			if(this.output != null) {
				return this.output.get();
			}
			return false;
		}
		public boolean isInputRisingEdge() {
			return !this.last_input && this.now_input;
		}
		public boolean isInputFallingEdge() {
			return this.last_input && !this.now_input;
		}
		public boolean isOutputRisingEdge() {
			return !this.last_output && this.now_output;
		}
		public boolean isOutputFallingEdge() {
			return this.last_output && !this.now_output;
		}
		public int getCargoCount() {
			return this.cargo_cnt;
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
			if(this.main != null) {
				this.main.set(ControlMode.PercentOutput, s);
			} else if(this.base_main != null) {
				this.base_main.set(s);
			}
		}
		public void setShooterVoltage(double v, ShooterCommand c) {
			if(this.main != null) {
				this.main.setVoltage(v);
			} else if(this.base_main != null) {
				this.base_main.setVoltage(v);
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
			if(this.main != null) {
				this.main.stopMotor();
			} else if(this.base_main != null) {
				this.base_main.stopMotor();
			}
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
			protected ShooterCommand(ShooterSubsystem s) {
				this.shooter_sys = s;
				super.addRequirements(s);
			}
			@Override public void end(boolean i) {
				this.shooter_sys.stopFeed(this);
				this.shooter_sys.stopShooter(this);
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
			if(this.transfer_sys.cargo_cnt < 2) {
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
			if(super.transfer_sys.getCurrentInput() && !super.transfer_sys.getCurrentOutput() && super.transfer_sys.getCargoCount() <= 2) {
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
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			super.shooter_sys.setShooterVoltage(this.shooter_voltage.getAsDouble(), this);
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
			super(s, ftrigger, fvolts, svolts);
			this.transfer_sys = t;
		}
		private ManagedShoot(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier ftrigger, DoubleSupplier fvolts, DoubleSupplier svolts) {
			super(s, ftrigger, fvolts, svolts);
			this.transfer_sys = t;
		}

		@Override public void initialize() {
			if(!this.transfer_sys.hasFeedback()) {
				System.out.println(getClass().getSimpleName() + ": No feedback sensors detected. Defaulting to BasicShoot.");
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public boolean isFinished() {
			return this.transfer_sys.hasFeedback() && this.transfer_sys.getCargoCount() <= 0;
		}


	}
	/**
	 * Spins the shooter and feed motor until the last limit switch is detected to have a falling edge (cargo just left).
	 */
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
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			this.transfer_command.execute();
			super.shooter_sys.setShooterVoltage(this.shoot_voltage, this);		// spin up shooter
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
			System.out.println(getClass().getSimpleName() + ": Running...");
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
			if(!super.shooter_sys.hasTalonShooter()) {
				System.out.println(getClass().getSimpleName() + ": TalonFX shooter not detected. Shooter disabled.");
				return;
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
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
			System.out.println(getClass().getSimpleName() + ": Running...");
			this.failed = false;
		}
		@Override public void execute() {
			if(!super.transfer_sys.hasFeedback() || super.transfer_sys.getCargoCount() > 0) {	// if sensors not detected or cargo in transfer system
				this.position = RapidReactVision.getHubPosition();
				if(this.position != null) {
					this.last_voltage = this.inches2voltage.convert(this.position.distance);	// update calculated voltage
				}
				super.shooter_sys.setShooterVoltage(this.last_voltage, this);		// set shooter speed
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
			System.out.println(getClass().getSimpleName() + ": Running...");
			this.failed = false;
		}
		@Override public void execute() {
			super.transfer_command.execute();
			this.position = RapidReactVision.getHubPosition();
			if(this.position != null) {
				this.last_voltage = this.inches2voltage.convert(this.position.distance);
			}
			super.shooter_sys.setShooterVoltage(this.last_voltage, this);
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