package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.modules.common.drive.Motors;
import frc.robot.modules.common.drive.Motors.MotorSupplier;
import frc.robot.modules.vision.java.VisionServer;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.*;


public final class CargoSystemV2 {

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
		public double getLast(IntakeCommand c) {
			return this.motor.get();
		}

		public static abstract class IntakeCommand extends CommandBase {

			protected final IntakeSubsystem intake_sys;
			protected IntakeCommand(IntakeSubsystem i) {
				this.intake_sys = i;
				super.addRequirements(i);
			}
			@Override public void end(boolean i) { this.intake_sys.stop(this); }


		}


	}
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

		public static class TransferCommand extends CommandBase {

			protected final TransferSubsystem transfer_sys;
			protected TransferCommand(TransferSubsystem t) {
				this.transfer_sys = t;
				super.addRequirements(t);
			}
			@Override public void end(boolean i) { this.transfer_sys.stop(this); }


		}


	}
	public static class ShooterSubsystem implements Subsystem {

		private final WPI_TalonFX main, secondary;
		private final MotorController feed, base_main;

		private void configureFalcons() {
			if(this.main != null) {
				this.main.configFactoryDefault();
				this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
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
		public void setShooterVelocity(double vr, ShooterCommand c) {
			if(this.main != null) {
				this.main.set(ControlMode.Velocity, vr);
			}
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
		public double getShooterVelocity() {
			if(this.main != null) {
				return this.getShooterRawVelocity() * 
					(Constants.shooter_wheel_diameter_meters * Math.PI / Constants.falcon_encoder_units_per_revolution) * 10;
			}
			return 0.0;
		}
		public boolean hasTalonShooter() {
			return this.main != null;
		}
		public boolean hasFeedMotor() {
			return this.feed != null;
		}

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


		}


	}





	private final IntakeSubsystem intake;
	private final TransferSubsystem transfer;
	private final ShooterSubsystem shooter;

	public CargoSystemV2(IntakeSubsystem i, TransferSubsystem t, ShooterSubsystem s) {
		this.intake = i;
		this.transfer = t;
		this.shooter = s;
	}

	public void startAutomaticTransfer(double s) { this.transfer.startAutomaticTransfer(s); }

	public BasicIntake basicIntake(double s) { return new BasicIntake(this.intake, s); }
	public BasicIntake basicIntake(DoubleSupplier s) { return new BasicIntake(this.intake, s); }
	public ManagedIntake managedIntake(double s) { return new ManagedIntake(this.intake, this.transfer, s); }
	public ManagedIntake managedIntake(DoubleSupplier s) { return new ManagedIntake(this.intake, this.transfer, s); }
	public BasicTransfer basicTransfer(double s) { return new BasicTransfer(this.transfer, s); }
	public BasicTransfer basicTransfer(DoubleSupplier s) { return new BasicTransfer(this.transfer, s); }

	public BasicShoot basicShoot(BooleanSupplier f, double fs, double ss) { return new BasicShoot(this.shooter, f, fs, ss); }
	public BasicShoot basicShoot(BooleanSupplier f, DoubleSupplier fs, DoubleSupplier ss) { return new BasicShoot(this.shooter, f, fs, ss); }
	public ManagedShoot managedShoot(BooleanSupplier f, double fs, double ss) { return new ManagedShoot(this.shooter, this.transfer, f, fs, ss); }
	public ManagedShoot managedShoot(BooleanSupplier f, DoubleSupplier fs, DoubleSupplier ss) { return new ManagedShoot(this.shooter, this.transfer, f, fs, ss); }
	public ShootOne shootSingle(double fs, double ss, double ts) { return new ShootOne(this.shooter, this.transfer, fs, ss, ts); }
	public ShootAll shootAll(double fs, double ss, double ts) { return new ShootAll(this.shooter, this.transfer, fs, ss, ts); }
	public BasicShootCL basicShootVelocity(BooleanSupplier f, double fs, double sv) { return new BasicShootCL(this.shooter, f, fs, sv); }
	public BasicShootCL basicShootVelocity(BooleanSupplier f, DoubleSupplier fs, DoubleSupplier sv) { return new BasicShootCL(this.shooter, f, fs, sv); }
	public ManagedShootCL managedShootVelocity(BooleanSupplier f, double fs, double sv) { return new ManagedShootCL(this.shooter, this.transfer, f, fs, sv); }
	public ManagedShootCL managedShootVelocity(BooleanSupplier f, DoubleSupplier fs, DoubleSupplier sv) { return new ManagedShootCL(this.shooter, this.transfer, f, fs, sv); }
	public VisionShoot visionShoot(BooleanSupplier f, double fs, VisionServer.Conversion in2vlt) { return new VisionShoot(this.shooter, this.transfer, f, fs, in2vlt); }
	public VisionShootOne visionShootSingle(double fs, double ts, VisionServer.Conversion in2vlt) { return new VisionShootOne(this.shooter, this.transfer, fs, ts, in2vlt); }
	public VisionShootAll visionShootAll(double fs, double ts, VisionServer.Conversion in2vlt) { return new VisionShootAll(this.shooter, this.transfer, fs, ts, in2vlt); }





	public static class BasicIntake extends IntakeSubsystem.IntakeCommand {

		private final DoubleSupplier speed;

		private BasicIntake(IntakeSubsystem i, double s) { this(i, ()->s); }
		private BasicIntake(IntakeSubsystem i, DoubleSupplier s) {
			super(i);
			this.speed = s;
		}

		@Override public void initialize() {
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			super.intake_sys.set(this.speed.getAsDouble(), this);
		}
		@Override public void end(boolean i) {
			super.intake_sys.stop(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() { return false; }


	}
	public static class ManagedIntake extends BasicIntake {

		private final TransferSubsystem transfer_sys;

		private ManagedIntake(IntakeSubsystem i, TransferSubsystem t, double s) { this(i, t, ()->s); }
		private ManagedIntake(IntakeSubsystem i, TransferSubsystem t, DoubleSupplier s) {
			super(i, s);
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
				super.intake_sys.set(0, this);
			}
		}
		@Override public boolean isFinished() { return false; }


	}
	public static class BasicTransfer extends TransferSubsystem.TransferCommand {

		private final DoubleSupplier speed;

		private BasicTransfer(TransferSubsystem t, double s) { this(t, ()->s); }
		private BasicTransfer(TransferSubsystem t, DoubleSupplier s) {
			super(t);
			this.speed = s;
		}

		@Override public void initialize() {
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			super.transfer_sys.set(this.speed.getAsDouble(), this);
		}
		@Override public void end(boolean i) {
			super.transfer_sys.stop(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() { return false; }


	}
	private static class AutomaticTransfer extends BasicTransfer {

		private AutomaticTransfer(TransferSubsystem t, double s) { super(t, s); }
		private AutomaticTransfer(TransferSubsystem t, DoubleSupplier s) { super(t, s); }

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
				super.transfer_sys.set(0, this);
			}
		}
		@Override public void end(boolean i) {
			super.transfer_sys.stop(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			//return !super.transfer_sys.hasFeedback();
			return false; 	// "default commands should not end"
		}


	}



	public static class BasicShoot extends ShooterSubsystem.ShooterCommand {

		protected final DoubleSupplier sspeed, fspeed;
		protected final BooleanSupplier feed;

		private BasicShoot(ShooterSubsystem s, BooleanSupplier f, double fs, double ss) { this(s, f, ()->fs, ()->ss); }
		private BasicShoot(ShooterSubsystem s, BooleanSupplier f, DoubleSupplier fs, DoubleSupplier ss) {
			super(s);
			this.sspeed = ss;
			this.fspeed = fs;
			this.feed = f;
		}

		@Override public void initialize() {
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			super.shooter_sys.setShooter(this.sspeed.getAsDouble(), this);
			if(this.feed.getAsBoolean()) {
				super.shooter_sys.setFeed(this.fspeed.getAsDouble(), this);
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
	public static class ManagedShoot extends BasicShoot {	// basicshoot except that this only runs when there is no feedback system or when there is and cargo is present in the transfer system

		private final TransferSubsystem transfer_sys;

		private ManagedShoot(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier f, double fs, double ss) {
			super(s, f, fs, ss);
			this.transfer_sys = t;
		}
		private ManagedShoot(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier f, DoubleSupplier fs, DoubleSupplier ss) {
			super(s, f, fs, ss);
			this.transfer_sys = t;
		}

		@Override public void initialize() {
			if(!this.transfer_sys.hasFeedback()) {
				System.out.println(getClass().getSimpleName() + ": No feedback sensors detected. Defaulting to BasicShoot.");
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		// @Override public void execute() {
		// 	if(!this.transfer_sys.hasFeedback() || this.transfer_sys.getCargoCount() > 0) {
		// 		super.execute();
		// 	} else {
		// 		super.shooter_sys.setShooter(0, this);
		// 		super.shooter_sys.setFeed(0, this);
		// 	}
		// }
		@Override public void end(boolean i) {
			this.shooter_sys.stopShooter(this);
			this.shooter_sys.stopFeed(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			return this.transfer_sys.hasFeedback() && this.transfer_sys.getCargoCount() <= 0;
		}


	}
	public static class ShootOne extends ShooterSubsystem.ShooterCommand {	// shoots a single cargo with the given speed - transfer sensors required

		protected final TransferSubsystem transfer_sys;
		private final BasicTransfer transfer_command;
		private final double feed, shoot;

		private ShootOne(ShooterSubsystem s, TransferSubsystem t, double fs, double ss, double ts) {
			super(s);
			this.transfer_sys = t;
			this.transfer_command = new BasicTransfer(t, ts);	// command for setting transfer belt to 'ts' during the duration of this command
			this.feed = fs;
			this.shoot = ss;
		}

		@Override public void initialize() {
			this.transfer_command.schedule(false);
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			super.shooter_sys.setShooter(this.shoot, this);		// spin up shooter
			super.shooter_sys.setFeed(this.transfer_sys.getCurrentOutput() ? this.feed : 0, this);	// set feed if output limit triggered
		}
		@Override public void end(boolean i) {
			this.transfer_command.cancel();
			super.shooter_sys.stopFeed(this);
			super.shooter_sys.stopShooter(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			return !this.transfer_sys.hasFeedback() ||	// no feedback devices detected, or...
				(this.transfer_sys.hasFeedback() && this.transfer_sys.isOutputFallingEdge());	// cargo leaving transfer system
		}


	}
	public static class ShootAll extends ShootOne {		// shoots all cargo in the transfer system - transfer sensors required

		private ShootAll(ShooterSubsystem s, TransferSubsystem t, double fs, double ss, double ts) { super(s, t, fs, ss, ts); }

		@Override public void initialize() {
			super.transfer_command.schedule(false);
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void end(boolean i) {
			super.transfer_command.cancel();
			super.shooter_sys.stopFeed(this);
			super.shooter_sys.stopShooter(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			return !super.transfer_sys.hasFeedback() ||	// no feedback devices detected, or...
				(super.transfer_sys.hasFeedback() && super.transfer_sys.getCargoCount() <= 0);	// no cargo left to shoot
		}


	}
	/*
		Speed units for shooter are in encoder units per second (see Constants for resolution).
		Does nothing if shooter is not a talon (no encoder)
	*/
	public static class BasicShootCL extends BasicShoot {	// meant for manual control but with closed-loop velocity

		private BasicShootCL(ShooterSubsystem s, BooleanSupplier f, double fs, double sv) { super(s, f, fs, sv); }
		private BasicShootCL(ShooterSubsystem s, BooleanSupplier f, DoubleSupplier fs, DoubleSupplier sv) { super(s, f, fs, sv); }

		@Override public void initialize() {
			if(!super.shooter_sys.hasTalonShooter()) {
				System.out.println(getClass().getSimpleName() + ": TalonFX shooter not detected. Shooter disabled.");
				return;
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			if(super.shooter_sys.hasTalonShooter()) {	// if no talon, do nothing, else set to given speed
				super.shooter_sys.setShooterVelocity(super.sspeed.getAsDouble() / 10, this);	// divide by ten to get units/100ms
				if(super.feed.getAsBoolean() && super.shooter_sys.getShooterRawVelocity() >= super.sspeed.getAsDouble() / 10) {
					super.shooter_sys.setFeed(super.fspeed.getAsDouble(), this);
				} else {
					super.shooter_sys.setFeed(0, this);
				}
			}
		}
		// @Override public void end(boolean i) {
		// 	this.shooter_sys.stopShooter(this);
		// 	this.shooter_sys.stopFeed(this);
		// 	System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		// }


	}
	/*
		Speed units for shooter are in encoder units per second (see Constants for resolution).
		Does nothing if shooter is not a talon (no encoder)
	*/
	public static class ManagedShootCL extends BasicShootCL {

		private final TransferSubsystem transfer_sys;

		private ManagedShootCL(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier f, double fs, double sv) { this(s, t, f, ()->fs, ()->sv); }
		private ManagedShootCL(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier f, DoubleSupplier fs, DoubleSupplier sv) {
			super(s, f, fs, sv);
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
	public static class VisionShoot extends ManagedShoot {

		private final VisionServer.Conversion inches2voltage;
		private VisionServer.TargetData position = null;
		private double last_speed = 0.0;
		private boolean failed = false;

		private VisionShoot(ShooterSubsystem s, TransferSubsystem t, BooleanSupplier f, double fs, VisionServer.Conversion in2vlt) {
			super(s, t, f, fs, 0);
			this.inches2voltage = in2vlt;
		}

		@Override public void initialize() {
			VisionServer.Get().applyCameraPreset(Constants.cam_hub_pipeline);
			VisionServer.Get().setCamera(Constants.hub_cam_name);
			if(!RapidReactVision.verifyHubPipelineActive()) {
				System.out.println(getClass().getSimpleName() + ": Failed to set UpperHub pipeline.");
				this.failed = true;
				return;
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			if(!super.transfer_sys.hasFeedback() || super.transfer_sys.getCargoCount() > 0) {	// if sensors not detected or cargo in transfer system
				this.position = RapidReactVision.getHubPosition();
				if(this.position != null) {
					this.last_speed = this.inches2voltage.convert(this.position.distance);	// update calculated voltage
				}
				super.shooter_sys.setShooterVoltage(this.last_speed, this);		// set shooter speed
				super.shooter_sys.setFeed(super.feed.getAsBoolean() ? super.fspeed.getAsDouble() : 0, this);	// set feed if booleansupplier allows
			} else {	// otherwise stop motors
				super.shooter_sys.setShooter(0, this);
				super.shooter_sys.setFeed(0, this);
			}
		}
		@Override public void end(boolean i) {
			super.shooter_sys.stopShooter(this);
			super.shooter_sys.stopFeed(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			if(this.position != null) {
				return super.isFinished();
			}
			return this.failed;
		}


	}
	public static class VisionShootOne extends ShootOne {

		private final VisionServer.Conversion inches2voltage;
		private VisionServer.TargetData position = null;
		private double last_speed = 0.0;
		private boolean failed = false;

		private VisionShootOne(ShooterSubsystem s, TransferSubsystem t, double fs, double ts, VisionServer.Conversion in2vlt) {
			super(s, t, fs, 0, ts);
			this.inches2voltage = in2vlt;
		}

		@Override public void initialize() {
			VisionServer.Get().applyCameraPreset(Constants.cam_hub_pipeline);
			VisionServer.Get().setCamera(Constants.hub_cam_name);
			if(!RapidReactVision.verifyHubPipelineActive()) {
				System.out.println(getClass().getSimpleName() + ": Failed to set UpperHub pipeline.");
				this.failed = true;
				return;
			}
			super.transfer_command.schedule(false);
			System.out.println(getClass().getSimpleName() + ": Running...");
		}
		@Override public void execute() {
			this.position = RapidReactVision.getHubPosition();
			if(this.position != null) {
				this.last_speed = this.inches2voltage.convert(this.position.distance);
			}
			super.shooter_sys.setShooterVoltage(this.last_speed, this);
			super.shooter_sys.setFeed(super.transfer_sys.getCurrentOutput() ? super.feed : 0, this);
		}
		@Override public void end(boolean i) {
			super.transfer_command.cancel();
			super.shooter_sys.stopFeed(this);
			super.shooter_sys.stopShooter(this);
			System.out.println(getClass().getSimpleName() + (i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			if(this.position != null) {
				return super.isFinished();
			}
			return this.failed;
		}


	}
	public static class VisionShootAll extends VisionShootOne {

		private VisionShootAll(ShooterSubsystem s, TransferSubsystem t, double fs, double ts, VisionServer.Conversion in2vlt) { super(s, t, fs, ts, in2vlt); }

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