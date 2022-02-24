package frc.robot;

import frc.robot.modules.common.LambdaCommand;
import frc.robot.modules.common.EventTriggers.EnabledTrigger;
import frc.robot.modules.common.Input.DigitalSupplier;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;


/**
 * Consists of the intake, transfer, feed, and shooter (and limit switches). 
 * Combines all three in order to provide "smart" features like counting and indexing cargo
 */
public class CargoSystem extends SubsystemBase {

	private static class Intake extends SubsystemBase {
        protected final PWMVictorSPX 
            motor;
        public Intake(int p) {
            this.motor = new PWMVictorSPX(p);
        }

        protected void set(double s) { this.motor.set(s); }
        protected void setVoltage(double v) { this.motor.setVoltage(v); }
        protected void stop() { this.motor.stopMotor(); }
	}
	private static class Transfer extends SubsystemBase {
        protected final MotorControllerGroup 
            primary;
        protected final DigitalInput
			lim_entering,   // this would be a limit switch just after the intake and before "ball position 1"
            //t_mid,      // this would be a limit switch inbetween the ball positions (may not need this one)
            lim_exiting;  // this would be a limit switch just before a ball enters the shooter

        public Transfer(int di, int ds, int... ports) {
            this.lim_entering = new DigitalInput(di);
            this.lim_exiting = new DigitalInput(ds);
            PWMVictorSPX[] motors = new PWMVictorSPX[ports.length];
            for(int i = 0; i < ports.length; i++) {
                motors[i] = new PWMVictorSPX(ports[i]);
            }
            this.primary = new MotorControllerGroup(motors);
        }

        protected void set(double s) { this.primary.set(s); }
        protected void setVoltage(double s) { this.primary.set(s); }
        protected void stop() { this.primary.stopMotor(); }
        protected boolean isIntakeTriggered() { return this.lim_entering.get(); }
        protected boolean isShooterTriggered() { return this.lim_exiting.get(); }

		protected void enableAutomaticTransfer(CargoStates s) {
			this.setDefaultCommand(new AutomaticTransfer(this, s));
		}
	}
	private static class Shooter extends SubsystemBase {		// add a way to invert
		protected final WPI_TalonFX
            main, secondary;
		protected final PWMVictorSPX 
            feed;

		public Shooter(int f, int m) {
			this.feed = new PWMVictorSPX(f);
            this.main = new WPI_TalonFX(m);
			this.secondary = null;
            this.main.configFactoryDefault();
            this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		}
		public Shooter(int f, int m, int s) {
			this.feed = new PWMVictorSPX(f);
            this.main = new WPI_TalonFX(m);
            this.secondary = new WPI_TalonFX(s);
            this.main.configFactoryDefault();
            this.secondary.configFactoryDefault();
            this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
            this.secondary.follow(this.main);
            this.secondary.setInverted(InvertType.FollowMaster);
		}

        protected void setFeed(double s) { this.feed.set(s); }
		protected void setFeedVoltage(double s) { this.feed.setVoltage(s); }
		protected void stopFeed() { this.feed.stopMotor(); }

		protected void setShooterPercentage(double p) {
			this.main.set(ControlMode.PercentOutput, p);
		}
		protected void setShooterVelocityRaw(double vr) {		// input velocity is currently in raw units per 100 ms
			this.main.set(ControlMode.Velocity, vr);
		}
		protected void setShooterVelocity(double v) {			// input velocity should be in meters per second
			this.main.set(
				ControlMode.Velocity, 
				(v * Constants.falcon_encoder_units_per_revolution) / (10 * Math.PI * Constants.shooter_wheel_diameter_meters)
			);
		}
		protected void stopShooter() {
			this.main.stopMotor();
		}
		public double getShooterRawPosition() {	// in raw encoder units (see Constants.falcon_encoder_units_per_revolution)
			return this.main.getSelectedSensorPosition();
		}
		public double getShooterRawVelocity() {	// in raw units per 100 ms
			return this.main.getSelectedSensorVelocity();
		}
		public double getShooterVelocity() {	// in meters per second
			return this.getShooterRawVelocity() * 
				(Constants.shooter_wheel_diameter_meters * Math.PI / Constants.falcon_encoder_units_per_revolution) * 10;
		}
	}
	private static class CargoStates {
		private final DigitalSupplier
			entering_src, exiting_src;
		private int count = 0;
		private boolean
			entering = false,
			exiting = false,
			last_entering = false,
			last_exiting = false;

		protected CargoStates(DigitalSupplier en, DigitalSupplier ex) {
			this.entering_src = en;
			this.exiting_src = ex;
		}

		public synchronized int update() {
			this.last_entering = this.entering;
			this.last_exiting = this.exiting;
			this.entering = this.entering_src.get();
			this.exiting = this.exiting_src.get();
			if(this.last_entering && !this.entering) {	// high to low on entering lim switch -> a ball has entered
				this.count++;
			}
			if(this.last_exiting && !this.exiting) {	// high to low on exiting lim switch -> a ball is leaving 
				this.count--;
			}
			return this.count;
		}

		public int cargoCount() { return this.count; }
		public boolean isCargoEntering() { return this.last_entering; }
		public boolean isCargoExiting() { return this.last_exiting; }
		public boolean spaceAvailable() { return this.count < 2; }
		public boolean canShoot() { return this.count > 0; }
	}

	private final CargoStates state;
	private final Intake intake;
	private final Transfer transfer;
	private final Shooter shooter;

	protected CargoSystem(Intake i, Transfer t, Shooter s) {
		this.intake = i;
		this.transfer = t;
		this.shooter = s;
		this.state = new CargoStates(()->t.lim_entering.get(), ()->t.lim_exiting.get());

		CommandScheduler.getInstance().registerSubsystem(this);
	}	
	public CargoSystem(
		int port_intake, int port_feed,
		int canid_main, int canid_secondary,
		int dio_lim_front, int dio_lim_back,
		int... transfer_ports
	) { this(
			new Intake(port_intake),
			new Transfer(dio_lim_front, dio_lim_back, transfer_ports),
			new Shooter(port_feed, canid_main, canid_secondary)
		);
	}
	public CargoSystem(
		int port_intake, int port_feed,
		int canid_main,
		int dio_lim_front, int dio_lim_back,
		int... transfer_ports
	) { this(
			new Intake(port_intake),
			new Transfer(dio_lim_front, dio_lim_back, transfer_ports),
			new Shooter(port_feed, canid_main)
		);
	}

	@Override public void periodic() {
		this.state.update();
	}

	public ManualOverride manualOverride(DigitalSupplier i, DigitalSupplier t, DigitalSupplier s) {
		return new ManualOverride(this, i, t, s);
	}
	public IntakeCargo intakeCargo(double s) {
		return new IntakeCargo(this.intake, this.state, s);
	}
	public ShootAll shootAllCargo(double v) {
		return new ShootAll(this.shooter, this.state, v);
	}



	public static class ManualOverride extends CommandBase {
		private final CargoSystem cargo_sys;
		private final DigitalSupplier
			intake, transfer;
		public ManualOverride(CargoSystem cs, DigitalSupplier i, DigitalSupplier t, DigitalSupplier s) {
			this.cargo_sys = cs;
			this.intake = i;
			this.transfer = t;
			super.addRequirements(cs.intake, cs.transfer);

			new Trigger(()->{return s.get();}).whenActive(new ShootAll(cs.shooter, cs.state, Constants.shooter_default_speed), false);
		}
		@Override public void execute() {
			if(this.intake.get()) {
				this.cargo_sys.intake.set(Constants.intake_speed);
			} else {
				this.cargo_sys.intake.set(0);
			}
			if(this.transfer.get()) {
				this.cargo_sys.transfer.set(Constants.transfer_speed);
			} else {
				this.cargo_sys.transfer.set(0);
			}
		}
		@Override public boolean isFinished() { return false; }
	}
	public static class IntakeCargo extends CommandBase {
		private final Intake intake_sys;
		private final CargoStates state;
		private final double speed;
		private boolean last_lim;
		public IntakeCargo(Intake i, CargoStates st, double s) {
			this.intake_sys = i;
			this.state = st;
			this.speed = s;
			this.last_lim = this.state.isCargoEntering();
			super.addRequirements(i);
		}
		@Override public void execute() { 
			if(this.state.spaceAvailable()) {
				this.intake_sys.set(this.speed); 
			}
		}
		@Override public boolean isFinished() {
			boolean temp = this.last_lim;
			this.last_lim = this.state.isCargoEntering();
			return !this.state.spaceAvailable() || (!temp && this.last_lim);	// << a ball triggered the limit switch
		}
	}
	private static class AutomaticTransfer extends CommandBase {
		private final Transfer transfer;
		private final CargoStates state;
		public AutomaticTransfer(Transfer t, CargoStates s) {
			this.transfer = t;
			this.state = s;
			super.addRequirements(t);
		}
		@Override public void execute() {
			if(this.state.isCargoEntering() && !this.state.isCargoExiting() && this.state.spaceAvailable()) {
				this.transfer.set(Constants.transfer_speed);
			}
		}
		@Override public void end(boolean i) { this.transfer.stop(); }
		@Override public boolean isFinished() { return false; }
	}
	public static class ShootAll extends CommandBase {
		private final Shooter shooter;
		private final CargoStates state;
		private final double velocity;	// <- probably in meters per second
		public ShootAll(Shooter sh, CargoStates st, double v) {
			this.shooter = sh;
			this.state = st;
			this.velocity = v;
			super.addRequirements(sh);
		}
		@Override public void initialize() {}
		@Override public void execute() {
			this.shooter.setShooterVelocity(this.velocity);
			if(Math.abs(this.velocity - this.shooter.getShooterVelocity()) < Constants.shooter_speed_tollerance && this.state.isCargoExiting()) {
				this.shooter.setFeed(Constants.feed_speed);
			}
		}
		@Override public boolean isFinished() {
			return this.state.cargoCount() == 0;
		}
	}
	public static class ShootVision extends CommandBase {

	}


}