package frc.robot;

import frc.robot.modules.common.LambdaCommand;
import frc.robot.modules.common.EventTriggers.EnabledTrigger;
import frc.robot.modules.common.Input.DigitalSupplier;
import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.vision.java.VisionServer;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.*;
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
		private Transfer(DigitalInput en, DigitalInput ex, int... ports) {
			this.lim_entering = en;
            this.lim_exiting = ex;
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
		private Shooter(WPI_TalonFX m, WPI_TalonFX s, int f) {
			this.feed = new PWMVictorSPX(f);
			this.main = m;
			this.secondary = s;
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

	public ManualOverride manualOverride(DigitalSupplier i, DigitalSupplier t, DigitalSupplier f, DigitalSupplier s) {
		return new ManualOverride(this, i, t, f, s);
	}
	public IntakeCargo intakeCargo(double s) {
		return new IntakeCargo(this.intake, this.state, s);
	}
	public ShootOne shootSingleCargo(double v) {
		return new ShootOne(this.shooter, this.state, v);
	}
	public ShootAll shootAllCargo(double v) {
		return new ShootAll(this.shooter, this.state, v);
	}
	
	public OLIntake olIntake(DigitalSupplier finish) {
		return new OLIntake(this.intake, finish);
	}
	public OLTransfer olTransfer(DigitalSupplier finish) {
		return new OLTransfer(this.transfer, finish);
	}
	public OLShoot olShoot(DigitalSupplier feed, DigitalSupplier finish, double v) {
		return new OLShoot(this.shooter, feed, finish, v);
	}



	public static class ManualOverride extends CommandBase {	// manually control all cargosystem motors
		private final CargoSystem cargo_sys;
		private final DigitalSupplier
			intake, transfer, feed, shooter;
		public ManualOverride(CargoSystem cs, DigitalSupplier i, DigitalSupplier t, DigitalSupplier f, DigitalSupplier s) {
			this.cargo_sys = cs;
			this.intake = i;
			this.transfer = t;
			this.feed = f;
			this.shooter = s;
			super.addRequirements(cs.intake, cs.transfer, cs.shooter);
		}
		@Override public void initialize() {
			System.out.println("ManualOverride: Running...");
		}
		@Override public void execute() {
			if(this.intake.get()) { this.cargo_sys.intake.set(Constants.intake_speed); }
			else { this.cargo_sys.intake.set(0); }
			if(this.transfer.get()) { this.cargo_sys.transfer.set(Constants.transfer_speed); }
			else { this.cargo_sys.transfer.set(0); }
			if(this.feed.get()) { this.cargo_sys.shooter.setFeed(Constants.feed_speed); }
			else { this.cargo_sys.shooter.setFeed(0); }
			if(this.shooter.get()) { this.cargo_sys.shooter.setShooterVelocity(Constants.shooter_default_speed); }
			else { this.cargo_sys.shooter.stopShooter(); }
		}
		@Override public void end(boolean i) {
			System.out.println("ManualOverride: Completed.");
			this.cargo_sys.intake.stop();
			this.cargo_sys.transfer.stop();
			this.cargo_sys.shooter.stopFeed();
			this.cargo_sys.shooter.stopShooter();

		}
		@Override public boolean isFinished() { return false; }
	}
	public static class IntakeCargo extends CommandBase {	// runs intake until a ball is detected by lim switch
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
		@Override public void initialize() {
			System.out.println("IntakeCargo: Running...");
		}
		@Override public void execute() { 
			if(this.state.spaceAvailable()) {
				this.intake_sys.set(this.speed); 
			}
		}
		@Override public void end(boolean i) {
			System.out.println("IntakeCargo: Completed.");
		}
		@Override public boolean isFinished() {
			boolean temp = this.last_lim;
			this.last_lim = this.state.isCargoEntering();
			return !this.state.spaceAvailable() || (!temp && this.last_lim);	// << a ball triggered the limit switch
		}
	}
	private static class AutomaticTransfer extends CommandBase {	// runs transfer belts based on limit switches
		private final Transfer transfer;
		private final CargoStates state;
		public AutomaticTransfer(Transfer t, CargoStates s) {
			this.transfer = t;
			this.state = s;
			super.addRequirements(t);
		}
		@Override public void initialize() {
			System.out.println("AutomaticTransfer: Running...");
		}
		@Override public void execute() {
			if(this.state.isCargoEntering() && !this.state.isCargoExiting() && this.state.spaceAvailable()) {
				this.transfer.set(Constants.transfer_speed);
			}
		}
		@Override public void end(boolean i) { 
			this.transfer.stop();
			System.out.println("AutomaticTransfer: Interrupted.");
		}
		@Override public boolean isFinished() { return false; }
	}
	public static class ShootOne extends CommandBase {	// spins up shooter motor then feeds until a ball is shot
		private final Shooter shooter;
		private final CargoStates state;
		private final double velocity;	// <- probably in meters per second
		private final int initial_count;
		public ShootOne(Shooter sh, CargoStates st, double v) {
			this.shooter = sh;
			this.state = st;
			this.velocity = v;
			this.initial_count = st.cargoCount();
			super.addRequirements(sh);
		}
		@Override public void initialize() {
			System.out.println("ShootOne: Running...");
		}
		@Override public void execute() {
			this.shooter.setShooterVelocity(this.velocity);
			if(Math.abs(this.velocity - this.shooter.getShooterVelocity()) < Constants.shooter_speed_tollerance && this.state.isCargoExiting()) {
				this.shooter.setFeed(Constants.feed_speed);
			}
		}
		@Override public void end(boolean i) {
			this.shooter.stopFeed();
			this.shooter.stopShooter();
			System.out.println(i ? "ShootOne: Terminated." : "ShootAll: Completed.");
		}
		@Override public boolean isFinished() {
			return !this.state.canShoot() || this.state.cargoCount() + 1 == this.initial_count;
		}
	}
	public static class ShootAll extends CommandBase {	// shoots all balls currently in transfer system
		private final Shooter shooter;
		private final CargoStates state;
		private final double velocity;	// <- probably in meters per second
		public ShootAll(Shooter sh, CargoStates st, double v) {
			this.shooter = sh;
			this.state = st;
			this.velocity = v;
			super.addRequirements(sh);
		}
		@Override public void initialize() {
			System.out.println("ShootAll: Running...");
		}
		@Override public void execute() {
			this.shooter.setShooterVelocity(this.velocity);
			if(Math.abs(this.velocity - this.shooter.getShooterVelocity()) < Constants.shooter_speed_tollerance && this.state.isCargoExiting()) {
				this.shooter.setFeed(Constants.feed_speed);
			}
		}
		@Override public void end(boolean i) {
			this.shooter.stopFeed();
			this.shooter.stopShooter();
			System.out.println(i ? "ShootAll: Terminated." : "ShootAll: Completed.");
		}
		@Override public boolean isFinished() {
			return !this.state.canShoot();
		}
	}
	public static class ShootVision extends CommandBase {	// shoot all balls in transfer system based on distance given by vision systems
		private final Shooter shooter;
		private final CargoStates state;
		private double t_velocity = 0.0;
		private boolean failed = false;
		public ShootVision(Shooter sh, CargoStates st, DriveBase db) {
			this.shooter = sh;
			this.state = st;
			super.addRequirements(sh, db);	// drivebase is required so we can garentee it isn't moving
		}
		@Override public void initialize() {
			VisionServer.Get().applyCameraPreset(Constants.cam_hub_pipeline);
			if(!RapidReactVision.verifyHubPipelineActive()) {
				System.out.println("ShootVision: Failed to set UpperHub pipeline");
				this.failed = true;
				return;
			}
			System.out.println("ShootVision: Running...");
		}
		@Override public void execute() {
			
		}
		@Override public void end(boolean i) {

		}
		@Override public boolean isFinished() {
			return false;
		}
	}




	// open loop because build team has let me down :(
	
	public static class OLIntake extends CommandBase {
		private final Intake intake;
		private final DigitalSupplier f;
		public OLIntake(Intake i, DigitalSupplier finish) {
			this.intake = i;
			this.f = finish;
			super.addRequirements(i);
		}
		@Override public void initialize() {
			System.out.println("OLIntake: Running...");
		}
		@Override public void execute() {
			this.intake.set(Constants.intake_speed);
		}
		@Override public void end(boolean i) {
			System.out.println(i ? "OLIntake: Terminated." : "OLIntake: Finished.");
			this.intake.stop();
		}
		@Override public boolean isFinished() {
			return this.f.get();
		}
	}
	public static class OLTransfer extends CommandBase {
		private final Transfer transfer;
		private final DigitalSupplier f;
		public OLTransfer(Transfer i, DigitalSupplier finish) {
			this.transfer = i;
			this.f = finish;
			super.addRequirements(i);
		}
		@Override public void initialize() {
			System.out.println("OLTransfer: Running...");
		}
		@Override public void execute() {
			this.transfer.set(Constants.transfer_speed);
		}
		@Override public void end(boolean i) {
			System.out.println(i ? "OLTransfer: Terminated." : "OLTransfer: Finished.");
			this.transfer.stop();
		}
		@Override public boolean isFinished() {
			return this.f.get();
		}
	}
	public static class OLShoot extends CommandBase {
		private final Shooter shooter;
		private final DigitalSupplier f, feed;
		private final double velocity;
		public OLShoot(Shooter s, DigitalSupplier feed, DigitalSupplier finish, double v) {
			this.shooter = s;
			this.f = finish;
			this.feed = feed;
			this.velocity = v;
			super.addRequirements(s);
		}
		@Override public void initialize() {
			System.out.println("OLShoot: Running...");
		}
		@Override public void execute() {
			this.shooter.setShooterVelocity(this.velocity);
			if(this.feed.get()) {
				this.shooter.setFeed(Constants.feed_speed);
			}
		}
		@Override public void end(boolean i) {
			System.out.println(i ? "OLShoot: Terminated." : "OLShoot: Finished.");
			this.shooter.stopFeed();
			this.shooter.stopShooter();
		}
		@Override public boolean isFinished() {
			return this.f.get();
		}
	}




	// special week 0 subsystem because our robot is lacking
	public static class WeekZero extends SubsystemBase {

		private static class W0_Intake extends Intake {
			public W0_Intake(int p) { super(p); }
		}
		private static class W0_Transfer extends Transfer {
			public W0_Transfer(int... ports) { super(null, null, ports); }
			@Override protected boolean isIntakeTriggered() { return false; }
			@Override protected boolean isShooterTriggered() { return false; }
			@Override protected void enableAutomaticTransfer(CargoStates s) {}
		}
		private static class W0_Shooter extends Shooter {
			private final PWMVictorSPX lp_main;	// "low-power" main -> currently the falcon is not installed :(
			public W0_Shooter(int lpm, int f) {
				super(null, null, f);
				this.lp_main = new PWMVictorSPX(lpm);
			}
			protected void set(double s) { this.lp_main.set(s); }
			protected void setVoltage(double v) { this.lp_main.setVoltage(v); }
			protected void stop() { this.lp_main.stopMotor(); }
			@Override protected void setShooterPercentage(double p) { this.set(p); }
			@Override protected void setShooterVelocityRaw(double vr) {}
			@Override protected void setShooterVelocity(double v) {}
			@Override protected void stopShooter() { this.stop(); }
			@Override public double  getShooterRawPosition() { return 0.0; }
			@Override public double getShooterRawVelocity() { return 0.0; }
			@Override public double getShooterVelocity() { return 0.0; }
		}

		private final W0_Intake intake;
		private final W0_Transfer transfer;
		private final W0_Shooter shooter;

		public WeekZero(
			int intake_port, int feed_port,
			int lpmain_port, int... transfer_ports
		) {
			this.intake = new W0_Intake(intake_port);
			this.transfer = new W0_Transfer(transfer_ports);
			this.shooter = new W0_Shooter(lpmain_port, feed_port);
		}



		public IntakeEnable intakeControl(double s, DigitalSupplier finish) {
			return new IntakeEnable(this.intake, s, finish);
		}
		public IntakeEnable intakeControl() {
			return new IntakeEnable(this.intake);
		}
		public TransferEnable transferControl(double s, DigitalSupplier finish) {
			return new TransferEnable(this.transfer, s, finish);
		}
		public TransferEnable transferControl() {
			return new TransferEnable(this.transfer);
		}
		public ShootRoutine shooterControl(double s, DigitalSupplier feed, DigitalSupplier finish) {
			return new ShootRoutine(this.shooter, s, feed, finish);
		}
		public ShootRoutine shooterControl() {
			return new ShootRoutine(this.shooter);
		}


		public static class IntakeEnable extends CommandBase {
			private final W0_Intake intake;
			private final DigitalSupplier finish;
			private final double speed;	// percentage of max output
			public IntakeEnable(W0_Intake i) { this(i, Constants.intake_speed, ()->false); }
			public IntakeEnable(W0_Intake i, double s) { this(i, s, ()->false); }
			public IntakeEnable(W0_Intake i, double s, DigitalSupplier finish) {
				this.intake = i;
				this.speed = s;
				this.finish = finish;
				super.addRequirements(i);
			}
			@Override public void initialize() { System.out.println("W0 IntakeEnable: Running..."); }
			@Override public void execute() { this.intake.set(this.speed); }
			@Override public void end(boolean i) {
				this.intake.stop();
				System.out.println(i ? "W0 IntakeEnable: Terminated." : "W0 IntakeEnable: Completed.");
			}
			@Override public boolean isFinished() { return this.finish.get(); }
		}
		public static class TransferEnable extends CommandBase {
			private final W0_Transfer transfer;
			private final DigitalSupplier finish;
			private final double speed;	// percentage of max output
			public TransferEnable(W0_Transfer t) { this(t, Constants.transfer_speed, ()->false); }
			public TransferEnable(W0_Transfer t, double s) { this(t, s, ()->false); }
			public TransferEnable(W0_Transfer t, double s, DigitalSupplier finish) {
				this.transfer = t;
				this.speed = s;
				this.finish = finish;
				super.addRequirements(t);
			}
			@Override public void initialize() { System.out.println("W0 TransferEnable: Running..."); }
			@Override public void execute() { this.transfer.set(this.speed); }
			@Override public void end(boolean i) {
				this.transfer.stop();
				System.out.println(i ? "W0 TransferEnable: Terminated." : "W0 TransferEnable: Completed.");
			}
			@Override public boolean isFinished() { return this.finish.get(); }
		}
		public static class ShootRoutine extends CommandBase {
			private final W0_Shooter shooter;
			private final DigitalSupplier feed, finish;
			private final double percentage;
			public ShootRoutine(W0_Shooter s) { this(s, 0.8, ()->false, ()->false); }
			public ShootRoutine(W0_Shooter s, double p) { this(s, p, ()->false, ()->false); }
			public ShootRoutine(W0_Shooter s, DigitalSupplier feed) { this(s, 0.8, feed, ()->false); }
			public ShootRoutine(W0_Shooter s, double p, DigitalSupplier feed, DigitalSupplier finish) {
				this.shooter = s;
				this.percentage = p;
				this.feed = feed;
				this.finish = finish;
				super.addRequirements(s);
			}
			@Override public void initialize() { System.out.println("W0 BasicShoot: Running..."); }
			@Override public void execute() { 
				this.shooter.set(this.percentage);
				if(this.feed.get()) {
					this.shooter.setFeed(Constants.feed_speed);
				} else {
					this.shooter.setFeed(0.0);
				}
			}
			@Override public void end(boolean i) {
				this.shooter.stop();
				this.shooter.stopFeed();
				System.out.println(i ? "W0 BasicShoot: Terminated." : "W0 BasicShoot: Completed.");
			}
			@Override public boolean isFinished() { return this.finish.get(); }
		}
		// public static class VisionShootRoutine extends CommandBase {
		// 	private final W0_Shooter shooter;
		// 	private final DigitalSupplier feed, finish;
		// 	private final VisionServer.Conversion to_voltage;
		// 	private double last_calculated = Constants.shooter_default_speed;
		// 	private final String camera;
		// 	public VisionShootRoutine(W0_Shooter s) { this(s, ()->false, (double v)->v, ()->false, null); }
		// 	public VisionShootRoutine(W0_Shooter s, String cam_name) { this(s, ()->false, ()->false, cam_name); }
		// 	public VisionShootRoutine(W0_Shooter s, DigitalSupplier feed) { this(s, feed, ()->false, null); }
		// 	public VisionShootRoutine(W0_Shooter s, DigitalSupplier feed, String cam_name) { this(s, feed, ()->false, cam_name); }
		// 	public VisionShootRoutine(W0_Shooter s, DigitalSupplier feed, VisionServer.Conversion tv, DigitalSupplier finish, String cam_name) {
		// 		this.shooter = s;
		// 		this.feed = feed;
		// 		this.finish = finish;
		// 		this.camera = cam_name;
		// 		this.to_voltage = tv;
		// 		super.addRequirements(s);
		// 	}
		// 	@Override public void initialize() { 
		// 		VisionServer.Get().applyCameraPreset(Constants.cam_hub_pipeline);
		// 		if(this.camera != null) {
		// 			VisionServer.Get().setCamera(this.camera);
		// 		}
		// 		if(!RapidReactVision.verifyHubPipelineActive()) {
		// 			System.out.println("W0 VisionShoot: Failed to set UpperHub pipeline");
		// 		}
		// 		System.out.println("W0 VisionShoot: Running..."); 
		// 	}
		// 	@Override public void execute() { 
		// 		this.shooter.set();
		// 		if(this.feed.get()) {
		// 			this.shooter.setFeed(Constants.feed_speed);
		// 		} else {
		// 			this.shooter.setFeed(0.0);
		// 		}
		// 	}
		// 	@Override public void end(boolean i) {
		// 		this.shooter.stop();
		// 		this.shooter.stopFeed();
		// 		System.out.println(i ? "W0 BasicShoot: Terminated." : "W0 BasicShoot: Completed.");
		// 	}
		// 	@Override public boolean isFinished() { return this.finish.get(); }
		// }
		public static class ManualOverride extends CommandBase {

		}


	}


}