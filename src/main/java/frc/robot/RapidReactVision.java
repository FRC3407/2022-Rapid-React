package frc.robot;

import frc.robot.team3407.Input.AnalogSupplier;
import frc.robot.team3407.drive.DriveBase;
import frc.robot.team3407.drive.DriveBase.DriveCommandBase;
import frc.robot.vision.java.VisionServer;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/**
 * Rapid-React specific vision methods and commands. For a general vision interface, see {@link VisionServer}
 */
public final class RapidReactVision {

	public static enum Cameras {
		HUB		("Hub"),		// the camera tilted upwards for viewing the hub
		CARGO	("Cargo");		// the camera pointed to view the ground

		public final String key;
		private Cameras(String k) {
			this.key = k;
		}

		public int getIndex() { return VisionServer.findCameraIdx(this.key); }
		public VisionServer.VsCamera getObject() { return VisionServer.getCamera(this.key); }
		public NetworkTable getTable() { return VisionServer.getCamerasTable().getSubTable(this.key); }
		public boolean setActive() { return VisionServer.setCamera(this.key); }
	}
	public static enum Pipelines {
		HUB_TRACKER		("Upper-Hub Pipeline"),		// hub position tracking
		CARGO_TRACKER	("Cargo Pipeline");			// cargo position tracking

		public final String key;
		private Pipelines(String k) {
			this.key = k;
		}

		public int getIndex() { return VisionServer.findPipelineIdx(this.key); }
		public VisionServer.VsPipeline getObject() { return VisionServer.getPipeline(this.key); }
		public NetworkTable getTable() { return VisionServer.getPipelinesTable().getSubTable(this.key); }
		public boolean setActive() { return VisionServer.setPipeline(this.key); }
	}



	private VisionServer.VsPipeline
		upperhub = null, cargo = null;

	private RapidReactVision() {
		if(VisionServer.isConnected()) {
			this.upperhub = Pipelines.HUB_TRACKER.getObject();
			this.cargo = Pipelines.CARGO_TRACKER.getObject();
		}
		// if(!VisionServer.isConnected()) {
		// 	new Trigger(()->VisionServer.isConnected()).whenActive(
		// 		()->{
		// 			this.upperhub = Pipelines.HUB_TRACKER.getObject();
		// 			this.cargo = Pipelines.CARGO_TRACKER.getObject();
		// 		}
		// 	);
		// } else {
		// 	this.upperhub = Pipelines.HUB_TRACKER.getObject();
		// 	this.cargo = Pipelines.CARGO_TRACKER.getObject();
		// }
	}
	private static RapidReactVision inst = new RapidReactVision();


// BASIC CHECKS/UPDATERS
	public static boolean hasHubPipeline() { return Pipelines.HUB_TRACKER.getObject() != null; }
	public static boolean isHubPipelineValid() { return inst.upperhub != null; }
	public static boolean hasCargoPipeline() { return Pipelines.CARGO_TRACKER.getObject() != null; }
	public static boolean isCargoPipelineValid() { return inst.cargo != null; }
	public static boolean hasPipelines() { return hasHubPipeline() && hasCargoPipeline(); }
	public static boolean arePipelinesValid() { return isHubPipelineValid() && isCargoPipelineValid(); }

	public static boolean verifyHubPipeline() {
		if(!isHubPipelineValid()) {
			//System.out.println("VerifyHubPipeline: pipeline not valid, updating...");
			inst.upperhub = Pipelines.HUB_TRACKER.getObject();
			return isHubPipelineValid();
		}
		//System.out.println("VerifyHubPipeline: pipeline already valid");
		return true;
	}
	public static boolean verifyCargoPipeline() {
		if(!isCargoPipelineValid()) {
			inst.cargo = Pipelines.CARGO_TRACKER.getObject();
			return isCargoPipelineValid();
		}
		return true;
	}
	public static boolean updatePipelines() {
		return verifyHubPipeline() || verifyCargoPipeline();
	}


// NT PARAMETER SETTERS
	public static boolean setHubPipelineScaling(int downscale) {	// returns false on failure
		if(verifyHubPipeline()) {
			return inst.upperhub.get().getEntry("Scaling").setDouble((double)downscale);
		}
		return false;
	}
	public static boolean showHubPipelineThreshold(boolean show) {	// returns false on failure
		if(verifyHubPipeline()) {
			return inst.upperhub.get().getEntry("Show Threshold").setBoolean(show);
		}
		return false;
	}
	public static boolean setHubPipelineOverlay(boolean show) {
		if(verifyHubPipeline()) {
			return inst.upperhub.get().getEntry("Enable Overlay").setBoolean(show);
		}
		return false;
	}
	public static boolean setHubPipelineMinRange(double inches) {
		if(verifyHubPipeline()) {
			return inst.upperhub.get().getEntry("Minimum Range").setDouble(inches);
		}
		return false;
	}
	public static boolean setHubPipelineMaxRange(double inches) {
		if(verifyHubPipeline()) {
			return inst.upperhub.get().getEntry("Maximum Range").setDouble(inches);
		}
		return false;
	}
	public static boolean setCargoPipelineScaling(int downscale) {	// returns false on failure
		if(verifyCargoPipeline()) {
			return inst.cargo.get().getEntry("Scaling").setDouble((double)downscale);
		}
		return false;
	}
	public static boolean showCargoPipelineThreshold(boolean show) {	// returns false on failure
		if(verifyCargoPipeline()) {
			return inst.cargo.get().getEntry("Show Threshold").setBoolean(show);
		}
		return false;
	}
	public static boolean showCargoPipelineContours(boolean show) {		// returns false on failure
		if(verifyCargoPipeline()) {
			return inst.cargo.get().getEntry("Show Contours").setBoolean(show);
		}
		return false;
	}


// HIGH LEVEL PIPELINE CONTROL
	public static boolean isHubPipelineActive() {
		if(verifyHubPipeline()) {
			return inst.upperhub == VisionServer.getCurrentPipeline();
		}
		return false;
	}
	public static boolean isCargoPipelineActive() {
		if(verifyCargoPipeline()) {
			return inst.cargo == VisionServer.getCurrentPipeline();
		}
		return false;
	}
	public static boolean setHubPipelineActive() {
		if(verifyHubPipeline()) {
			//System.out.println("SetHubPipelineActive: pipeline exists, attempting to set");
			return VisionServer.setPipeline(inst.upperhub.getIdx());
		}
		//System.out.println("SetHubPipelineActive: pipeline nonexistant");
		return false;
	}
	public static boolean setCargoPipelineActive() {
		if(verifyCargoPipeline()) {
			//System.out.println("SetCargoPipelineActive: pipeline exists, attempting to set");
			return VisionServer.setPipeline(inst.cargo.getIdx());
		}
		//System.out.println("SetCargoPipelineActive: pipeline nonexistant");
		return false;
	}
	public static boolean verifyHubPipelineActive() {
		if(!isHubPipelineActive()) {
			//System.out.println("VerifyHubPipelineActive: not active");
			return setHubPipelineActive();
		}
		//System.out.println("VerifyHubPipelineActive: already active");
		return true;
	}
	public static boolean verifyCargoPipelineActive() {
		if(!isCargoPipelineActive()) {
			//System.out.println("VerifyCargoPipelineActive: not active");
			return setCargoPipelineActive();
		}
		//System.out.println("VerifyCargoPipelineActive: already active");
		return true;
	}

	private static boolean isRedCargoEnabled() {
		if(verifyCargoPipeline()) {
			return inst.cargo.get().getEntry("Process Red").getBoolean(true);
		}
		return false;
	}
	private static boolean setRedCargoEnabled(boolean enable) {		// returns false on failure
		if(verifyCargoPipeline()) {
			return inst.cargo.get().getEntry("Process Red").setBoolean(enable);
		}
		return false;
	}
	private static boolean verifyRedCargo(boolean status) {
		if(verifyCargoPipeline()) {
			if(inst.cargo.get().getEntry("Process Red").getBoolean(status) != status) {
				return inst.cargo.get().getEntry("Process Red").setBoolean(status);
			}
			return true;
		}
		return false;
	}
	private static boolean isBlueCargoEnabled() {
		if(verifyCargoPipeline()) {
			return inst.cargo.get().getEntry("Process Blue").getBoolean(true);
		}
		return false;
	}
	private static boolean setBlueCargoEnabled(boolean enable) {		// returns false on failure
		if(verifyCargoPipeline()) {
			return inst.cargo.get().getEntry("Process Blue").setBoolean(enable);
		}
		return false;
	}
	private static boolean verifyBlueCargo(boolean status) {
		if(verifyCargoPipeline()) {
			if(inst.cargo.get().getEntry("Process Blue").getBoolean(status) != status) {
				return inst.cargo.get().getEntry("Process Blue").setBoolean(status);
			}
			return true;
		}
		return false;
	}
	public static boolean isCargoAllianceColorMode(DriverStation.Alliance a) {
		switch(a) {
			case Red:
				return isRedCargoEnabled() && !isBlueCargoEnabled();
			case Blue:
				return !isRedCargoEnabled() && isBlueCargoEnabled();
			case Invalid:
			default:
				return (isRedCargoEnabled() && isBlueCargoEnabled()) || (!isRedCargoEnabled() && !isBlueCargoEnabled());
		}
	}
	public static boolean setCargoAllianceColorMode(DriverStation.Alliance a) {	// returns false on failure
		switch(a) {
			case Red: 
				return setRedCargoEnabled(true) && setBlueCargoEnabled(false);
			case Blue:
				return setRedCargoEnabled(false) && setBlueCargoEnabled(true);
			case Invalid:
			default:
				return setRedCargoEnabled(true) && setBlueCargoEnabled(true);
		}
	}
	public static boolean verifyCargoAllianceColorMode(DriverStation.Alliance a) {
		verifyCargoPipelineActive();
		switch(a) {
			case Red:
				return (verifyRedCargo(true) && verifyBlueCargo(false)); 
			case Blue:
				return (verifyRedCargo(false) && verifyBlueCargo(true));
			case Invalid:
			default:
				return (verifyRedCargo(true) && verifyBlueCargo(true)) || (verifyRedCargo(false) && verifyBlueCargo(false));
		}
	}
	public static boolean activeTargetMatchesAlliance(DriverStation.Alliance a) {
		switch(a) {
			case Red:
				return VisionServer.getActiveTargetName().equals("Cargo-1r");
			case Blue:
				return VisionServer.getActiveTargetName().equals("Cargo-1b");
			case Invalid:
			default:
				return VisionServer.getActiveTargetName().equals("none");
		}
	}
	public static boolean disableCargoProcessing() {		// returns false on failure
		return setRedCargoEnabled(false) || setBlueCargoEnabled(false);
	}


// MAIN METHODS OF USE -> all return null on incorrect target or other error
	public static VisionServer.TargetData getHubPosition() {	// returns null on incorrect target
		verifyHubPipelineActive();
		return VisionServer.getTargetDataIfMatching("Upper-Hub");
	}
	public static boolean isHubDetected() {
		return getHubPosition() != null;
	}

	public static VisionServer.TargetData getClosestAllianceCargo(DriverStation.Alliance a) {	// returns null on incorrect target
		verifyCargoPipelineActive();
		switch(a) {
			case Red:
				if(verifyRedCargo(true) && verifyBlueCargo(false)) {
					return VisionServer.getTargetDataIfMatching("Cargo-1r");
				}
				break;
			case Blue:
				if(verifyRedCargo(false) && verifyBlueCargo(true)) {
					return VisionServer.getTargetDataIfMatching("Cargo-1b");
				}
				break;
			case Invalid:
			default:
				return VisionServer.getTargetData();
		}
		return null;
	}
	public static VisionServer.TargetData getClosestRedPosition() {		// returns null on incorrect target
		return getClosestAllianceCargo(DriverStation.Alliance.Red);
	}
	public static VisionServer.TargetData getClosestBluePosition() {	// returns null on incorrect target
		return getClosestAllianceCargo(DriverStation.Alliance.Blue);
	}
	public static boolean isAllianceCargoDetected(DriverStation.Alliance a) {
		return getClosestAllianceCargo(a) != null;
	}



	public static double getHubBaseDistance(VisionServer.TargetData d, double height) {		// output in inches
		return Math.sqrt(Math.pow(d.distance, 2) - Math.pow(height, 2));	// pythagorean solved for A (sqrt(C^2 - B^2) = A)
	}







/*
	This is to make it more obvious what constants are used in the commands below and where they come from
*/
	public static final double
		cargo_max_range = Constants.max_cargo_range_inches,
		hub_max_range = Constants.max_hub_range_inches,
		max_heading_offset = Constants.max_heading_offset,
		heading_thresh = Constants.heading_offset_thresh,
		distance_thresh = 2.0,	// for hub-assist

		default_cargo_target = Constants.cargo_follow_target_inches,

		continuation_percent = Constants.uncertainty_continuation_percentage,

		static_forward_voltage = Constants.cl_params.static_voltage,
		static_turning_voltage = Constants.turning_static_voltage,
		default_max_forward_voltage = Constants.auto_max_forward_voltage,
		default_max_turning_voltage = Constants.auto_max_turn_voltage,
		default_max_voltage_ramp = Constants.auto_max_voltage_ramp
	;





	public static class CargoFind extends DriveBase.DriveCommandBase {
	
		private final Alliance team;
		private final SlewRateLimiter limit;
		private final double turning_voltage;
		private double last_voltage = 0.0;
		private boolean failed = false;

		public CargoFind(DriveBase db) { this(db, DriverStation.getAlliance()); }
		public CargoFind(DriveBase db, Alliance a) { this(db, a, default_max_turning_voltage); }
		public CargoFind(DriveBase db, Alliance a, double tvolts) { this(db, a, tvolts, default_max_voltage_ramp); }
		public CargoFind(DriveBase db, Alliance a, double tvolts, double mvr) {
			super(db);
			this.team = a;
			this.limit = new SlewRateLimiter(mvr);
			this.turning_voltage = tvolts;
		}
	
		@Override public void initialize() {
			Constants.vision_cargo.run();
			if(!verifyCargoPipelineActive()) {
				System.out.println("CargoFind: Failed to set Cargo pipeline");
				this.failed = true;
				return;
			}
			System.out.println("CargoFind: Running...");
			this.failed = false;	// in case the command is run again after a failure
		}
		@Override public void execute() {
			this.last_voltage = this.limit.calculate(getClosestAllianceCargo(this.team) != null ? 0.0 : this.turning_voltage);
			super.autoTurnVoltage(this.last_voltage);
		}
		@Override public void end(boolean i) {
			super.autoTurnVoltage(0);
			System.out.println(this.failed || i ? "CargoFind: Terminated." : "CargoFind: Completed.");
		}
		@Override public boolean isFinished() {
			return this.failed || (getClosestAllianceCargo(this.team) != null && this.last_voltage < static_turning_voltage);
		}
	
	
	}
	public static class CargoTurn extends DriveBase.DriveCommandBase {

		private final Alliance team;
		private final double max_turn_voltage;
		private VisionServer.TargetData position = null;
		private boolean failed = false;

		public CargoTurn(DriveBase db) { this(db, DriverStation.getAlliance()); }
		public CargoTurn(DriveBase db, Alliance a) { this(db, a, default_max_turning_voltage); }
		public CargoTurn(DriveBase db, Alliance a, double mtvolts) {
			super(db);
			this.team = a;
			this.max_turn_voltage = mtvolts;
		}
	
		@Override public void initialize() {
			Constants.vision_cargo.run();
			if(!verifyCargoPipelineActive()) {
				System.out.println("CargoTurn: Failed to set Cargo pipeline");
				this.failed = true;
				return;
			}
			System.out.println("CargoTurn: Running...");
			this.failed = false;
		}
		@Override public void execute() {
			this.position = getClosestAllianceCargo(this.team);
			if(this.position != null) {
				double v = MathUtil.clamp(this.position.lr / max_heading_offset, -1.0, 1.0) * (this.max_turn_voltage - static_turning_voltage);
				super.autoTurn((Math.signum(v) * static_turning_voltage) + v);
			} else {
				super.fromLast(continuation_percent);	// % of what was last set (decelerating)
			}
		}
		@Override public void end(boolean i) {
			super.autoDriveVoltage(0, 0);
			System.out.println(this.failed ? "CargoTurn: Terminated." : "CargoTurn: Completed.");
		}
		@Override public boolean isFinished() {	// change threshold angle when testing >>
			if(this.position != null) {
				return Math.abs(this.position.lr) < heading_thresh;
			}
			return this.failed;
		}


		/** Extension of CargoTurn class that runs indefinately */
		public static class Demo extends CargoTurn {
	
			public Demo(DriveBase db) { super(db); }
			public Demo(DriveBase db, Alliance a) { super(db, a); }
			public Demo(DriveBase db, Alliance a, double mtvolts) { super(db, a, mtvolts); }
	
			@Override public boolean isFinished() { return super.failed; }
	
		}
	
	
	}
	public static class CargoFollow extends DriveBase.DriveCommandBase {

		private final Alliance team;
		private final SlewRateLimiter f_limit;
		private final double target, max_forward_voltage, max_turning_voltage;
		private VisionServer.TargetData position = null;
		private boolean failed = false;
		protected double cont_percent = continuation_percent;

		public CargoFollow(DriveBase db) { this(db, DriverStation.getAlliance()); }
		public CargoFollow(DriveBase db, double target_inches) { this(db, target_inches, default_max_voltage_ramp); }
		public CargoFollow(DriveBase db, double target_inches, double mfvr) { this(db, target_inches, default_max_forward_voltage, default_max_turning_voltage, mfvr); }
		public CargoFollow(DriveBase db, double target_inches, double mfvolts, double mtvolts, double mfvr) { this(db, DriverStation.getAlliance(), target_inches, mfvolts, mtvolts, mfvr); }
		public CargoFollow(DriveBase db, Alliance a) { this(db, a, default_cargo_target); }
		public CargoFollow(DriveBase db, Alliance a, double target_inches) { this(db, a, target_inches, default_max_voltage_ramp); }
		public CargoFollow(DriveBase db, Alliance a, double target_inches, double mfa) { this(db, a, target_inches, default_max_forward_voltage, default_max_turning_voltage, mfa); }
		// 					drivebase, alliance, distance target, max forward voltage, max turn voltage, max forard voltage acceleration
		public CargoFollow(DriveBase db, Alliance a, double target_inches, double mfvolts, double mtvolts, double mfvr) {
			super(db);
			this.team = a;
			this.f_limit = new SlewRateLimiter(mfvr);
			this.target = target_inches;
			this.max_forward_voltage = mfvolts;
			this.max_turning_voltage = mtvolts;
		}
	
		@Override public void initialize() {
			Constants.vision_cargo.run();
			if(!verifyCargoPipelineActive()) {
				System.out.println(getClass().getSimpleName() + ": Failed to set Cargo pipeline");
				this.failed = true;
				return;
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
			this.failed = false;
		}
		@Override public void execute() {
			this.position = getClosestAllianceCargo(this.team);
			if(this.position != null) {
				double f = MathUtil.clamp(
					(this.position.distance - this.target) / cargo_max_range, -1.0, 1.0		// the forward error, clamped to [-1, 1]
				) * (this.max_forward_voltage - static_forward_voltage);					// multiply by forward voltage range
				double f_l = this.f_limit.calculate(f);						// calculate rate-limited voltage
				double t = MathUtil.clamp(
					this.position.lr / max_heading_offset, -1.0, 1.0		// the turning error, clamped to [-1, 1]
				) * (this.max_turning_voltage - static_turning_voltage);		// multiply by turning voltage range
				t *= (f_l / f);		// normalize turning so that it is proportional to the rate-limited forward speed
				double st = static_forward_voltage + Math.abs((static_turning_voltage - static_forward_voltage) * (t / this.max_turning_voltage));	// recalculate static voltage based on proportion of turning (scaled from 'static forward' to 'static turning')
				super.autoDriveVoltage(
					st * Math.signum(f_l + t) + f_l + t,	// static voltage (in correct direction) + limited forward voltage + normalized turning voltage
					st * Math.signum(f_l - t) + f_l - t		// static voltage (in correct direction) + limited forward voltage - normalized turning voltage
				);
			} else { 
				super.fromLast(this.cont_percent);	// handle jitters in vision detection -> worst case cenario this causes a gradual deceleration
			}
		}
		@Override public void end(boolean i) {
			super.autoDriveVoltage(0.0, 0.0);
			System.out.println(getClass().getSimpleName() + (this.failed || i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			if(this.position != null) {
				return Math.abs(this.position.lr) <= heading_thresh && Math.abs(this.position.distance) <= this.target;
			}
			return this.failed;
		}

		public boolean inRange(double dist_inches) {
			return this.position != null && this.position.distance <= dist_inches;
		}


		/** Extension of CargoFollow that runs indefinately */
		public static class Demo extends CargoFollow {
	
			public Demo(DriveBase db) { super(db); }
			public Demo(DriveBase db, double target_inches) { super(db, target_inches); }
			public Demo(DriveBase db, double target_inches, double mfvr) { super(db, target_inches, mfvr); }
			public Demo(DriveBase db, double target_inches, double mfvolts, double mtvolts, double mfvr) { super(db, target_inches, mfvolts, mtvolts, mfvr); }
			public Demo(DriveBase db, Alliance a) { super(db, a); }
			public Demo(DriveBase db, Alliance a, double target_inches) { super(db, a, target_inches); }
			public Demo(DriveBase db, Alliance a, double target_inches, double mfa) { super(db, a, target_inches, mfa); }
			public Demo(DriveBase db, Alliance a, double target_inches, double mfvolts, double mtvolts, double mfa) { super(db, a, target_inches, mfvolts, mtvolts, mfa); }
	
			@Override public boolean isFinished() { return super.failed; }
	
		}
	
	
	}
	/**
	 * Merges CargoFollow and ModeDrive together 
	 */
	public static class CargoAssistRoutine extends CargoFollow {

		public static final double intake_enable_distance = 20;		// when the cargo gets this close (inches), the intake will be started
		public static final int intake_continuation_cycles = 75;	// 1.5 seconds

		private final DriveCommandBase drive;
		private final CargoSystem.IntakeSubsystem.IntakeCommand intake;
		private int intake_count = -1;

		public CargoAssistRoutine(DriveBase db, DriveCommandBase d, CargoSystem.IntakeSubsystem.IntakeCommand i) {
			super(db);
			this.drive = d;
			this.intake = i;
			super.cont_percent = 1.0;

			super.addRequirements(this.intake.getRequirements().toArray(new CargoSystem.IntakeSubsystem[]{}));
		}
		public CargoAssistRoutine(
			DriveBase db, DriveCommandBase d, 
			CargoSystem.IntakeSubsystem.IntakeCommand i, 
			Alliance a, 
			double target_inches, double mfvolts, double mtvolts, double mfva
		) {
			super(db, a, target_inches, mfvolts, mtvolts, mfva);
			this.drive = d;
			this.intake = i;
			super.cont_percent = 1.0;

			super.addRequirements(this.intake.getRequirements().toArray(new CargoSystem.IntakeSubsystem[]{}));
		}

		@Override public void initialize() {
			super.initialize();
			if(this.drive.isScheduled()) {
				this.drive.cancel();
			}
			if(this.intake.isScheduled()) {
				this.intake.cancel();
			}
			this.drive.initialize();
			this.intake.initialize();
			this.intake_count = -1;
		}
		@Override public void execute() {
			super.execute();
			if(super.position != null) {
				if(super.position.distance <= intake_enable_distance) {
					this.intake.execute();
					this.intake_count = 0;
				}
			} else {
				if(this.intake_count <= intake_continuation_cycles && this.intake_count != -1) {
					this.intake.execute();
					this.intake_count++;
				} else if(this.intake_count != -1) {
					this.intake.end(false);
					this.intake_count = -1;
				}
				this.drive.execute();
			}
		}
		@Override public void end(boolean i) {
			super.end(i);
			this.drive.end(i);
			this.intake.end(i);
		}
		@Override public boolean isFinished() {
			return false;
		}


		public static class EndOnIntake extends CargoAssistRoutine {

			private final BooleanSupplier end;
			private final boolean use_timeout;

			public EndOnIntake(DriveBase db, DriveCommandBase d, CargoSystem.IntakeSubsystem.IntakeCommand i, CargoSystem.TransferSubsystem t) {
				super(db, d, i);
				this.end = ()->t.isInputRisingEdge();
				this.use_timeout = !t.hasFeedback();
			}
			public EndOnIntake(
				DriveBase db, DriveCommandBase d, 
				CargoSystem.IntakeSubsystem.IntakeCommand i, 
				CargoSystem.TransferSubsystem t,
				Alliance a, 
				double target_inches, double mfvolts, double mtvolts, double mfva
			) {
				super(db, d, i, a, target_inches, mfvolts, mtvolts, mfva);
				this.end = ()->t.isInputRisingEdge();
				this.use_timeout = !t.hasFeedback();
			}

			@Override public boolean isFinished() {
				return this.end.getAsBoolean() || (this.use_timeout && super.intake_count > intake_continuation_cycles);
			}


		}


	}
	public static class HubFind extends DriveBase.DriveCommandBase {

		private final SlewRateLimiter limit;
		private final double turning_voltage;
		private double last_voltage = 0.0;
		private boolean failed = false;

		public HubFind(DriveBase db) { this(db, default_max_turning_voltage, default_max_voltage_ramp); }
		public HubFind(DriveBase db, double tvolts, double mvr) {
			super(db);
			this.limit = new SlewRateLimiter(mvr);
			this.turning_voltage = tvolts;
		}
	
		@Override public void initialize() {
			Constants.vision_hub.run();
			if(!verifyHubPipelineActive()) {
				System.out.println(getClass().getSimpleName() + ": Failed to set UpperHub pipeline.");
				this.failed = true;
				return;
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
			this.failed = false;
		}
		@Override public void execute() {
			this.last_voltage = this.limit.calculate(isHubDetected() ? 0.0 : this.turning_voltage);
			super.autoTurnVoltage(this.last_voltage);
		}
		@Override public void end(boolean i) {
			super.autoTurnVoltage(0);
			System.out.println(getClass().getSimpleName() + (this.failed || i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			return this.failed || (isHubDetected() && this.last_voltage < static_turning_voltage);
		}


	}
	public static class HubTurn extends DriveBase.DriveCommandBase {

		private final double max_turn_voltage;
		private VisionServer.TargetData position = null;
		private boolean failed = false;

		public HubTurn(DriveBase db) { this(db, default_max_turning_voltage); }
		public HubTurn(DriveBase db, double mtvolts) {
			super(db);
			this.max_turn_voltage = mtvolts;
		}
	
		@Override public void initialize() {
			Constants.vision_hub.run();
			if(!verifyHubPipelineActive()) {
				System.out.println(getClass().getSimpleName() + ": Failed to set UpperHub pipeline");
				this.failed = true;
				return;
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
			this.failed = false;
		}
		@Override public void execute() {
			this.position = getHubPosition();
			if(this.position != null) {
				double v = MathUtil.clamp(
					this.position.lr / max_heading_offset, -1.0, 1.0
				) * (this.max_turn_voltage - static_turning_voltage);
				super.autoTurnVoltage((Math.signum(v) * static_turning_voltage) + v);
			} else {
				super.fromLast(continuation_percent);
			}
		}
		@Override public void end(boolean i) {
			super.autoTurnVoltage(0);
			System.out.println(getClass().getSimpleName() + (this.failed || i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			if(this.position != null) {
				return Math.abs(this.position.lr) <= heading_thresh;
			}
			return this.failed;
		}


	}
	public static class HubTarget extends DriveBase.DriveCommandBase {

		private final SlewRateLimiter f_limit;
		private final double target, max_forward_voltage, max_turn_voltage, max_voltage_distance;
		private VisionServer.TargetData position = null;
		private boolean failed = false;
		protected double cont_percent = continuation_percent;

		public HubTarget(DriveBase db, double target_inches, double mvolts_distance, double mfvolts, double mtvolts, double mfvr) {
			super(db);
			this.f_limit = new SlewRateLimiter(mfvr);
			this.target = target_inches;
			this.max_voltage_distance = mvolts_distance;
			this.max_forward_voltage = mfvolts;
			this.max_turn_voltage = mtvolts;
		}

		@Override public void initialize() {
			Constants.vision_hub.run();
			if(!verifyHubPipelineActive()) {
				System.out.println(getClass().getSimpleName() + ": Failed to set UpperHub pipeline");
				this.failed = true;
				return;
			}
			System.out.println(getClass().getSimpleName() + ": Running...");
			this.failed = false;
		}
		@Override public void execute() {
			this.position = getHubPosition();
			if(this.position != null) {
				double f = MathUtil.clamp(
					(this.position.distance - this.target) / this.max_voltage_distance, -1.0, 1.0
				) * (this.max_forward_voltage - static_forward_voltage);
				double f_l = this.f_limit.calculate(f);
				double t = MathUtil.clamp(
					this.position.lr / max_heading_offset, -1.0, 1.0
				) * (this.max_turn_voltage - static_turning_voltage);
				t *= (f_l / f);
				double st = static_forward_voltage + Math.abs((static_turning_voltage - static_forward_voltage) * (t / this.max_turn_voltage));
				super.autoDriveVoltage(
					st * Math.signum(f_l + t) + f_l + t,
					st * Math.signum(f_l - t) + f_l - t
				);
			} else {
				super.fromLast(this.cont_percent);
			}
		}
		@Override public void end(boolean i) {
			super.autoDriveVoltage(0.0, 0.0);
			System.out.println(getClass().getSimpleName() + (this.failed || i ? ": Terminated." : ": Completed."));
		}
		@Override public boolean isFinished() {
			if(this.position != null) {
				return Math.abs(this.position.lr) <= heading_thresh && Math.abs(this.position.distance - this.target) <= distance_thresh;
			}
			return this.failed;
		}


	}
	public static class HubAssistRoutine extends HubTurn {

		public static final int discontinuity_timeout_cycles = 50;

		private final AnalogSupplier control;
		private final SlewRateLimiter olimit;
		private double last_voltage = 0.0;
		private int misses = 0;

		public HubAssistRoutine(DriveBase db, AnalogSupplier op_input) { this(db, op_input, default_max_turning_voltage); }
		public HubAssistRoutine(DriveBase db, AnalogSupplier op_input, double mtvolts) { this(db, op_input, mtvolts, default_max_voltage_ramp); }
		public HubAssistRoutine(DriveBase db, AnalogSupplier op_input, double mtvolts, double mvr) {
			super(db, mtvolts);
			this.control = op_input;
			this.olimit = new SlewRateLimiter(mvr);
		}
		@Override public void execute() {
			super.position = getHubPosition();
			if(super.position != null) {
				this.misses = 0;
				double p = MathUtil.clamp(
					super.position.lr / max_heading_offset, -1.0, 1.0
				) * (super.max_turn_voltage - static_turning_voltage) * Math.abs(this.control.get());
				this.last_voltage = (Math.signum(p) * static_turning_voltage) + p;
				this.olimit.calculate(this.last_voltage);
			} else if(this.misses >= discontinuity_timeout_cycles) {	// if hub goes undetected for more than a second
				this.last_voltage = this.olimit.calculate(this.control.get() * super.max_turn_voltage);
			} else {
				this.misses++;
				this.last_voltage *= continuation_percent;
				this.olimit.calculate(this.last_voltage);
			}
			super.autoTurnVoltage(this.last_voltage);
		}
		@Override public boolean isFinished() { return false; }


	}
	public static class HubAssistRoutineV2 extends HubTarget {

		private final DriveCommandBase drive;
		
		public HubAssistRoutineV2(
			DriveBase db, DriveCommandBase d, double target_inches, double mvolts_distance, double mfvolts, double mtvolts, double mfvr
		) {
			super(db, target_inches, mvolts_distance, mfvolts, mtvolts, mfvr);
			this.drive = d;
			this.cont_percent = 1.0;
		}

		@Override public void initialize() {
			super.initialize();
			if(this.drive.isScheduled()) {
				this.drive.cancel();
			}
			this.drive.initialize();
		}
		@Override public void execute() {
			super.execute();
			if(super.position == null) {
				this.drive.execute();
			}
		}
		@Override public void end(boolean i) {
			super.end(i);
			this.drive.end(i);
			System.out.println("\tHUB ASSIST {V2}: TARGETING COMPLETE.");
		}
		@Override public boolean isFinished() {
			return false;
		}


	}


}