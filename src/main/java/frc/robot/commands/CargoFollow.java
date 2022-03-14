// package frc.robot.commands;

// import frc.robot.modules.common.drive.DriveBase;
// import frc.robot.modules.vision.java.VisionServer;
// import frc.robot.*;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;


// public class CargoFollow extends DriveBase.RateLimitedAutoDrive {

// 	public static final double	// these don't need to be parameters
// 		target_range = Constants.cargo_thresh,
// 		max_range = Constants.cargo_distance_range,
// 		angle_range = Constants.target_angle_range_lr,

// 		static_voltage = Constants.cl_params.static_voltage,
// 		continuation_percent = Constants.uncertainty_continuation_percentage,

// 		default_max_forward_voltage = Constants.auto_max_forward_speed * 10,
// 		default_max_turning_voltage = Constants.auto_max_turn_speed * 10;


// 	private final double
// 		max_forward_voltage,
// 		max_turning_voltage;
// 	private final Alliance team;
// 	private final String camera;
// 	private boolean failed = false;
// 	private VisionServer.TargetData position;

// 	public CargoFollow(DriveBase db, Alliance a) { this(db, a, null); }
// 	public CargoFollow(DriveBase db, Alliance a, String cam_name) {
// 		this(db, a, cam_name, default_max_forward_voltage, default_max_turning_voltage, Double.MAX_VALUE);
// 	}
// 	// 					drivebase,	alliance,	camera name, max forward voltage, max turn voltage, max voltage acceleration
// 	public CargoFollow(DriveBase db, Alliance a, String cam_name, double mfvolts, double mtvolts, double mvr) {
// 		super(db, mvr);
// 		this.team = a;
// 		this.camera = cam_name;
// 		this.max_forward_voltage = mfvolts;
// 		this.max_turning_voltage = mtvolts;
// 	}

// 	@Override public void initialize() {
// 		RapidReactVision.setCargoPipelineScaling(4);
// 		VisionServer.applyCameraPreset(Constants.cam_cargo_pipeline);
// 		if(this.camera != null) {
// 			VisionServer.setCamera(this.camera);
// 		}
// 		if(!RapidReactVision.verifyCargoPipelineActive()) {
// 			System.out.println("CargoFollow: Failed to set Cargo pipeline");
// 			this.failed = true;
// 			return;
// 		}
// 		System.out.println("CargoFollow: Running...");
// 	}
// 	@Override public void execute() {
// 		this.position = RapidReactVision.getClosestAllianceCargo(this.team);
// 		if(this.position != null) {
// 			double f = MathUtil.clamp((this.position.distance - target_range) / max_range, -1.0, 1.0) * 9 * Constants.auto_max_forward_speed;	// forward voltage
// 			double t = MathUtil.clamp(this.position.lr / angle_range, -1.0, 1.0) * 9 * Constants.auto_max_turn_speed;	// turning voltage
// 			super.autoDriveVoltage(Math.signum(f+t) + f + t, Math.signum(f-t) + f - t);	// add 1 volt to each side in the direction that they are going (static voltage)
// 		} else {
// 			super.fromLast(continuation_percent);	// handle jitters in vision detection -> worst case cenario this causes a gradual deceleration
// 		}
// 	}
// 	@Override public void end(boolean i) {
// 		System.out.println(this.failed || i ? "CargoFollow: Terminated." : "CargoFollow: Completed.");
// 	}
// 	@Override public boolean isFinished() {
// 		if(this.position != null) {
// 			return Math.abs(this.position.lr) < Constants.turn_thresh && Math.abs(this.position.distance) < Constants.cargo_thresh;
// 		}
// 		return this.failed;
// 	}



// 	/** Extension of CargoFollow that runs indefinately */
// 	public static class Demo extends CargoFollow {

// 		public Demo(DriveBase db, Alliance a) { super(db, a); }
// 		public Demo(DriveBase db, Alliance a, String cam_name) { super(db, a, cam_name); }

// 		@Override public boolean isFinished() { return super.failed; }

// 	}


// }