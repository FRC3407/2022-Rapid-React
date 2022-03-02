package frc.robot.commands;

import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.vision.java.VisionServer;
import frc.robot.Constants;
import frc.robot.RapidReactVision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class CargoTurn extends DriveBase.DriveCommandBase {

	private boolean failed = false;
	private final Alliance team;
	private final String camera;
	private VisionServer.TargetData position;

	public CargoTurn(DriveBase db, Alliance a) { this(db, a, null); }
	public CargoTurn(DriveBase db, Alliance a, String cam_name) {
		super(db);
		this.team = a;
		this.camera = cam_name;
	}

	@Override public void initialize() {
		RapidReactVision.setCargoPipelineScaling(4);
		VisionServer.applyCameraPreset(Constants.cam_cargo_pipeline);
		if(this.camera != null) {
			VisionServer.setCamera(this.camera);
		}
		if(!RapidReactVision.verifyCargoPipelineActive()) {
			System.out.println("CargoTurn: Failed to set Cargo pipeline");
			this.failed = true;
			return;
		}
		System.out.println("CargoTurn: Running...");
	}
	@Override public void execute() {
		this.position = RapidReactVision.getClosestAllianceCargo(this.team);
		if(this.position != null) {
			super.autoTurn((this.position.lr / Constants.target_angle_range_lr) * Constants.auto_max_turn_speed);
		} else {
			super.fromLast(Constants.uncertainty_continuation_percentage);	// % of what was last set (decelerating)
		}
	}
	@Override public void end(boolean i) {
		System.out.println(this.failed ? "CargoTurn: Terminated." : "CargoTurn: Completed.");
	}
	@Override public boolean isFinished() {	// change threshold angle when testing >>
		if(this.position != null) {
			return Math.abs(this.position.lr) < Constants.turn_thresh;
		}
		return this.failed;
	}



	/** Extension of CargoTurn class that runs indefinately */
	public static class Demo extends CargoTurn {

		public Demo(DriveBase db, Alliance a) { super(db, a); }
		public Demo(DriveBase db, Alliance a, String cam_name) { super(db, a, cam_name); }

		@Override public boolean isFinished() { return false; }

	}


}