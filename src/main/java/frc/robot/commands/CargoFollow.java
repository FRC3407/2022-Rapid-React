package frc.robot.commands;

import frc.robot.modules.common.DriveBase;
import frc.robot.modules.vision.java.VisionServer;
import frc.robot.Constants;
import frc.robot.RapidReactVision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class CargoFollow extends DriveBase.DriveCommandBase {

	private final Alliance team;
	private VisionServer.TargetData position;

	public CargoFollow(DriveBase db, Alliance a) {
		super(db);
		this.team = a;
	}

	@Override public void initialize() {
		// set correct camera
		RapidReactVision.setCargoPipelineScaling(4);
		VisionServer.Get().getCurrentCamera().applyPreset(Constants.cam_cargo_pipeline);
		if(!RapidReactVision.verifyCargoPipelineActive()) {
			System.out.println("CargoFollow: Failed to set Cargo pipeline");
			this.cancel();
			return;
		}
		System.out.println("CargoFollow: Running...");
	}
	@Override public void execute() {
		this.position = RapidReactVision.getClosestAllianceCargo(this.team);
		if(this.position != null) {
			double f = (this.position.distance - Constants.cargo_thresh) / Constants.cargo_distance_range * Constants.auto_max_forward_speed;	// forward speed
			double t = this.position.lr / Constants.target_angle_range_lr * Constants.auto_max_turn_speed;	// turning offset
			super.autoDrive(f + t, f - t);	// probably need to test this
		} else {
			super.fromLast(Constants.uncertainty_continuation_percentage);	// 75% of what was last set (decelerating)
			//super.autoDrive(0, 0);
			//System.out.println("CargoFollow: Idling...");
		}
	}
	@Override public void end(boolean i) {
		System.out.println("CargoFollow: Completed.");
	}
	@Override public boolean isFinished() {
		if(this.position != null) {
			return Math.abs(this.position.lr) < Constants.turn_thresh && Math.abs(this.position.distance) < Constants.cargo_thresh;
		}
		return false;
	}



	/** Extension of CargoFollow that runs indefinately */
	public static class Demo extends CargoFollow {

		public Demo(DriveBase db, Alliance a) { super(db, a); }

		@Override public boolean isFinished() { return false; }

	}


}