package frc.robot.commands;

import frc.robot.modules.common.DriveBase;
import frc.robot.modules.vision.java.VisionServer;
import frc.robot.Constants;
import frc.robot.RapidReactVision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class CargoTurn extends DriveBase.DriveCommandBase {

	private final Alliance team;
	private VisionServer.TargetData position;

	public CargoTurn(DriveBase db, Alliance a) {
		super(db);
		this.team = a;
	}

	@Override public void initialize() {
		// set correct camera
		RapidReactVision.setCargoPipelineScaling(4);
		VisionServer.Get().getCurrentCamera().applyPreset(Constants.cam_cargo_pipeline);
		if(!RapidReactVision.verifyCargoPipelineActive()) {
			System.out.println("CargoTurn: Failed to set Cargo pipeline");
			this.cancel();
			return;
		}
		System.out.println("CargoTurn: Running...");
	}
	@Override public void execute() {
		this.position = RapidReactVision.getClosestAllianceCargo(this.team);
		if(this.position != null) {
			//System.out.println("Turning???");
			//System.out.println(String.valueOf(this.position.lr));
			super.autoTurn((this.position.lr / Constants.target_angle_range_lr) * Constants.auto_max_turn_speed);
		} else {
			super.fromLast(Constants.uncertainty_continuation_percentage);	// % of what was last set (decelerating)
			//super.autoDrive(0, 0);
			//System.out.println("CargoTurn: Idling...");
		}
	}
	@Override public void end(boolean i) {
		System.out.println("CargoTurn: Completed.");
	}
	@Override public boolean isFinished() {	// change threshold angle when testing >>
		if(this.position != null) {
			return Math.abs(this.position.lr) < Constants.turn_thresh;
		}
		return false;
	}



	/** Extension of CargoTurn class that runs indefinately */
	public static class Demo extends CargoTurn {

		public Demo(DriveBase db, Alliance a) { super(db, a); }

		@Override public boolean isFinished() { return false; }

	}


}