package frc.robot.commands;

import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.vision.java.VisionServer;
import frc.robot.Constants;
import frc.robot.RapidReactVision;


public class HubTurn extends DriveBase.DriveCommandBase {

	private boolean failed = false;
	private VisionServer.TargetData position;

	public HubTurn(DriveBase db) {
		super(db);
	}

	@Override public void initialize() {
		VisionServer.Get().applyCameraPreset(Constants.cam_hub_pipeline);
		if(!RapidReactVision.verifyHubPipelineActive()) {
			System.out.println("HubTurn: Failed to set UpperHub pipeline");
			this.failed = true;
			return;
		}
		System.out.println("HubTurn: Running...");
	}
	@Override public void execute() {
		this.position = RapidReactVision.getHubPosition();
		if(this.position != null) {
			super.autoTurn((this.position.lr / Constants.target_angle_range_lr) * Constants.auto_max_turn_speed);
		} else {
			super.fromLast(Constants.uncertainty_continuation_percentage);
		}
	}
	@Override public void end(boolean i) {
		System.out.println(this.failed ? "HubTurn: Terminated." : "HubTurn: Completed.");
	}
	@Override public boolean isFinished() {
		if(this.position != null) {
			return Math.abs(this.position.lr) < Constants.turn_thresh;
		}
		return this.failed;
	}


}