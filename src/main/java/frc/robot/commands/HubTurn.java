package frc.robot.commands;

import frc.robot.modules.common.Input.AnalogSupplier;
import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.vision.java.VisionServer;
import frc.robot.Constants;
import frc.robot.RapidReactVision;


public class HubTurn extends DriveBase.DriveCommandBase {

	private final String camera;
	private boolean failed = false;
	private VisionServer.TargetData position;

	public HubTurn(DriveBase db) { this(db, null); }
	public HubTurn(DriveBase db, String cam_name) {
		super(db);
		this.camera = cam_name;
	}

	@Override public void initialize() {
		VisionServer.Get().applyCameraPreset(Constants.cam_hub_pipeline);
		if(this.camera != null) {
			VisionServer.Get().setCamera(this.camera);
		}
		if(!RapidReactVision.verifyHubPipelineActive()) {
			System.out.println(getClass().getSimpleName() + ": Failed to set UpperHub pipeline");
			this.failed = true;
			return;
		}
		System.out.println(getClass().getSimpleName() + ": Running...");
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
		System.out.println(getClass().getSimpleName() + (this.failed || i ? ": Terminated." : ": Completed."));
	}
	@Override public boolean isFinished() {
		if(this.position != null) {
			return Math.abs(this.position.lr) < Constants.turn_thresh;
		}
		return this.failed;
	}



	public static class TeleopAssist extends HubTurn {

		private final AnalogSupplier turnvec;

		public TeleopAssist(DriveBase db, AnalogSupplier tv) {
			super(db, Constants.hub_cam_name);
			this.turnvec = tv;
		}

		@Override public void execute() {
			super.position = RapidReactVision.getHubPosition();
			if(super.position != null) {
				super.autoTurn((super.position.lr / Constants.target_angle_range_lr) * Constants.auto_max_turn_speed * this.turnvec.get());
			} else {
				super.fromLast(Constants.uncertainty_continuation_percentage);
			}
		}
		@Override public boolean isFinished() { return false; }

	}


}