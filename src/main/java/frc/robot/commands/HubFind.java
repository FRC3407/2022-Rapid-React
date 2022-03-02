package frc.robot.commands;

import frc.robot.modules.common.Input.AnalogSupplier;
import frc.robot.modules.common.drive.*;
import frc.robot.modules.vision.java.VisionServer;
import frc.robot.Constants;
import frc.robot.RapidReactVision;


public class HubFind extends DriveBase.DriveCommandBase {

	private final String camera;
	private boolean failed = false;

	public HubFind(DriveBase db) { this(db, null); }
	public HubFind(DriveBase db, String cam_name) {
		super(db);
		this.camera = cam_name;
	}

	@Override public void initialize() {
		VisionServer.applyCameraPreset(Constants.cam_hub_pipeline);
		if(this.camera != null) {
			VisionServer.setCamera(this.camera);
		}
		if(!RapidReactVision.verifyHubPipelineActive()) {
			System.out.println(getClass().getSimpleName() + ": Failed to set UpperHub pipeline.");
			this.failed = true;
			return;
		}
		System.out.println(getClass().getSimpleName() + ": Running...");
	}
	@Override public void execute() {
		super.autoTurn(Constants.auto_max_turn_speed * 0.75);
	}
	@Override public void end(boolean i) {
		System.out.println(getClass().getSimpleName() + (this.failed || i ? ": Terminated." : ": Completed."));
	}
	@Override public boolean isFinished() {
		return this.failed || RapidReactVision.isHubDetected();		// initializing pipeline failed or the hub is visible
	}



	public static class TeleopAssist extends HubFind {

		private final AnalogSupplier turnvec;

		public TeleopAssist(DriveBase db, AnalogSupplier tv) {
			super(db, Constants.hub_cam_name);
			this.turnvec = tv;
		}

		@Override public void execute() {
			super.autoTurn(Constants.auto_max_turn_speed * this.turnvec.get());
		}


	}


}