package frc.robot.commands;

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
		VisionServer.Get().applyCameraPreset(Constants.cam_hub_pipeline);
		if(this.camera != null) {
			VisionServer.Get().setCamera(this.camera);
		}
		if(!RapidReactVision.verifyHubPipelineActive()) {
			System.out.println("HubFind: Failed to set UpperHub pipeline");
			this.failed = true;
			return;
		}
		System.out.println("HubFind: Running...");
	}
	@Override public void execute() {
		super.autoTurn(Constants.auto_max_turn_speed * 0.75);
	}
	@Override public void end(boolean i) {
		System.out.println(this.failed || i ? "HubFind: Terminated." : "HubFind: Completed.");
	}
	@Override public boolean isFinished() {
		return this.failed || RapidReactVision.getHubPosition() != null;	// initializing pipeline failed or the hub is visible
	}


}