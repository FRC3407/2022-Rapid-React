package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RapidReactVision;
import frc.robot.modules.common.drive.*;
import frc.robot.modules.vision.java.VisionServer;


public class CargoFind extends DriveBase.DriveCommandBase {
	
	private boolean failed = false;
	private final Alliance team;
	public CargoFind(DriveBase db, Alliance a) {
		super(db);
		this.team = a;
	}

	@Override public void initialize() {
		RapidReactVision.setCargoPipelineScaling(4);
		VisionServer.Get().applyCameraPreset(Constants.cam_cargo_pipeline);
		if(!RapidReactVision.verifyCargoPipelineActive()) {
			System.out.println("CargoFind: Failed to set Cargo pipeline");
			this.failed = true;
			return;
		}
		System.out.println("CargoFind: Running...");
	}
	@Override public void execute() {
		super.autoTurn(Constants.auto_max_turn_speed/2.0);
	}
	@Override public void end(boolean i) {
		System.out.println(this.failed || i ? "CargoFind: Terminated." : "CargoFind: Completed.");
	}
	@Override public boolean isFinished() {
		return this.failed || RapidReactVision.getClosestAllianceCargo(this.team) != null;
	}


}