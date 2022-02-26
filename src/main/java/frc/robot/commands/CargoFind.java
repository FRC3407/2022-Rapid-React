package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RapidReactVision;
import frc.robot.modules.common.drive.*;
import frc.robot.modules.vision.java.VisionServer;


public class CargoFind extends DriveBase.DriveCommandBase {
	
	private boolean failed = false;
	private final Alliance team;
	private final String camera;

	public CargoFind(DriveBase db) { this(db, DriverStation.getAlliance(), null); }
	public CargoFind(DriveBase db, Alliance a) { this(db, a, null); }
	public CargoFind(DriveBase db, String cam_name) { this(db, DriverStation.getAlliance(), cam_name); }
	public CargoFind(DriveBase db, Alliance a, String cam_name) {
		super(db);
		this.team = a;
		this.camera = cam_name;
	}

	@Override public void initialize() {
		RapidReactVision.setCargoPipelineScaling(4);
		VisionServer.Get().applyCameraPreset(Constants.cam_cargo_pipeline);
		if(this.camera != null) {
			VisionServer.Get().setCamera(this.camera);
		}
		if(!RapidReactVision.verifyCargoPipelineActive()) {
			System.out.println("CargoFind: Failed to set Cargo pipeline");
			this.failed = true;
			return;
		}
		System.out.println("CargoFind: Running...");
	}
	@Override public void execute() {
		super.autoTurn(Constants.auto_max_turn_speed / 2.0);
	}
	@Override public void end(boolean i) {
		System.out.println(this.failed || i ? "CargoFind: Terminated." : "CargoFind: Completed.");
	}
	@Override public boolean isFinished() {
		return this.failed || RapidReactVision.getClosestAllianceCargo(this.team) != null;
	}


}