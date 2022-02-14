package frc.robot.commands;

import frc.robot.modules.vision.java.VisionServer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SetCameras extends CommandBase {

	private final VisionServer.CameraPreset preset;
	public SetCameras(VisionServer.CameraPreset p) {
		this.preset = p;
	}

	@Override public void initialize() {
		for(VisionServer.VsCamera camera : VisionServer.Get().getCameras()) {
			camera.applyPreset(this.preset);
		}
	}
	@Override public boolean isFinished() { return true; }


}