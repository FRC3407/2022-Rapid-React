package frc.robot.commands;

import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.vision.java.VisionServer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public final class Test {

	public static class Print extends CommandBase {
		private final String identifier;
		public Print(String id) { this.identifier = id; }
	
		@Override public void initialize() { System.out.println("Command Init: " + this.identifier); }
		@Override public void execute() { System.out.println("\tCommand Exec: " + this.identifier); }
		@Override public void end(boolean i) { System.out.println("Command End(" + i + "): " + this.identifier); }
		@Override public boolean isFinished() { return true; }
		@Override public boolean runsWhenDisabled() { return true; }
	
	}

	public static class DriveCommand extends DriveBase.DriveCommandBase {
		private final String identifier;
		public DriveCommand(DriveBase db, String id) { 
			super(db);
			this.identifier = id; 
		}

		@Override public void initialize() { System.out.println("Command Init: " + this.identifier); }
		@Override public void execute() { System.out.println("\tCommand Exec: " + this.identifier); }
		@Override public void end(boolean i) { System.out.println("Command End(" + i + "): " + this.identifier); }
		@Override public boolean isFinished() { return true; }
		@Override public boolean runsWhenDisabled() { return true; }
	}

	public static class CameraPreset extends CommandBase {
		
		private final VisionServer.CameraPreset preset;
		public CameraPreset(VisionServer.CameraPreset p) {
			this.preset = p;
		}
		@Override public void initialize() { 
			VisionServer.Get().getCurrentCamera().applyPreset(this.preset); 
		}
		@Override public boolean isFinished() { return true; }
		@Override public boolean runsWhenDisabled() { return true; }
	}

	
}