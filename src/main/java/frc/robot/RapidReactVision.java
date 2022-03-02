package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.modules.vision.java.VisionServer;

public class RapidReactVision {

	private static VisionServer.VsPipeline 
		upperhub = VisionServer.getPipeline("Upper-Hub Pipeline"), 
		cargo = VisionServer.getPipeline("Cargo Pipeline");

	public static boolean hasHubPipeline() { return VisionServer.getPipeline("Upper-Hub Pipeline") != null; }
	public static boolean isHubPipelineValid() { return upperhub != null; }
	public static boolean hasCargoPipeline() { return VisionServer.getPipeline("Cargo Pipeline") != null; }
	public static boolean isCargoPipelineValid() { return cargo != null; }
	public static boolean hasPipelines() { return hasHubPipeline() && hasCargoPipeline(); }
	public static boolean arePipelinesValid() { return isHubPipelineValid() && isCargoPipelineValid(); }

	public static boolean verifyHubPipeline() {
		if(!isHubPipelineValid()) {
			//System.out.println("VerifyHubPipeline: pipeline not valid, updating...");
			upperhub = VisionServer.getPipeline("Upper-Hub Pipeline");
			return isHubPipelineValid();
		}
		//System.out.println("VerifyHubPipeline: pipeline already valid");
		return true;
	}
	public static boolean verifyCargoPipeline() {
		if(!isCargoPipelineValid()) {
			cargo = VisionServer.getPipeline("Cargo Pipeline");
			return isCargoPipelineValid();
		}
		return true;
	}
	public static boolean updatePipelines() {
		return verifyHubPipeline() || verifyCargoPipeline();
	}

	// public static void applySettingsListener() {
	// 	int p_handle = 0;
	// 	final int c_handle = VisionServer.cameras.addSubTableListener(
	// 		(parent, name, table)->{
	// 			VisionServer.updateCameras();
	// 			VisionServer.getCamera(name).setExposure(20);
	// 			VisionServer.getCamera(name).setWhiteBalance(3000);
	// 			VisionServer.cameras.removeTableListener(c_handle);
	// 		}, false
	// 	);
	// 	VisionServer.pipelines.addSubTableListener(
	// 		(parent, name, table)->{
	// 			VisionServer.updatePipelines();
	// 		}, false
	// 	);
	// }

// ADD NULLPTR SAFETY TO ALL OF THESE? ->>

	public static boolean setHubPipelineScaling(int downscale) {	// returns false on failure
		if(verifyHubPipeline()) {
			return upperhub.get().getEntry("Scaling").setDouble((double)downscale);
		}
		return false;
	}
	public static boolean showHubPipelineThreshold(boolean show) {	// returns false on failure
		if(verifyHubPipeline()) {
			return upperhub.get().getEntry("Show Thresholed").setBoolean(show);
		}
		return false;
	}
	public static boolean setCargoPipelineScaling(int downscale) {	// returns false on failure
		if(verifyCargoPipeline()) {
			return cargo.get().getEntry("Scaling").setDouble((double)downscale);
		}
		return false;
	}
	public static boolean showCargoPipelineThreshold(boolean show) {	// returns false on failure
		if(verifyCargoPipeline()) {
			return cargo.get().getEntry("Show Threshold").setBoolean(show);
		}
		return false;
	}
	public static boolean showCargoPipelineContours(boolean show) {		// returns false on failure
		if(verifyCargoPipeline()) {
			return cargo.get().getEntry("Show Contours").setBoolean(show);
		}
		return false;
	}

	public static boolean isHubPipelineActive() {
		if(verifyHubPipeline()) {
			return upperhub == VisionServer.getCurrentPipeline();
		}
		return false;
	}
	public static boolean isCargoPipelineActive() {
		if(verifyCargoPipeline()) {
			return cargo == VisionServer.getCurrentPipeline();
		}
		return false;
	}
	public static boolean setHubPipelineActive() {
		if(verifyHubPipeline()) {
			//System.out.println("SetHubPipelineActive: pipeline exists, attempting to set");
			return VisionServer.setPipeline(upperhub.getIdx());
		}
		//System.out.println("SetHubPipelineActive: pipeline nonexistant");
		return false;
	}
	public static boolean setCargoPipelineActive() {
		if(verifyCargoPipeline()) {
			//System.out.println("SetCargoPipelineActive: pipeline exists, attempting to set");
			return VisionServer.setPipeline(cargo.getIdx());
		}
		//System.out.println("SetCargoPipelineActive: pipeline nonexistant");
		return false;
	}
	public static boolean verifyHubPipelineActive() {
		if(!isHubPipelineActive()) {
			//System.out.println("VerifyHubPipelineActive: not active");
			return setHubPipelineActive();
		}
		//System.out.println("VerifyHubPipelineActive: already active");
		return true;
	}
	public static boolean verifyCargoPipelineActive() {
		if(!isCargoPipelineActive()) {
			//System.out.println("VerifyCargoPipelineActive: not active");
			return setCargoPipelineActive();
		}
		//System.out.println("VerifyCargoPipelineActive: already active");
		return true;
	}

	private static boolean isRedCargoEnabled() {
		if(verifyCargoPipeline()) {
			return cargo.get().getEntry("Process Red").getBoolean(true);
		}
		return false;
	}
	private static boolean setRedCargoEnabled(boolean enable) {		// returns false on failure
		if(verifyCargoPipeline()) {
			return cargo.get().getEntry("Process Red").setBoolean(enable);
		}
		return false;
	}
	private static boolean verifyRedCargo(boolean status) {
		if(verifyCargoPipeline()) {
			if(cargo.get().getEntry("Process Red").getBoolean(status) != status) {
				return cargo.get().getEntry("Process Red").setBoolean(status);
			}
			return true;
		}
		return false;
	}
	private static boolean isBlueCargoEnabled() {
		if(verifyCargoPipeline()) {
			return cargo.get().getEntry("Process Blue").getBoolean(true);
		}
		return false;
	}
	private static boolean setBlueCargoEnabled(boolean enable) {		// returns false on failure
		if(verifyCargoPipeline()) {
			return cargo.get().getEntry("Process Blue").setBoolean(enable);
		}
		return false;
	}
	private static boolean verifyBlueCargo(boolean status) {
		if(verifyCargoPipeline()) {
			if(cargo.get().getEntry("Process Blue").getBoolean(status) != status) {
				return cargo.get().getEntry("Process Blue").setBoolean(status);
			}
			return true;
		}
		return false;
	}
	public static boolean isCargoAllianceColorMode(DriverStation.Alliance a) {
		switch(a) {
			case Red:
				return isRedCargoEnabled() && !isBlueCargoEnabled();
			case Blue:
				return !isRedCargoEnabled() && isBlueCargoEnabled();
			case Invalid:
			default:
				return (isRedCargoEnabled() && isBlueCargoEnabled()) || (!isRedCargoEnabled() && !isBlueCargoEnabled());
		}
	}
	public static boolean setCargoAllianceColorMode(DriverStation.Alliance a) {	// returns false on failure
		switch(a) {
			case Red: 
				return setRedCargoEnabled(true) && setBlueCargoEnabled(false);
			case Blue:
				return setRedCargoEnabled(false) && setBlueCargoEnabled(true);
			case Invalid:
			default:
				return setRedCargoEnabled(true) && setBlueCargoEnabled(true);
		}
	}
	public static boolean verifyCargoAllianceColorMode(DriverStation.Alliance a) {
		verifyCargoPipelineActive();
		switch(a) {
			case Red:
				return (verifyRedCargo(true) && verifyBlueCargo(false)); 
			case Blue:
				return (verifyRedCargo(false) && verifyBlueCargo(true));
			case Invalid:
			default:
				return (verifyRedCargo(true) && verifyBlueCargo(true)) || (verifyRedCargo(false) && verifyBlueCargo(false));
		}
	}
	public static boolean activeTargetMatchesAlliance(DriverStation.Alliance a) {
		switch(a) {
			case Red:
				return VisionServer.getActiveTargetName().equals("Cargo-1r");
			case Blue:
				return VisionServer.getActiveTargetName().equals("Cargo-1b");
			case Invalid:
			default:
				return VisionServer.getActiveTargetName().equals("none");
		}
	}
	public static boolean disableCargoProcessing() {		// returns false on failure
		return setRedCargoEnabled(false) || setBlueCargoEnabled(false);
	}

// MAIN METHODS OF USE -> all return null on incorrect target or other error

	public static VisionServer.TargetData getHubPosition() {	// returns null on incorrect target
		verifyHubPipelineActive();
		return VisionServer.getTargetDataIfMatching("Upper-Hub");
	}
	public static boolean isHubDetected() {
		return getHubPosition() != null;
	}

	public static VisionServer.TargetData getClosestAllianceCargo(DriverStation.Alliance a) {	// returns null on incorrect target
		verifyCargoPipelineActive();
		switch(a) {
			case Red:
				if(verifyRedCargo(true) && verifyBlueCargo(false)) {
					return VisionServer.getTargetDataIfMatching("Cargo-1r");
				}
			case Blue:
				if(verifyRedCargo(false) && verifyBlueCargo(true)) {
					return VisionServer.getTargetDataIfMatching("Cargo-1b");
				}
			case Invalid:
			default:
				return VisionServer.getTargetData();
		}
	}
	public static VisionServer.TargetData getClosestRedPosition() {		// returns null on incorrect target
		return getClosestAllianceCargo(DriverStation.Alliance.Red);
	}
	public static VisionServer.TargetData getClosestBluePosition() {	// returns null on incorrect target
		return getClosestAllianceCargo(DriverStation.Alliance.Blue);
	}
	public static boolean isAllianceCargoDetected(DriverStation.Alliance a) {
		return getClosestAllianceCargo(a) != null;
	}



	public static double getHubBaseDistance(VisionServer.TargetData d, double height) {		// output in inches
		return Math.sqrt(Math.pow(d.distance, 2) - Math.pow(height, 2));	// pythagorean solved for A (sqrt(C^2 - B^2) = A)
	}


}