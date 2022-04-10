package frc.robot;

import java.nio.file.Path;

import frc.robot.modules.common.drive.*;
import frc.robot.modules.common.drive.Types.*;
import frc.robot.modules.vision.java.VisionServer;
import frc.robot.modules.vision.java.VisionServer.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public final class Constants {

    //       				descriptions >>  {fl fr bl br} {MotorController Type}  {inversion settings}  {Drive layout}
	public static final DriveMap_4<WPI_TalonSRX>
		drivebase_map_2022 = new DriveMap_4<>(0, 3, 1, 2, Motors.can_talonsrx, Inversions.RIGHT, DriveLayout.DIFFERENTIAL)		// competition bot
	;

	public static final int
	// pwm ports
		intake_port = 1,
		shooter_port = 0,
		climber_left_port = 3,
		climber_right_port = 4,
	// dio ports
		input_entering_dio = 8,
		input_exiting_dio = 9
	;
	public static final int[]
		transfer_ports = {2}	// ports for all additional motors in transfer system (all controlled together)
	;



	public static final boolean 
		teleop_drivebase_speed_squaring = false		// to square or not to square?
	;

	public static final double 
// Vision -> inches and degrees
		heading_offset_thresh = 0.5,	// threshold for targeting angle
		max_heading_offset = 22.0,		// maximum angle range that a target could be offset (used as divisor for P-loop)
		cargo_thresh = 20.0,			// threshold distance for a cargo to be considered close enough to influenced by intake
		max_cargo_range_inches = 100.0,		// maximum range that a cargo could be (and detected - used as divisor for P-loop)
		cargo_follow_target_inches = 5.0,	// the distance away from a cargo that cargo-following commands will target
		max_hub_range_inches = 215.0,		// maximum range that the hub can be detected from (used as divisor for P-loop)
		min_hub_range_inches = 170.0,		// minimum range for shooting into the hub

// "percent output" (1.0 as 100%)
		uncertainty_continuation_percentage = 0.95,	// if break in target detection, keep powering motors at this percent of the last values used

		teleop_drivebase_scaling = -0.7,	// cap the voltage @70% -> negative because joysticks default to being inverted
		teleop_drivebase_deadband = 0.05,	// the input range that is discarded
		teleop_max_input_ramp = 2.0,		// maximum input acceleration in percent/sec^2 -> no more than 200% per second per second

// voltage
		universal_max_voltage = 10.0,

		auto_max_turn_voltage = 2.5,	// maximum voltage for turning in place during auto
		auto_max_forward_voltage = 4.5,	// maximum voltage for driving forward during auto
		auto_max_voltage_ramp = 20.0,	// maximum voltage ramp (acceleration) in voltage/sec^2

		teleop_assist_turn_voltage = 3.0,	// maximum turn voltage for finding the hub


		turning_static_voltage = 1.586,

		intake_voltage = 8,
		intake_deploy_voltage = 5,	intake_deploy_time = 0.75,
		transfer_voltage = 3,
		feed_voltage = 4.5,
		//shooter_static_voltage = 0.8,	// CURRENTLY JUST AN ESTIMATE
		shooter_max_voltage = 11.25,
		shooter_min_voltage = 10.75,
		climber_extend_voltage = 8,
		climber_hold_ext_voltage = 1,
		climber_retract_voltage = 10,
		climber_hold_ret_voltage = 2,

// Physical properties
		drivetrack_width_inches = 21.819,	// track width of drivebase
		drivewheel_diameter_inches = 6.0,	// wheel diameter for drivebase
		shooter_diameter_inches = 8.0,		// diameter of shooter wheel

// DriveBase closed-loop params
		ramsete_B = 2.0,
		ramsete_Zeta = 0.7,					// constants for ramsete command -> recommended in WPILib docs

		srx_mag_units_per_revolution = 4096,	// self explainatory (12 bit precision)
		falcon_units_per_revolution = 2048		// self explainatory (11 bit precision)
	;

	public static final Inversions
		cl_encoder_inversions = Inversions.BOTH;	// which encoders should be inverted (aka "phase-changed" in the phoenix api)
	public static final ClosedLoopDifferentialDrive.CLDriveParams
		cl_params = new ClosedLoopDifferentialDrive.CLDriveParams(
			//Units.inchesToMeters(drivetrack_width_inches),		// drivebase track width in meters
			1.3553,	// empirical value from characterization -> seems very high?
			Units.inchesToMeters(drivewheel_diameter_inches),	// drivebase wheel diameter in meters
			1.1185,	// "kS"(volts) -> base voltage required to overcome static friction -> from SysID characterization
			2.1132,	// "kV"(volts * seconds / meters) -> voltage required for each additional meter/second of velocity -> from SysID characterization
			1.0668,	// "kA"(volts * seconds^2 / meters) -> voltage required for each additional meter/second^2 of acceleration -> from SysID characterization
			3.5176,	// "kP"(volts * seconds / meters) -> voltage added to correct for error
			universal_max_voltage,	// (volts) maximum voltage that can be supplied
			1.5,	// (meters per second) maximum velocity in meters per second
			1.5		// (meters per second^2) maximum acceleration in meters per second squared
		);


	public static final VisionServer.Conversion
		inches2volts_shooter = (double in)->
			shooter_min_voltage + (	(
					(in - min_hub_range_inches) / (max_hub_range_inches - min_hub_range_inches)
				) * (shooter_max_voltage - shooter_min_voltage)
			);



// Vision presets
	public static final int
		cargo_pipeline_scaling = 4	// the cargo pipeline lags pretty bad without downscaling -> full resolution actually doesn't help the detection either
	;
	public static final CameraPreset
		driving_camera_preset = new CameraPreset(50, 40, 3500),
		hub_camera_preset = new CameraPreset(50, 10, 3500),
		cargo_blue_camera_preset = new CameraPreset(50, 30, 4000),
		cargo_red_camera_preset = new CameraPreset(50, 15, 3500)
	;
	public static final Runnable	// these functions contain all configs that should be applied before each vision usecase
		vision_driving = ()->{
			VisionServer.setProcessingEnabled(false);
			VisionServer.setStatistics(false);
			VisionServer.applyCameraPreset(driving_camera_preset);
		},
		vision_cargo = ()->{
			VisionServer.setProcessingEnabled(true);
			VisionServer.setStatistics(true);
			VisionServer.applyCameraPreset(
				DriverStation.getAlliance() == Alliance.Blue ? cargo_blue_camera_preset : cargo_red_camera_preset
			);
			RapidReactVision.setCargoPipelineScaling(cargo_pipeline_scaling);
			RapidReactVision.Cameras.CARGO.setActive();
		},
		vision_hub = ()->{
			VisionServer.setProcessingEnabled(true);
			VisionServer.setStatistics(true);
			VisionServer.applyCameraPreset(hub_camera_preset);
			RapidReactVision.Cameras.HUB.setActive();
		}
	;


// PathWeaver filesystem locations / Generated paths
	public static final Path
		test_straight1m = Filesystem.getDeployDirectory().toPath().resolve("paths/output/Straight-1m.wpilib.json"),
		test_arc90R = Filesystem.getDeployDirectory().toPath().resolve("paths/output/Arc-90d(r)-0.5m.wpilib.json"),
		test_arc180L = Filesystem.getDeployDirectory().toPath().resolve("paths/output/Arc-180d(l)-0.5m.wpilib.json"),
		test_arc360R = Filesystem.getDeployDirectory().toPath().resolve("paths/output/Arc-360d(R)-1m.wpilib.json"),
		test_zigzag = Filesystem.getDeployDirectory().toPath().resolve("paths/output/ZigZag-4m.wpilib.json");


	public static enum StartingPose {	// values sourced from pathweaver and "eyeballing" -> definately are not super accureate
		B1	(new Pose2d(6.7, 5.5, new Rotation2d(-1.293, 0.851))),
		B2	(new Pose2d(6.2, 4.1, new Rotation2d(-1.595, 0.334))),
		B3	(new Pose2d(6.8, 2.5, new Rotation2d(-0.701, -0.959))),
		B4	(new Pose2d(8.4, 2, new Rotation2d(-0.313, -1.239))),
		R1	(new Pose2d(9.9, 2.8, new Rotation2d(0.884, -0.992))),
		R2	(new Pose2d(10.2, 4, new Rotation2d(1.433, 0.097))),
		R3	(new Pose2d(9.5, 5.5, new Rotation2d(1.164, 0.808))),
		R4	(new Pose2d(8.5, 6, new Rotation2d(0.279, 1.388))),
		ORG	(new Pose2d());	// field origin, for testing purposes

		public final Pose2d pose;
		private StartingPose(Pose2d p) {
			this.pose = p;
		}

		public static void addOptions(SendableChooser<StartingPose> s, Alliance a) {
			s.setDefaultOption("Origin", ORG);
			if(a == Alliance.Red || a == Alliance.Invalid) {
				s.addOption("Red 1", R1);
				s.addOption("Red 2", R2);
				s.addOption("Red 3", R3);
				s.addOption("Red 4", R4);
			}
			if(a == Alliance.Blue || a == Alliance.Invalid) {
				s.addOption("Blue 1", B1);
				s.addOption("Blue 2", B2);
				s.addOption("Blue 3", B3);
				s.addOption("Blue 4", B4);
			}
		}
		public static SendableChooser<StartingPose> getSelectable(Alliance a) {
			SendableChooser<StartingPose> s = new SendableChooser<StartingPose>();
			addOptions(s, a);
			return s;
		}
	}


}