package frc.robot;

import java.nio.file.Path;

import frc.robot.modules.common.drive.Types.*;
import frc.robot.modules.common.drive.*;
import frc.robot.modules.vision.java.VisionServer;
import frc.robot.modules.vision.java.VisionServer.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;


public final class Constants {

    //       				 descriptions >>  {fl fr bl br} {MotorController Instantiation}  {inversion settings}  {Drive layout}
    public static final DriveMap_4<PWMVictorSPX> 
        drivebase_map_testbot = new DriveMap_4<>(8, 6, 7, 5, Motors.pwm_victorspx, Inversions.RIGHT, DriveLayout.DIFFERENTIAL)/*,
        //drivebase_map_2019 = new DriveMap_4<>(6, 8, 9, 7, Motors.pwm_victorsp, Inversions.RIGHT, DriveLayout.DIFFERENTIAL)*/;
	public static final DriveMap_4<WPI_TalonSRX>
		drivebase_map_2022 = new DriveMap_4<>(0, 3, 1, 2, Motors.can_talonsrx, Inversions.RIGHT, DriveLayout.DIFFERENTIAL);

	public static final int 
		intake_port = 1,		// port for intake motor
		feed_port = 3,			// port for transfer system feed motor
		shooter_canid = 4,		// port for shooter motor (can id for falcon)
		w0_shooter_port = 0,
		lim_entering_dio = 0,
		lim_exiting_dio = 1;
	public static final int[]
		transfer_ports = {2};	// ports for all additional motors in transfer system (all controlled together)


	public static final boolean 
		teleop_drivebase_speed_squaring = false;

	public static final double 
// Vision
		heading_offset_thresh = 0.5,	// threshold for targeting angle
		max_heading_offset = 22.0,		// maximum angle range that a target could be offset (used as divisor for P-loop)
		cargo_thresh = 20.0,			// threshold distance for a cargo to be considered close enough to influenced by intake
		cargo_distance_range = 100.0,	// maximum range that a cargo could be (and detected - used as divisor for P-loop)

// motorcontroller units (-1.0 to 1.0) or "percent output"
		uncertainty_continuation_percentage = 0.95,	// if break in target detection, keep powering motors at this percent of the last values used
		motors_thresh_tozero = 0.1,		// the point where it is safe to go straight to zero (deceleration)

		auto_max_turn_voltage = 2.0,	// maximum voltage for turning in place during auto
		auto_max_forward_voltage = 4.0,	// maximum voltage for driving forward during auto
		auto_max_voltage_ramp = 20.0,	// maximum voltage ramp (acceleration) in voltage/sec^2

		teleop_assist_turn_voltage = 3.5,	// in volts
		teleop_drivebase_scaling = -0.5,	// cap the voltage @50% -> negative because joysticks default to being inverted
		teleop_drivebase_deadband = 0.05,	// the input range that is discarded
		teleop_max_input_ramp = 2.0,		// maximum input acceleration in percent/sec^2

		intake_speed = 0.65,
		transfer_speed = 0.6,
		feed_speed = 0.4,
		shooter_default_speed = 0.85,

		// shooter_velocity = 8.0,				// in meters per second
		// shooter_speed_tollerance = 0.1,		// in meters per second

// Physical properties
		drivetrack_width_inches = 21.819,		// track width of drivebase
		drivewheel_diameter_inches = 6.0,	// wheel diameter for drivebase
		shooter_diameter_inches = 8.0,		// diameter of shooter wheel

// DriveBase closed-loop params
		ramsete_B = 2.0,
		ramsete_Zeta = 0.7,					// constants for ramsete command -> recommended in WPILib docs

		srx_mag_units_per_revolution = 4096,	// self explainatory (12 bit precision)
		falcon_units_per_revolution = 2048;		// self explainatory (11 bit precision)


	public static final Inversions
		cl_encoder_inversions = Inversions.LEFT;
	public static final ClosedLoopDifferentialDrive.CLDriveParams
		cl_params = new ClosedLoopDifferentialDrive.CLDriveParams(
			Units.inchesToMeters(drivetrack_width_inches),		// drivebase track width in meters
			Units.inchesToMeters(drivewheel_diameter_inches),	// drivebase wheel diameter in meters
			1.1185,	// "kS"(volts) -> base voltage required to overcome static friction -> from SysID characterization
			2.1132,	// "kV"(volts * seconds / meters) -> voltage required for each additional meter/second of velocity -> from SysID characterization
			1.0668,	// "kA"(volts * seconds^2 / meters) -> voltage required for each additional meter/second^2 of acceleration -> from SysID characterization
			3.5176,	// "kP"(volts * seconds / meters) -> voltage added to correct for error
			10.0,	// (volts) maximum voltage that can be supplied
			1.5,	// (meters per second) maximum velocity in meters per second
			2.0		// (meters per second^2) maximum acceleration in meters per second squared
		);




// Vision stuff
	public static final int
		cargo_pipeline_scaling = 4
	;
	public static final CameraPreset
		driving_camera_preset = new CameraPreset(50, -1, -1),
		hub_camera_preset = new CameraPreset(50, 10, 3500),
		cargo_camera_preset = new CameraPreset(50, 25, 3500)
	;
	public static final Runnable
		vision_driving = ()->{
			VisionServer.setProcessingEnabled(false);
			VisionServer.setStatistics(false);
			VisionServer.applyCameraPreset(driving_camera_preset);
		},
		vision_cargo = ()->{
			VisionServer.setProcessingEnabled(true);
			VisionServer.setStatistics(true);
			VisionServer.applyCameraPreset(cargo_camera_preset);
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
		test_straight1m = Filesystem.getDeployDirectory().toPath().resolve("./pathweaver/json/Straight-1m.wpilib.json")/*,
		test_arc90R = Filesystem.getDeployDirectory().toPath().resolve("./pathweaver/json/Arc-90dd(r)-0.5m.wpilib.json"),
		test_arc180L = Filesystem.getDeployDirectory().toPath().resolve("./pathweaver/json/Arc-180d(l)-0.5m.wpilib.json"),
		test_arc360R = Filesystem.getDeployDirectory().toPath().resolve("./pathweaver/json/Arc-360d(R)-1.wpilib.json"),
		test_diag45R = Filesystem.getDeployDirectory().toPath().resolve("./pathweaver/json/Diag-45d(R)-0.5m.wpilib.json")*/;

}