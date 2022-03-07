package frc.robot;

import frc.robot.modules.common.drive.Types.*;
import frc.robot.modules.common.drive.*;
import frc.robot.modules.vision.java.VisionServer.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.math.util.Units;


public final class Constants {

    //        descriptions >>  {fl fr bl br} {MotorController Instantiation}  {inversion settings}  {Drive layout}
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
		turn_thresh = 1.0,				// threshold for targeting angle
		target_angle_range_lr = 20.0,	// maximum angle range that a target could be offset (used as divisor for P-loop)
		cargo_thresh = 20.0,			// threshold distance for a cargo to be considered close enough to influenced by intake
		cargo_distance_range = 100.0,	// maximum range that a cargo could be (and detected - used as divisor for P-loop)

// motorcontroller units (-1.0 to 1.0) or "percent output"
		uncertainty_continuation_percentage = 0.95,	// if break in target detection, keep powering motors at this percent of the last values used
		motors_thresh_tozero = 0.1,		// the point where it is safe to go straight to zero (deceleration)

		auto_max_turn_speed = 0.2,		// maximum speed when turning in place during auto
		auto_max_forward_speed = 0.4,	// maximum speed when driving forward during auto
		auto_max_acceleration = 2.5,	// maximum acceleration in percent output/sec^2

		teleop_assist_turn_speed = 0.4,
		teleop_drivebase_scaling = -0.5,
		teleop_drivebase_deadband = 0.05,
		teleop_max_acceleration = 2.0,		// maximum acceleration in percent output/sec^2

		intake_speed = 0.65,
		transfer_speed = 0.6,
		feed_speed = 0.4,
		shooter_default_speed = 0.85,

		// shooter_velocity = 8.0,				// in meters per second
		// shooter_speed_tollerance = 0.1,		// in meters per second

// Physical properties
		drivetrack_width_inches = 25.0,		// track width of drivebase
		drivewheel_diameter_inches = 6.0,	// wheel diameter for drivebase
		shooter_diameter_inches = 8.0,

// DriveBase closed-loop params
		ramsete_B = 2.0,
		ramsete_Zeta = 0.7,					// constants for ramsete command -> recommended in WPILib docs

		srx_mag_units_per_revolution = 4096,	// self explainatory (12 bit precision)
		falcon_units_per_revolution = 2048;		// self explainatory (11 bit precision)

	public static final CLDifferentialBase.CLParams cl_params = new CLDifferentialBase.CLParams(
		Units.inchesToMeters(drivetrack_width_inches),		// drivebase track width in meters
		Units.inchesToMeters(drivewheel_diameter_inches),	// drivebase wheel diameter in meters
		0.0,	// "kS"(volts) -> base voltage required to overcome static friction -> from SysID characterization
		0.0,	// "kV"(volts * seconds / meters) -> voltage required for each additional meter/second of velocity -> from SysID characterization
		0.0,	// "kA"(volts * seconds^2 / meters) -> voltage required for each additional meter/second^2 of acceleration -> from SysID characterization
		0.0,	// "kP"(volts * seconds / meters) -> voltage added to correct for error
		3.0,	// maximum velocity in meters per second
		3.0		// maximum acceleration in meters per second squared
	);


	public static final String
		cargo_cam_name = "Cargo",
		hub_cam_name = "Hub";
	public static final CameraPreset
		cam_driving = new CameraPreset(50, -1, -1),
		cam_hub_pipeline = new CameraPreset(50, 10, 3500),
		cam_cargo_pipeline = new CameraPreset(50, 25, 3500);
	public static final EntryPreset[]
		cargo_pipeline_options = new EntryPreset[]{
			new NumberOption("Scaling", 4.0)
		},
		hub_pipeline_options = new EntryPreset[]{

		};


}