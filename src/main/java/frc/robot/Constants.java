package frc.robot;

import frc.robot.modules.common.Types.*;
import frc.robot.modules.vision.java.VisionServer.*;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public final class Constants {

    //        descriptions >>  {fl fr bl br} {MotorController Instantiation}  {inversion settings}  {Drive layout}
    public static final DB4 
        drivebase_map = new DB4(3, 1, 2, 0, (int p)->new PWMVictorSPX(p), Inversions.LEFT, DriveLayout.DIFFERENTIAL),
        db_2019_map = new DB4(6, 8, 9, 7, (int p)->new PWMVictorSPX(p), Inversions.RIGHT, DriveLayout.DIFFERENTIAL);
    public static final DBS 
        drivebase_settings = new DBS(Deceleration._98, true);

	public static final int 
		intake_port = -1,		// port for intake motor
		feed_port = -1,			// port for transfer system feed motor
		shooter_port = -1;		// port for shooter motor (can id for falcon)
	public static final int[]
		transfer_ports = {-1};	// ports for all additional motors in transfer system (all controlled together)


	public static final double 
// degrees
		turn_thresh = 1.0,				// threshold targeting angle
		target_angle_range_lr = 20.0,	// maximum angle range that a target could be offset (used as divisor for P-loop)
// inches
		cargo_thresh = 20.0,			// threshold distance for a cargo to be considered close enough to influenced by intake
		cargo_distance_range = 100.0,	// maximum range that a cargo could be (and detected - used as divisor for P-loop)
// motorcontroller units (-1.0 to 1.0)
		uncertainty_continuation_percentage = 0.75,	// if break in target detection, keep powering motors at this percent of the last values used
		auto_max_turn_speed = 0.2,		// maximum speed when turning in place during auto
		auto_max_forward_speed = 0.4,	// maximum speed when driving forward during auto
		motors_thresh_tozero = 0.1,		// the point where it is safe to go straight to zero (deceleration)

// DriveBase closed-loop params
		kS_voltage = 0.0,					// voltage to overcome static friction -> from SysID characterization
		kV_volt_seconds_per_meter = 0.0,	// voltage for each meter/second of velocity -> from SysID characterization
		kA_volt_seconds2_per_meter = 0.0,	// voltage for each meter/seconds^2 of acceleration -> from SysID characterization
		kP_velocity_correction = 0.0,		// voltage gain for each meter/second velocity error in PID loop
		drive_track_width = 0.0,			// track width of drivebase in meters
		max_velocity = 3.0,					// max velocity of drivebase in meters/second
		max_acceleration = 3.0,				// max acceleration of drivebase in meters/second^2
		ramsete_B = 2.0,
		ramsete_Zeta = 0.7;					// constants for ramsete command -> recommended in WPILib docs


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