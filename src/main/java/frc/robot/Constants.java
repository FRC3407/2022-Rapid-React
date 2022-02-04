package frc.robot;

import frc.robot.modules.common.Types.*;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public final class Constants {
    //        descriptions >>  {fl fr bl br} {MotorController Instantiation}  {inversion settings}  {Drive layout}
    public static final DB4 
        drivebase_map = new DB4(3, 1, 2, 0, (int p)->new PWMVictorSPX(p), Inversions.LEFT, DriveLayout.DIFFERENTIAL),
        db_2019_map = new DB4(6, 8, 9, 7, (int p)->new PWMVictorSPX(p), Inversions.LEFT, DriveLayout.DIFFERENTIAL);
    public static final DBS 
        drivebase_settings = new DBS(Deceleration._98, true);


// degrees
    public static final double turn_thresh = 1.0;	// threshold targeting angle
	public static final double target_angle_range_lr = 20.0;	// maximum angle range that a target could be offset (used as divisor for P-loop)
// inches
	public static final double cargo_thresh = 10.0;		// threshold distance for a cargo to be considered close enough to influenced by intake
	public static final double cargo_distance_range = 100.0;	// maximum range that a cargo could be (and detected - used as divisor for P-loop)
// motorcontroller units (-1.0 to 1.0)
	public static final double uncertainty_continuation_percentage = 0.75;	// if break in target detection, keep powering motors at this percent of the last values used
	public static final double auto_max_turn_speed = 0.2;		// maximum speed when turning in place during auto
	public static final double auto_max_forward_speed = 0.4;	// maximum speed when driving forward during auto
	public static final double motors_thresh_tozero = 0.1;		// the point where it is safe to go straight to zero (deceleration)


}