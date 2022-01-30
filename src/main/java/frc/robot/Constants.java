package frc.robot;

import frc.robot.modules.common.Types.*;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public final class Constants {
    //        descriptions >>  {fl fr bl br} {MotorController Instantiation}  {inversion settings}  {Drive layout}
    public static final DB4 
        drivebase_map = new DB4(3, 1, 2, 0, (int p)->new PWMVictorSPX(p), Inversions.LEFT, DriveLayout.DIFFERENTIAL);
    public static final DBS 
        drivebase_settings = new DBS(Deceleration._98, false);

}