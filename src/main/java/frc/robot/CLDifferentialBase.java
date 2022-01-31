package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.modules.common.DriveBase;
import frc.robot.modules.common.Types;
import frc.robot.modules.common.Types.DB2;
import frc.robot.modules.common.Types.DB4;
import frc.robot.modules.common.Types.DBS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


// 'Closed-Loop' Differential drivebase
public class CLDifferentialBase extends DriveBase {

	//private final Gyro gyro;
	//private final Encoder left, right;

	public CLDifferentialBase(Types.DB2 map, Types.DBS s) {
		super(map, s);
	}
	public CLDifferentialBase(Types.DB4 map, Types.DBS s) {
		super(map, s);
	}
	
}
