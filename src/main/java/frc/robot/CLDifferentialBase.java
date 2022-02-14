// package frc.robot;

// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
// import edu.wpi.first.wpilibj.Encoder;
// import frc.robot.modules.common.DriveBase;
// import frc.robot.modules.common.Types.*;
// import com.ctre.phoenix.motorcontrol.can.*;
// import com.ctre.phoenix.motorcontrol.*;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;


// // 'Closed-Loop' Differential drivebase
// public class CLDifferentialBase extends DriveBase {	// not completely safe yet as an extension of DriveBase (managing control)

// 	private final Gyro 
// 		gyro = new ADIS16470();

// 	private final WPI_TalonSRX
// 		left, right;

// 	// private void initTalons() {
// 	// 	front_left.configFactoryDefault();
// 	// 	front_right.configFactoryDefault();
// 	// 	back_left.configFactoryDefault();
// 	// 	back_right.configFactoryDefault();

// 	// 	back_left.follow(front_left);
// 	// 	back_right.follow(front_right);
// 	// 	back_left.setInverted(InvertType.FollowMaster);
// 	// 	back_right.setInverted(InvertType.FollowMaster);

// 	// 	front_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
// 	// 	front_right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
// 	// }

// 	public static DB2 construct(DB2 map, WPI_TalonSRX l, WPI_TalonSRX r) {
// 		l = new WPI_TalonSRX(map.p_left);
// 		r = new WPI_TalonSRX(map.p_right);
// 		return map;
// 	}

// 	public CLDifferentialBase(DB2 map, DBS s) {
// 		super(map, s);
// 	}
// 	public CLDifferentialBase(DB4 map, DBS s) {
// 		super(map, s);
// 	}
	
// }
