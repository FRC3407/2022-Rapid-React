package frc.robot.team3407.drive;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;


public class Helper {
	
	// public static MotorController inlineInverter(MotorController m, boolean invert) {
	// 	m.setInverted(invert);
	// 	return m;
	// }
	public static<M extends MotorController> M getInverted(M m, boolean invert) {
		m.setInverted(invert);
		return m;
	}
	public static void applyDeceleration(Types.Deceleration constant, MotorController m, MotorController... ms) {
		m.set(m.get() * constant.value);
		for(int i = 0; i < ms.length; i++) {
			ms[i].set(ms[i].get() * constant.value);
		}
	}
	public static void applyDeceleration(Types.Deceleration constant, MotorController[] motors) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].set(motors[i].get() * constant.value);
		}
	}
	public static void applyPercentage(MotorController[] motors, double p) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].set(motors[i].get() * p);
		}
	}
	public static void applyAll(MotorController[] motors, double s) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].set(s);
		}
	}
	public static void applyStop(MotorController m, MotorController... ms) {
		m.set(0);
		for(int i = 0; i < ms.length; i++) {
			ms[i].set(0);
		}
	}
	public static void applyStop(MotorController[] motors) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].set(0);
		}
	}



	// public static final class TankDriveValues {
	// 	public final double
	// 		left, right;

	// 	public TankDriveValues(double l, double r) {
	// 		this.left = l;
	// 		this.right = r;
	// 	}
	// 	public static TankDriveValues turning(double v) {	// positive means right turn
	// 		return new TankDriveValues(v, -v);
	// 	}
	// 	public static TankDriveValues forward(double v) {	// positive forward
	// 		return new TankDriveValues(v, v);
	// 	}
	// }

	// public static double proportionalTargetFollow1D(

	// ) {
	// 	return 0;
	// }
	// public static TankDriveValues proportionalTargetFollowPolar(
	// 	double distance,
	// 	double distance_target,
	// 	double max_distance,
	// 	double static_forward,
	// 	double max_forward,
	// 	double left_right,
	// 	double offset_target,
	// 	double max_offset,
	// 	double static_turning,
	// 	double max_turning
	// ) {
	// 	double f_err = distance - distance_target;
	// 	double t_err = left_right - offset_target;


	// 	return new TankDriveValues(0, 0);
	// }


}