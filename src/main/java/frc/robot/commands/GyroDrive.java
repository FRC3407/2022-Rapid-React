package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import frc.robot.Constants;
import frc.robot.team3407.drive.DriveBase;


public class GyroDrive {

	public static final double
		max_error_degrees = 90,		// the angle which represents maximum error
		static_turn_voltage = Constants.turning_static_voltage,
		max_turn_voltage = Constants.auto_max_turn_voltage
	;

	public static class Straight extends DriveBase.DriveCommandBase {

		private final Gyro gyro;
		private final double speed;
		private double initial;

		public Straight(DriveBase db, Gyro gy, double s) {
			super(db);
			this.gyro = gy;
			this.speed = s;
			this.initial = this.gyro.getAngle();
		}

		@Override public void initialize() {
			this.initial = this.gyro.getAngle();
			System.out.println("Gyro-Straight: Running...");
		}
		@Override public void execute() {
			double e = MathUtil.clamp(this.gyro.getAngle() - this.initial, -1.0, 1.0) / max_error_degrees * this.speed;
			super.autoDrive(this.speed - e, this.speed + e);
		}
		@Override public void end(boolean i) {
			System.out.println("Gyro-Straight: " + (i ? "Terminated." : "Completed."));
		}

		public static class ByVoltage extends Straight {

			public ByVoltage(DriveBase db, Gyro gy, double v) {
				super(db, gy, v);
			}

			@Override public void execute() {
				double e = MathUtil.clamp(super.gyro.getAngle() - super.initial, -1.0, 1.0) / max_error_degrees * super.speed;
				super.autoDriveVoltage(super.speed - e, super.speed + e);
			}


		}


	}

	public static class TurnInPlace extends DriveBase.DriveCommandBase {

		private final Gyro gyro;
		private final double target;
		private double calculated;

		public TurnInPlace(DriveBase db, Gyro gy, double deg) {	// deg -> angle in degrees, positive is clockwise looking down
			super(db);
			this.gyro = gy;
			this.target = deg;
		}

		@Override public void initialize() {
			this.calculated = this.gyro.getAngle() + this.target;
			System.out.println("TurnInPlace: Running...");
		}
		@Override public void execute() {
			double e = MathUtil.clamp((this.calculated - this.gyro.getAngle()) / 90.0, -1.0, 1.0);
			super.autoTurnVoltage(
				(Math.signum(e) * static_turn_voltage) + (e * (max_turn_voltage - static_turn_voltage))
			);
		}
		@Override public void end(boolean i) {
			System.out.println("TurnInPlace: " + (i ? "Terminated." : "Completed."));
		}
		@Override public boolean isFinished() {
			return Math.abs(this.calculated - this.gyro.getAngle()) < 1.0;
		}


	}

	public static class ArcTurn extends DriveBase.DriveCommandBase {

		private final Gyro gyro;
		private final double target, left, right;
		private double calculated;
		private boolean failed = false;

		public ArcTurn(DriveBase db, Gyro gy, double l, double r, double deg) {
			super(db);
			this.gyro = gy;
			this.left = l;
			this.right = r;
			this.target = deg;
		}
		// public ArcTurn(DriveBase db, Gyro gy, double l, double r, double deg, double m) {
		// 	super(db, m);
		// 	this.gyro = gy;
		// 	this.left = l;
		// 	this.right = r;
		// 	this.target = deg;
		// }

		@Override public void initialize() {
			this.failed = false;
			double ratio = this.left / this.right;
			if(Math.abs(ratio - 1.0) < 0.05) {	// tune this -> if not enough difference to turn substancially
				System.out.println("ArcTurn: Drivespeed ratio of " + ratio + " will not produce tight enough turn. Exitting...");
				this.failed = true;
				return;
			}
			this.calculated = (ratio < 1.0) ? (this.gyro.getAngle() - this.target) : (this.gyro.getAngle() + this.target);
			System.out.println("ArcTurn: Running...");
		}
		@Override public void execute() {
			double o = this.calculated - this.gyro.getAngle();
			super.autoDrive(this.left - MathUtil.clamp(1.0 / o, -1.0, 1.0) * this.left, this.right - MathUtil.clamp(1.0 / o, -1.0, 1.0) * this.right);	// this will need to be fixed
		}
		@Override public void end(boolean i) {
			System.out.println("ArcTurn: " + (i ? "Terminated." : "Completed."));
		}
		@Override public boolean isFinished() {
			return this.failed || Math.abs(this.calculated - this.gyro.getAngle()) < 1.0;
		}


	}


}