package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.modules.common.drive.DriveBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.interfaces.Gyro;


public class GyroDrive {

	public static class Straight extends DriveBase.RateLimitedAutoDrive {

		private final Gyro gyro;
		private final double speed;
		private double initial;

		public Straight(DriveBase db, Gyro gy, double s) {
			super(db);
			this.gyro = gy;
			this.speed = s;
			this.initial = this.gyro.getAngle();
		}
		public Straight(DriveBase db, Gyro gy, double s, double m) {
			super(db, m);
			this.gyro = gy;
			this.speed = s;
			this.initial = this.gyro.getAngle();
		}

		@Override public void initialize() {
			this.initial = this.gyro.getAngle();
		}
		@Override public void execute() {
			double o = this.gyro.getAngle() - this.initial;
			super.autoDrive(this.speed - (o / 90.0 * speed), this.speed + (o / 90.0 * speed));	// probably needs tuning
		}


	}

	public static class TurnInPlace extends DriveBase.RateLimitedAutoDrive {

		private final Gyro gyro;
		private final double target;
		private double calculated;

		public TurnInPlace(DriveBase db, Gyro gy, double deg) {	// deg -> angle in degrees, positive is clockwise looking down
			super(db);
			this.gyro = gy;
			this.target = deg;
		}
		public TurnInPlace(DriveBase db, Gyro gy, double deg, double m) {	// deg -> angle in degrees, positive is clockwise looking down
			super(db, m);
			this.gyro = gy;
			this.target = deg;
		}

		@Override public void initialize() {
			this.calculated = this.gyro.getAngle() + this.target;
		}
		@Override public void execute() {
			System.out.println(((this.calculated - this.gyro.getAngle()) / 90.0) * Constants.auto_max_turn_speed);
			super.autoTurn(MathUtil.clamp(((this.calculated - this.gyro.getAngle()) / 90.0), -1.0, 1.0) * Constants.auto_max_turn_speed);	// also probably needs tuning
		}
		@Override public boolean isFinished() {
			return Math.abs(this.calculated - this.gyro.getAngle()) < 1.0;
		}


	}

	public static class ArcTurn extends DriveBase.RateLimitedAutoDrive {

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
		public ArcTurn(DriveBase db, Gyro gy, double l, double r, double deg, double m) {
			super(db, m);
			this.gyro = gy;
			this.left = l;
			this.right = r;
			this.target = deg;
		}

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
		@Override public boolean isFinished() {
			return this.failed || Math.abs(this.calculated - this.gyro.getAngle()) < 1.0;
		}


	}


}