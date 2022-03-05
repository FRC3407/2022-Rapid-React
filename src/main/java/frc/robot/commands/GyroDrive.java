package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.modules.common.drive.DriveBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;


public class GyroDrive {

	public static class Straight extends DriveBase.RateLimitedAutoDrive {

		private final Gyro gyro;
		private final double speed, initial;

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

		@Override public void execute() {
			double o = this.gyro.getAngle() - this.initial;
			super.autoDrive(this.speed - (o / 90.0 * speed), this.speed + (o / 90.0 * speed));	// probably needs tuning
		}


	}

	public static class TurnInPlace extends DriveBase.RateLimitedAutoDrive {

		private final Gyro gyro;
		private final double target;

		public TurnInPlace(DriveBase db, Gyro gy, double deg) {	// deg -> angle in degrees, positive is clockwise looking down
			super(db);
			this.gyro = gy;
			this.target = this.gyro.getAngle() + deg;
		}
		public TurnInPlace(DriveBase db, Gyro gy, double deg, double m) {	// deg -> angle in degrees, positive is clockwise looking down
			super(db, m);
			this.gyro = gy;
			this.target = this.gyro.getAngle() + deg;
		}

		@Override public void execute() {
			super.autoTurn(Math.sqrt(this.target - this.gyro.getAngle() / 90.0) * Constants.auto_max_turn_speed);	// also probably needs tuning
		}


	}

	public static class ArcTurn extends DriveBase.RateLimitedAutoDrive {

		private final Gyro gyro;
		private final double target, speed;

		public ArcTurn(DriveBase db, Gyro gy, double s, double deg) {
			super(db);
			this.gyro = gy;
			this.speed = s;
			this.target = this.gyro.getAngle() + deg;
		}
		public ArcTurn(DriveBase db, Gyro gy, double s, double deg, double m) {
			super(db, m);
			this.gyro = gy;
			this.speed = s;
			this.target = this.gyro.getAngle() + deg;
		}

		@Override public void execute() {
			double o = this.target - this.gyro.getAngle();
			super.autoDrive(speed + (o / 360 * (speed/2.0)), speed - (o / 360 * (speed/2.0)));	// this will need to be fixed
		}


	}


}