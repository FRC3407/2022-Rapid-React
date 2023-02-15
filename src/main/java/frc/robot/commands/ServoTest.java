package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;


public final class ServoTest {
	
	public static class ConfigServo extends Servo {
		public ConfigServo(int c, double min_ms, double max_ms, double center_ms) {
			super(c);
			super.setBounds(max_ms, 0, center_ms, 0, min_ms);
		}
		public ConfigServo(int c, double min_ms, double max_ms) {
			this(c, min_ms, max_ms, 0);
		}
	}

	public static class Percentage extends CommandBase {
		private final Servo servo;
		private final DoubleSupplier percent;
		private double last;
		public Percentage(Servo s, DoubleSupplier p) {
			this.servo = s;
			this.percent = p;
		}
		public Percentage(Servo s, double p) {
			this(s, ()->p);
		}
		@Override
		public void initialize() {
			this.last = this.percent.getAsDouble();
			this.servo.set(this.last);
		}
		@Override
		public void execute() {
			double n = this.percent.getAsDouble();
			if(this.last != n) {
				this.last = n;
				this.servo.set(n);
			}
		}
		@Override
		public void end(boolean i) {
			this.servo.setDisabled();
		}
	}
	public static class Angle extends CommandBase {
		private final Servo servo;
		private final DoubleSupplier angle;
		private double last;
		public Angle(Servo s, DoubleSupplier a) {
			this.servo = s;
			this.angle = a;
		}
		public Angle(Servo s, double a) {
			this(s, ()->a);
		}
		@Override
		public void initialize() {
			this.last = this.angle.getAsDouble();
			this.servo.set(this.last);
		}
		@Override
		public void execute() {
			double n = this.angle.getAsDouble();
			if(this.last != n) {
				this.last = n;
				this.servo.setAngle(n);
			}
		}
		@Override
		public void end(boolean i) {
			this.servo.setDisabled();
		}
	}

}
