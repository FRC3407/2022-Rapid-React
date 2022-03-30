package frc.robot;

import frc.robot.modules.common.drive.Motors;
import frc.robot.modules.common.drive.Motors.MotorSupplier;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.*;


public class ClimberSubsystem implements Subsystem {

	public static enum ClimberState {
		RETRACTED	(-1),
		EXTENDING	(1),
		EXTENDED	(1),
		RETRACTING	(-1);

		public final int sign;
		private ClimberState(int m) { this.sign = m; }
		
		public ClimberState next() {
			for(int i = 0; i < values().length; i++) {
				if(values()[i] == this) {
					return i == values().length-1 ? values()[0] : values()[i + 1];
				}
			}
			return this;	// null???
		}
	}


	private final MotorController motor;

	public ClimberSubsystem(int p) { this(p, Motors.pwm_victorspx); }
	public ClimberSubsystem(int p1, int p2) { this(p1, p2, Motors.pwm_victorspx); }
	public<M extends MotorController> ClimberSubsystem(int p, MotorSupplier<M> t) {
		this.motor = t.create(p);
	}
	public<M extends MotorController> ClimberSubsystem(int p1, int p2, MotorSupplier<M> t) {
		this.motor = new MotorControllerGroup(t.create(p1), t.create(p2));
	}

	public void setInverted(boolean i) {
		this.motor.setInverted(i);
	}

	public void set(double p, ClimberCommand c) {
		this.motor.set(p);
	}
	public void setVoltage(double v, ClimberCommand c) {
		this.motor.setVoltage(v);
	}
	public void stop(ClimberCommand c) {
		this.motor.stopMotor();
	}
	public double getLast() {
		return this.motor.get();
	}

	public static abstract class ClimberCommand extends CommandBase {

		protected final ClimberSubsystem climber_sys;
		protected ClimberCommand(ClimberSubsystem cl) {
			this.climber_sys = cl;
			super.addRequirements(cl);
		}
		@Override public void end(boolean i) { this.climber_sys.stop(this); }

		protected final void set(double s) {
			this.climber_sys.set(s, this);
		}
		protected final void setVoltage(double v) {
			this.climber_sys.setVoltage(v, this);
		}
		protected final void stop() {
			this.climber_sys.stop(this);
		}


	}



	public static class ToggleControl extends ClimberCommand {

		private final BooleanSupplier trigger;
		private ClimberState state = ClimberState.RETRACTED;
		private boolean last;

		public ToggleControl(ClimberSubsystem cl, BooleanSupplier t) {
			super(cl);
			this.trigger = t;
		}

		@Override public void initialize() {

		}
		@Override public void execute() {
			boolean t = trigger.getAsBoolean();
			if(t && !this.last) {
				this.state = this.state.next();
			}
			switch(this.state) {
				case RETRACTED: {
					
				}
				case EXTENDING: {

				}
				case EXTENDED: {

				}
				case RETRACTING: {

				}
			}
			this.last = t;
		}
		@Override public void end(boolean i) {

		}
		@Override public boolean isFinished() {
			return false;
		}


	}


}