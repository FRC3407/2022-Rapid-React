package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.team3407.drive.Motors;
import frc.robot.team3407.drive.Motors.MotorSupplier;


public class ClimberSubsystem implements Subsystem {

	public static enum ClimberState {
		//IDLE		(0),
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

		protected final BooleanSupplier trigger;
		protected final double ext_voltage, ret_voltage, hold_ext_voltage, hold_ret_voltage;
		protected ClimberState state = ClimberState.RETRACTED;
		private boolean last = false;

		public ToggleControl(ClimberSubsystem cl, BooleanSupplier t, double ev, double rv, double hrv) { this(cl, t, ev, rv, 0, hrv); }
		public ToggleControl(ClimberSubsystem cl, BooleanSupplier t, double ev, double rv, double hev, double hrv) {
			super(cl);
			this.trigger = t;
			this.ext_voltage = ev;
			this.ret_voltage = rv;
			this.hold_ext_voltage = hev;
			this.hold_ret_voltage = hrv;
		}

		@Override public void initialize() {
			this.last = false;
			System.out.println("ToggleClimber: Running...");
		}
		@Override public void execute() {
			boolean t = trigger.getAsBoolean();
			if(t && !this.last) {
				this.state = this.state.next();
			}
			switch(this.state) {
				case RETRACTED: {
					super.setVoltage(this.hold_ret_voltage * -1);
					break;
				}
				case EXTENDING: {
					super.setVoltage(this.ext_voltage);
					break;
				}
				case EXTENDED: {
					super.setVoltage(this.hold_ext_voltage);
					break;
				}
				case RETRACTING: {
					super.setVoltage(this.ret_voltage * -1);
					break;
				}
			}
			this.last = t;
		}
		@Override public void end(boolean i) {
			super.stop();
			System.out.println("ToggleClimber: " + (i ? "Terminated." : "Completed."));
		}
		@Override public boolean isFinished() {
			return false;
		}

		public static class FullLoop extends ToggleControl {

			private int iteration = 0;

			public FullLoop(ClimberSubsystem cl, BooleanSupplier t, double ev, double rv, double hrv) { super(cl, t, ev, rv, hrv); }
			public FullLoop(ClimberSubsystem cl, BooleanSupplier t, double ev, double rv, double hev, double hrv) { super(cl, t, ev, rv, hev, hrv); }

			@Override public void execute() {
				boolean t = trigger.getAsBoolean();
				if(t && !super.last) {
					super.state = super.state.next();
					this.iteration++;
				}
				switch(super.state) {
					case RETRACTED: {
						if(this.iteration > 4) {
							this.iteration = 0;
						}
						if(this.iteration == 0) {
							super.setVoltage(0);
						} else {
							super.setVoltage(this.hold_ret_voltage * -1);
						}
						break;
					}
					case EXTENDING: {
						super.setVoltage(this.ext_voltage);
						break;
					}
					case EXTENDED: {
						super.setVoltage(this.hold_ext_voltage);
						break;
					}
					case RETRACTING: {
						super.setVoltage(this.ret_voltage * -1);
						break;
					}
				}
			}


		}


	}
	public static class HoldToggleControl extends ToggleControl {

		public HoldToggleControl(ClimberSubsystem cl, BooleanSupplier t, double ev, double rv, double hrv) { super(cl, t, ev, rv, hrv); }
		public HoldToggleControl(ClimberSubsystem cl, BooleanSupplier t, double ev, double rv, double hev, double hrv) { super(cl, t, ev, rv, hev, hrv); }

		@Override public void execute() {
			switch(super.state) {
				case RETRACTED: {
					super.setVoltage(super.hold_ret_voltage * -1);
					if(super.trigger.getAsBoolean()) {
						super.state = super.state.next();
					}
					break;
				}
				case EXTENDING: {
					super.setVoltage(super.ext_voltage);
					if(!super.trigger.getAsBoolean()) {
						super.state = super.state.next();
					}
					break;
				}
				case EXTENDED: {
					super.setVoltage(super.hold_ext_voltage);
					if(super.trigger.getAsBoolean()) {
						super.state = super.state.next();
					}
					break;
				}
				case RETRACTING: {
					super.setVoltage(super.ret_voltage * -1);
					if(!super.trigger.getAsBoolean()) {
						super.state = super.state.next();
					}
					break;
				}
			}
		}

		public static class FullLoop extends ToggleControl.FullLoop {

			public FullLoop(ClimberSubsystem cl, BooleanSupplier t, double ev, double rv, double hrv) { super(cl, t, ev, rv, hrv); }
			public FullLoop(ClimberSubsystem cl, BooleanSupplier t, double ev, double rv, double hev, double hrv) { super(cl, t, ev, rv, hev, hrv); }

			@Override public void execute() {
				switch(super.state) {
					case RETRACTED: {
						if(super.iteration > 4) {
							super.iteration = 0;
						}
						if(super.iteration == 0) {
							super.setVoltage(0);
						} else {
							super.setVoltage(this.hold_ret_voltage * -1);
						}
						if(super.trigger.getAsBoolean()) {
							super.state = super.state.next();
							super.iteration++;
						}
						break;
					}
					case EXTENDING: {
						super.setVoltage(super.ext_voltage);
						if(!super.trigger.getAsBoolean()) {
							super.state = super.state.next();
							super.iteration++;
						}
						break;
					}
					case EXTENDED: {
						super.setVoltage(super.hold_ext_voltage);
						if(super.trigger.getAsBoolean()) {
							super.state = super.state.next();
							super.iteration++;
						}
						break;
					}
					case RETRACTING: {
						super.setVoltage(super.ret_voltage * -1);
						if(!super.trigger.getAsBoolean()) {
							super.state = super.state.next();
							super.iteration++;
						}
						break;
					}
				}
			}
	
	
		}


	}


}