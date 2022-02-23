package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.common.EventTriggers.EnabledTrigger;
import frc.robot.modules.common.Input.DigitalSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;


/**
 * Consists of the intake, transfer, feed, and shooter (and limit switches). 
 * Combines all three in order to provide "smart" features like counting and indexing cargo
 */
public class CargoSystem extends SubsystemBase {

	private final CargoStates transfer_state = new CargoStates();
	private final Robot.Intake intake_sys;
	private final Robot.Transfer transfer_sys;
	private final Robot.Shooter shooter_sys;

	public CargoSystem(Robot.Intake i, Robot.Transfer t, Robot.Shooter s) {
		this.intake_sys = i;
		this.transfer_sys = t;
		this.shooter_sys = s;

		EnabledTrigger.Get().whileActiveContinuous(
			()->{ 
				this.transfer_state.update(
					this.transfer_sys.lim_entering.get(),
					this.transfer_sys.lim_exiting.get()
				);
				//System.out.println("Updated Transfer System State");
			}
		);
	}	
	public CargoSystem(
		int port_intake, int port_feed,
		int canid_main, int canid_secondary,
		int dio_lim_front, int dio_lim_back,
		int... transfer_ports
	) { this(
			new Robot.Intake(port_intake),
			new Robot.Transfer(dio_lim_front, dio_lim_back, transfer_ports),
			new Robot.Shooter(port_feed, canid_main, canid_secondary)
		);
	}
	public CargoSystem(
		int port_intake, int port_feed,
		int canid_main,
		int dio_lim_front, int dio_lim_back,
		int... transfer_ports
	) { this(
			new Robot.Intake(port_intake),
			new Robot.Transfer(dio_lim_front, dio_lim_back, transfer_ports),
			new Robot.Shooter(port_feed, canid_main)
		);
	}

	public ManualOverride manualOverride(DigitalSupplier i, DigitalSupplier t, DigitalSupplier s) {
		return new ManualOverride(this, i, t, s);
	}





	public static abstract class CargoSystemCommand extends CommandBase {
		private final CargoSystem cargo_sys;
		protected CargoSystemCommand(CargoSystem cs) {
			this.cargo_sys = cs;
			super.addRequirements(cs, cs.intake_sys, cs.transfer_sys, cs.shooter_sys);
		}
		protected void setIntake(double s) { this.cargo_sys.intake_sys.motor.set(s); }
		protected void stopIntake() { this.cargo_sys.intake_sys.motor.stopMotor(); }
		protected void setFeed(double s) { this.cargo_sys.shooter_sys.feed.set(s); }
		protected void stopFeed() { this.cargo_sys.shooter_sys.feed.stopMotor(); }
		protected void setTransfer(double s) { this.cargo_sys.transfer_sys.primary.set(s); }
		protected void stopTransfer() { this.cargo_sys.transfer_sys.primary.stopMotor(); }
		public boolean getEnteringTriggered() { return this.cargo_sys.transfer_sys.lim_entering.get(); }
		public boolean getExitingTriggered() { return this.cargo_sys.transfer_sys.lim_exiting.get(); }
		protected void setShooterPercentage(double p) { this.cargo_sys.shooter_sys.main.set(ControlMode.PercentOutput, p); }
		protected void setshooterVelocity(double v) { this.cargo_sys.shooter_sys.main.set(ControlMode.Velocity, v); }
		public double getShooterRawPosition() { return this.cargo_sys.shooter_sys.main.getSelectedSensorPosition(); }	// in raw encoder units
		public double getShooterRawVelocity() { return this.cargo_sys.shooter_sys.main.getSelectedSensorVelocity(); }	// in raw units per 100 ms
		public double getShooterVelocity() { 
			return this.getShooterRawVelocity() *
			(Constants.shooter_wheel_diameter_meters * Math.PI / Constants.falcon_encoder_units_per_revolution) * 10;
		}
		protected void stopShooter() { this.cargo_sys.shooter_sys.main.stopMotor(); }
		public int cargoCount() { return this.cargo_sys.transfer_state.cargoCount(); }
		public boolean isCargoEntering() { return this.cargo_sys.transfer_state.isCargoEntering(); }
		public boolean isCargoExiting() { return this.cargo_sys.transfer_state.isCargoExiting(); }
		public boolean spaceAvailable() { return this.cargo_sys.transfer_state.spaceAvailable(); }
		public boolean canShoot() { return this.cargo_sys.transfer_state.canShoot(); }

		@Override public void end(boolean i) {
			this.stopIntake();
			this.stopFeed();
			this.stopTransfer();
			this.stopShooter();
		}
		@Override public boolean runsWhenDisabled() { return false; }

	}
	public static class ManualOverride extends CargoSystemCommand {
		private DigitalSupplier
			intake, transfer, shooter;
		public ManualOverride(CargoSystem m, DigitalSupplier i, DigitalSupplier t, DigitalSupplier s) {
			super(m);
			this.intake = i;
			this.transfer = t;
			this.shooter = s;
		}
		@Override public void execute() {
			if(this.intake.get()) {
				super.setIntake(Constants.intake_speed);
			} else {
				super.setIntake(0);
			}
			if(this.transfer.get()) {
				super.setTransfer(Constants.transfer_speed);
			} else {
				super.setTransfer(0);
			}
			if(this.shooter.get()) {
				// schedule shooter command
			}
		}
		@Override public boolean isFinished() { return false; }
	}
	public static class IntakeCargo extends CargoSystemCommand {
		private final double speed;
		private boolean last_lim;
		public IntakeCargo(CargoSystem m, double s) {
			super(m);
			this.speed = s;
			this.last_lim = super.isCargoEntering();
		}
		private boolean checkAndUpdate() {
			this.last_lim = super.isCargoEntering();
			return this.last_lim;
		}
		@Override public void execute() { 
			if(super.spaceAvailable()) {
				super.setIntake(this.speed); 
			}
		}
		@Override public boolean isFinished() {
			boolean temp = this.last_lim;	// just to be safe
			return !super.spaceAvailable() || (!temp && this.checkAndUpdate());
		}
	}


	private static class CargoStates {

		private int count = 0;
		private boolean
			last_entering = false,
			last_exiting = false;

		public int update(boolean entering, boolean exiting) {
			if(this.last_entering && !entering) {	// high to low on entering lim switch -> a ball has entered
				this.count++;
			}
			if(this.last_exiting && !exiting) {		// high to low on exiting lim switch -> a ball is leaving 
				this.count--;
			}
			this.last_entering = entering;
			this.last_exiting = exiting;
			return this.count;
		}

		public int cargoCount() { return this.count; }
		public boolean isCargoEntering() { return this.last_entering; }
		public boolean isCargoExiting() { return this.last_exiting; }
		public boolean spaceAvailable() { return this.count < 2; }
		public boolean canShoot() { return this.count > 0; }

	}


}