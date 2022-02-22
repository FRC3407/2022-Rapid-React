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
 * Consists of the intake, transfer system, feed, and shooter
 */
public class BallManagement extends SubsystemBase {

	private final PWMVictorSPX 
		intake,			// intake motor
		feed;			// feed motor (shooter)
	private final MotorControllerGroup
		transfer;		// transfer motors are all controlled together
	private final WPI_TalonFX
		main,			// main shooter (integrated encoder of this used)
		secondary;		// secondary shooter motor if added
	private final DigitalInput
		lim_entering,	// limit switch at beginning of transfer stage
		lim_exiting;	// limit switch at end of transfer stage
	private final CargoStates 
		transfer_state = new CargoStates();
	private final GenericSubsystem
		intake_sys = new GenericSubsystem(),
		transfer_sys = new GenericSubsystem(),
		shooter_sys = new GenericSubsystem();

	public BallManagement(
		int port_intake, 
		int port_feed,
		int canid_main,
		int canid_secondary,
		int dio_lim_front,
		int dio_lim_back,
		int... transfer_ports
	) {
		this.intake = new PWMVictorSPX(port_intake);
		this.feed = new PWMVictorSPX(port_feed);
		this.main = new WPI_TalonFX(canid_main);
		this.secondary = new WPI_TalonFX(canid_secondary);
		this.lim_entering = new DigitalInput(dio_lim_front);
		this.lim_exiting = new DigitalInput(dio_lim_back);
		PWMVictorSPX[] motors = new PWMVictorSPX[transfer_ports.length];
		for(int i = 0; i < transfer_ports.length; i++) {
			motors[i] = new PWMVictorSPX(transfer_ports[i]);
		}
		this.transfer = new MotorControllerGroup(motors);

		this.main.configFactoryDefault();
		this.secondary.configFactoryDefault();
		this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		this.secondary.follow(this.main);
		this.secondary.setInverted(InvertType.FollowMaster);

		EnabledTrigger.Get().whileActiveContinuous(
			()->{ 
				this.transfer_state.update(
					this.lim_entering.get(),
					this.lim_exiting.get()
				);
				//System.out.println("Updated Transfer System State");
			}
		);
	}
	public BallManagement(
		int port_intake, 
		int port_feed,
		int canid_main,
		int dio_lim_front,
		int dio_lim_back,
		int... transfer_ports
	) {
		this.intake = new PWMVictorSPX(port_intake);
		this.feed = new PWMVictorSPX(port_feed);
		this.main = new WPI_TalonFX(canid_main);
		this.secondary = null;
		this.lim_entering = new DigitalInput(dio_lim_front);
		this.lim_exiting = new DigitalInput(dio_lim_back);
		PWMVictorSPX[] motors = new PWMVictorSPX[transfer_ports.length];
		for(int i = 0; i < transfer_ports.length; i++) {
			motors[i] = new PWMVictorSPX(transfer_ports[i]);
		}
		this.transfer = new MotorControllerGroup(motors);

		this.main.configFactoryDefault();
        this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

		EnabledTrigger.Get().whileActiveContinuous(
			()->{ 
				this.transfer_state.update(
					this.lim_entering.get(),
					this.lim_exiting.get()
				);
				//System.out.println("Updated Transfer System State");
			}
		);
	}

	public ManualOverride manualOverride(DigitalSupplier i, DigitalSupplier t, DigitalSupplier s) {
		return new ManualOverride(this, i, t, s);
	}



	private class GenericSubsystem extends SubsystemBase {}

	public static abstract class BallManageCommand extends CommandBase {
		private final BallManagement management;
		protected BallManageCommand(BallManagement m) {
			this.management = m;
			super.addRequirements(m);
		}
		protected void setIntake(double s) { this.management.intake.set(s); }
		protected void stopIntake() { this.management.intake.stopMotor(); }
		protected void setFeed(double s) { this.management.feed.set(s); }
		protected void stopFeed() { this.management.feed.stopMotor(); }
		protected void setTransfer(double s) { this.management.transfer.set(s); }
		protected void stopTransfer() { this.management.transfer.stopMotor(); }
		public boolean getEnteringTriggered() { return this.management.lim_entering.get(); }
		public boolean getExitingTriggered() { return this.management.lim_exiting.get(); }
		protected void setShooterPercentage(double p) { this.management.main.set(ControlMode.PercentOutput, p); }
		protected void setshooterVelocity(double v) { this.management.main.set(ControlMode.Velocity, v); }
		public double getShooterRawPosition() { return this.management.main.getSelectedSensorPosition(); }	// in raw encoder units
		public double getShooterRawVelocity() { return this.management.main.getSelectedSensorVelocity(); }	// in raw units per 100 ms
		public double getShooterVelocity() { 
			return this.getShooterRawVelocity() *
			(Constants.shooter_wheel_diameter_meters * Math.PI / Constants.falcon_encoder_units_per_revolution) * 10;
		}
		protected void stopShooter() { this.management.main.stopMotor(); }
		public int cargoCount() { return this.management.transfer_state.cargoCount(); }
		public boolean isCargoEntering() { return this.management.transfer_state.isCargoEntering(); }
		public boolean isCargoExiting() { return this.management.transfer_state.isCargoExiting(); }
		public boolean spaceAvailable() { return this.management.transfer_state.spaceAvailable(); }
		public boolean canShoot() { return this.management.transfer_state.canShoot(); }

		@Override public void end(boolean i) {
			this.stopIntake();
			this.stopFeed();
			this.stopTransfer();
			this.stopShooter();
		}
		@Override public boolean runsWhenDisabled() { return false; }

	}
	public static class ManualOverride extends BallManageCommand {
		private DigitalSupplier
			intake, transfer, shooter;
		public ManualOverride(BallManagement m, DigitalSupplier i, DigitalSupplier t, DigitalSupplier s) {
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
	public static class IntakeCargo extends BallManageCommand {
		private double speed;
		public IntakeCargo(BallManagement m, double s) {
			super(m);
			this.speed = s;
		}
		@Override public void execute() { 
			if(super.spaceAvailable()) {
				super.setIntake(this.speed); 
			}
		}
		@Override public boolean isFinished() {
			return !super.spaceAvailable();
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