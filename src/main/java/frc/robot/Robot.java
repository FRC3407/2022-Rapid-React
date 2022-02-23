package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;


public class Robot {
	
	protected static class Intake extends SubsystemBase {
		
        protected final PWMVictorSPX 
            motor;
        // private EnableIntake 
        //     enable = new EnableIntake(this, 0);
        public Intake(int p) {
            this.motor = new PWMVictorSPX(p);
        }

        // public EnableIntake intakeCommand() { return this.enable; }
        // public EnableIntake intakeCommand(double s) {
        //     if(s != this.enable.speed) {    // no need to reinstantiate if the speed is the same
        //         this.enable = new EnableIntake(this, s);
        //     }
        //     return this.enable;
        // }

        /**
         * Intake control is granted through extending this class. This enforces the use of "addRequirements()" and makes sure 
         * no 2 things are trying to control the intake motor at a time
        */
        public static abstract class IntakeCommand extends CommandBase {
            protected final Intake intake;
            protected IntakeCommand(Intake i) {
                this.intake = i;
                super.addRequirements(i);
            }
            protected void set(double s) { this.intake.motor.set(s); }
            protected void setVoltage(double v) { this.intake.motor.setVoltage(v); }
            protected void stop() { this.intake.motor.stopMotor(); }
            @Override public void end(boolean i) { this.stop(); }   // stop motor when finished by default
            @Override public boolean runsWhenDisabled() { return false; }
        }

        // public static class EnableIntake extends IntakeCommand {	// implemented in ManualOverride in CargoSystem class

        //     private final double speed;
        //     public EnableIntake(Intake i, double s) {
        //         super(i);
        //         this.speed = s;
        //     }
        //     @Override public void execute() { super.set(this.speed); }
        //     @Override public boolean isFinished() { return false; }


        // }


	}
	protected static class Transfer extends SubsystemBase {
        
        protected final MotorControllerGroup 
            primary;
        protected final DigitalInput
			lim_entering,   // this would be a limit switch just after the intake and before "ball position 1"
            //t_mid,      // this would be a limit switch inbetween the ball positions (may not need this one)
            lim_exiting;  // this would be a limit switch just before a ball enters the shooter

        public Transfer(int di, int ds, int... ports) {
            this.lim_entering = new DigitalInput(di);
            this.lim_exiting = new DigitalInput(ds);
            PWMVictorSPX[] motors = new PWMVictorSPX[ports.length];
            for(int i = 0; i < ports.length; i++) {
                motors[i] = new PWMVictorSPX(ports[i]);
            }
            this.primary = new MotorControllerGroup(motors);
        }

        public static abstract class TransferCommand extends CommandBase {
            protected final Transfer transfer;
            protected TransferCommand(Transfer t) {
                this.transfer = t;
                super.addRequirements(t);
            }
            protected void set(double s) { this.transfer.primary.set(s); }
            protected void setVoltage(double s) { this.transfer.primary.set(s); }
            protected void stop() { this.transfer.primary.stopMotor(); }
            protected boolean isIntakeTriggered() { return this.transfer.lim_entering.get(); }
            protected boolean isShooterTriggered() { return this.transfer.lim_exiting.get(); }
			@Override public void end(boolean i) { this.stop(); }   // stop motor when finished by default
            @Override public boolean runsWhenDisabled() { return false; }
        }

        // public static class AutomaticTransfer extends TransferCommand {

        // }


	}
	protected static class Shooter extends SubsystemBase {		// add a way to invert

		protected final WPI_TalonFX
            main, secondary;
		protected final PWMVictorSPX 
            feed;

		public Shooter(int f, int m) {
			this.feed = new PWMVictorSPX(f);
            this.main = new WPI_TalonFX(m);
			this.secondary = null;
            this.main.configFactoryDefault();
            this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		}
		public Shooter(int f, int m, int s) {
			this.feed = new PWMVictorSPX(f);
            this.main = new WPI_TalonFX(m);
            this.secondary = new WPI_TalonFX(s);
            this.main.configFactoryDefault();
            this.secondary.configFactoryDefault();
            this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
            this.secondary.follow(this.main);
            this.secondary.setInverted(InvertType.FollowMaster);
		}

		public static abstract class ShooterCommand extends CommandBase {
			protected final Shooter shooter;
			protected ShooterCommand(Shooter s) {
				this.shooter = s;
				super.addRequirements(s);
			}
			protected void setFeed(double s) { this.shooter.feed.set(s); }
			protected void setFeedVoltage(double s) { this.shooter.feed.setVoltage(s); }
			protected void stopFeed() { this.shooter.feed.stopMotor(); }

			protected void setShooterPercentage(double p) {
				this.shooter.main.set(ControlMode.PercentOutput, p);
			}
			protected void setShooterVelocity(double v) {		// input velocity is currently in raw units per 100 ms
				this.shooter.main.set(ControlMode.Velocity, v);
			}
			protected void stopShooter() {
				this.shooter.main.stopMotor();
			}
			public double getShooterRawPosition() {	// in raw encoder units (see Constants.falcon_encoder_units_per_revolution)
				return this.shooter.main.getSelectedSensorPosition();
			}
			public double getShooterRawVelocity() {	// in raw units per 100 ms
				return this.shooter.main.getSelectedSensorVelocity();
			}
			public double getShooterVelocity() {	// in meters per second
				return this.getShooterRawVelocity() * 
					(Constants.shooter_wheel_diameter_meters * Math.PI / Constants.falcon_encoder_units_per_revolution) * 10;
			}
			@Override public void end(boolean i) {	// stop motor when finished by default
				this.stopFeed();
				this.stopShooter();
			}
            @Override public boolean runsWhenDisabled() { return false; }
		}


	}
	public static class Climber extends SubsystemBase {
		
	}


}

/* POSSIBLE (high level) CONTROL COMMANDS TO AIM TOWARDS SUPPORTING
 - "Shoot" -> sets shooter to correct speed, feeds ball, then shifts secondary ball into position for another shot
 - "ShootVision" -> ^^^ but with calculated speed based on distance found from vision target
 - "Intake" -> spin up intake and possibly end the command when a ball is collected (prerequisite of having space for cargo)
 - "Climb" -> execute a full climbing action (prerequisite of being in position)

~~~~~~
Things to add: 
- ball management system for keeping track of position -> theoretically the only commands needed then would be those to intake and shoot, 
... the transfer system would be completely autonomous and self-contained

*/