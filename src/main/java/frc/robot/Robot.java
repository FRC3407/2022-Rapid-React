package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;


public class Robot {

    private final Intake intake = null;
    private final Transfer transfer = null;
    private final Shooter shooter = null;
	
	public static class Intake extends SubsystemBase {
		
        private final PWMVictorSPX 
            motor;

        public Intake(int p) {
            this.motor = new PWMVictorSPX(p);
        }

        public void set(double s) {
            this.motor.set(s);
        }
        public void setVoltage(double v) {
            this.motor.setVoltage(v);
        }


	}
	public static class Transfer extends SubsystemBase {
        
        private final PWMVictorSPX feed;
        private final MotorControllerGroup primary;
        private final DigitalInput
            t_intake,   // this would be a limit switch just after the intake and before "ball position 1"
            t_mid,      // this would be a limit switch inbetween the ball positions
            t_shooter;  // this would be a limit switch just before a ball enters the shooter

        public Transfer(int di, int dm, int ds, int feedp, int... ports) {
            this.t_intake = new DigitalInput(di);
            this.t_mid = new DigitalInput(dm);
            this.t_shooter = new DigitalInput(ds);
            this.feed = new PWMVictorSPX(feedp);
            PWMVictorSPX[] motors = new PWMVictorSPX[ports.length];
            for(int i = 0; i < ports.length; i++) {
                motors[i] = new PWMVictorSPX(ports[i]);
            }
            this.primary = new MotorControllerGroup(motors);
        }

        @Override public void periodic() {
            // get all limit switch values and update "states" of where the balls are in the transfer system
            // maybe create a separate class that handles all the "states"
        }

        public void setFeed(double s) {
            this.feed.set(s);
        }
        public void setFeedVoltage(double s) {
            this.feed.setVoltage(s);
        }
        public void stopFeed() {
            this.feed.stopMotor();
        }
        public void setPrimary(double s) {
            this.primary.set(s);
        }
        public void setPrimaryVoltage(double s) {
            this.primary.set(s);
        }
        public void stopPrimary() {
            this.primary.stopMotor();
        }
        
        public boolean isIntakeTriggered() {
            return this.t_intake.get();
        }
        public boolean isMidTriggered() {
            return this.t_mid.get();
        }
        public boolean isShooterTriggered() {
            return this.t_shooter.get();
        }

        private static class BallPositions {

            private boolean
				position1 = false, position2 = false,
				last_intake = false, last_mid = false, last_shooter = false;

			public void update(boolean i, boolean m, boolean s) {
				if(this.position1 == true && m == true) {
					// the ball was in position 1 but has now moved out
					this.position1 = false;
				}
				if(this.position2 == true && s == true) {
					// the ball was in position 2 but is moving to be shot
					this.position2 = false;
				}
				if(this.last_intake == true && i == false) {
					// a ball has moved into the first position
					this.position1 = true;
				}
				if(this.last_mid == true && m == false) {
					// a ball has moved into the second position
					this.position2 = true;
				}
				if(last_shooter == true && s == false) {
					// a ball was shot
				}
				this.last_intake = i;
				this.last_mid = m;
				this.last_shooter = s;
			}

			public int ballCount() {
				return this.position1 ? (this.position2 ? 2 : 1) : (this.position2 ? 1 : 0);
			}
			public boolean ballPos1() {
				return this.position1;
			}
			public boolean ballPos2() {
				return this.position2;
			}
			public boolean canIntake() {
				return (!this.last_intake && !this.position2 && !this.last_shooter);
			}
			public boolean canShoot() {
				return this.ballCount() > 0;
			}

			/* info to provide:
			- number of balls currently in the system
			- can we intake base on ^^^
			- can we shoot based on ^^^ and ball positions
			
			*/

        }


	}
	public static class Shooter extends SubsystemBase {		// add a way to invert
		
		private final WPI_TalonFX
            main, secondary;

		public Shooter(int i) {
            this.main = new WPI_TalonFX(i);
			this.secondary = null;
            this.main.configFactoryDefault();
            this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		}
		public Shooter(int i_m, int i_s) {
            this.main = new WPI_TalonFX(i_m);
            this.secondary = new WPI_TalonFX(i_s);
            this.main.configFactoryDefault();
            this.secondary.configFactoryDefault();
            this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
            this.secondary.follow(this.main);
            this.secondary.setInverted(InvertType.FollowMaster);
		}

		public void setPercentage(double p) {
			this.main.set(ControlMode.PercentOutput, p);
		}
		public void setVelocity(double v) {		// input velocity is currently in raw units per 100 ms
			this.main.set(ControlMode.Velocity, v);
		}
        public double getRawPosition() {	// in raw encoder units (see Constants.falcon_encoder_units_per_revolution)
            return this.main.getSelectedSensorPosition();
        }
        public double getRawVelocity() {	// in raw units per 100 ms
            return this.main.getSelectedSensorVelocity();
        }
        public double getVelocity() {	// in meters per second
            return this.getRawVelocity() * 
                (Constants.shooter_wheel_diameter_meters * Math.PI / Constants.falcon_encoder_units_per_revolution) * 10;
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