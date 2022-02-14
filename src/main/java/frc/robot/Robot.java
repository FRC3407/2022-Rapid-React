package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
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

        public Transfer(int feedp, int... ports) {
            this.feed = new PWMVictorSPX(feedp);
            PWMVictorSPX[] motors = new PWMVictorSPX[ports.length];
            for(int i = 0; i < ports.length; i++) {
                motors[i] = new PWMVictorSPX(ports[i]);
            }
            this.primary = new MotorControllerGroup(motors);
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


	}
	public static class Shooter extends SubsystemBase {
		
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
		}

		public void setPercentage(double p) {
			this.main.set(ControlMode.PercentOutput, p);
		}
		public void setVelocity(double v) {
			this.main.set(ControlMode.Velocity, v);
		}

	}
	public static class Climber extends SubsystemBase {
		
	}

}