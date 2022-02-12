package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;


public class Robot {
	
	public static class Intake extends SubsystemBase {
		
	}
	public static class Transfer extends SubsystemBase {

	}
	public static class Shooter extends SubsystemBase {
		
		private final WPI_TalonFX[] talons;

		public Shooter(int id) {
			this.talons = new WPI_TalonFX[1];
			this.talons[0] = new WPI_TalonFX(id);
			this.talons[0].configFactoryDefault();
			this.talons[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		}
		public Shooter(int id_1, int id_2) {
			this.talons = new WPI_TalonFX[2];
			this.talons[0] = new WPI_TalonFX(id_1);
			this.talons[1] = new WPI_TalonFX(id_2);
		}

		public void setPercentage(double p) {
			for(WPI_TalonFX talon : this.talons) {
				talon.set(TalonFXControlMode.PercentOutput, p);
			}
		}

	}
	public static class Climber extends SubsystemBase {
		
	}

}