package frc.robot.team3407.drive;

import edu.wpi.first.wpilibj.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;


public class Motors {

	public static interface MotorSupplier<M extends MotorController> { M create(int p); }

	public static final MotorSupplier<PWMTalonFX> pwm_talonfx = (int p)->new PWMTalonFX(p);
	public static final MotorSupplier<PWMTalonSRX> pwm_talonsrx = (int p)->new PWMTalonSRX(p);
	public static final MotorSupplier<PWMVictorSPX> pwm_victorspx = (int p)->new PWMVictorSPX(p);
	public static final MotorSupplier<VictorSP> pwm_victorsp = (int p)->new VictorSP(p);
	public static final MotorSupplier<Victor> pwm_victor = (int p)->new Victor(p);
	public static final MotorSupplier<Talon> pwm_talon = (int p)->new Talon(p);
	public static final MotorSupplier<PWMSparkMax> pwm_sparkmax = (int p)->new PWMSparkMax(p);
	public static final MotorSupplier<Spark> pwm_spark = (int p)->new Spark(p);
	public static final MotorSupplier<PWMVenom> pwm_venom = (int p)->new PWMVenom(p);

	public static final MotorSupplier<WPI_TalonFX> can_talonfx = (int id)->new WPI_TalonFX(id);
	public static final MotorSupplier<WPI_TalonSRX> can_talonsrx = (int id)->new WPI_TalonSRX(id);
	public static final MotorSupplier<WPI_VictorSPX> can_victorspx = (int id)->new WPI_VictorSPX(id);


}