package frc.robot;

import java.util.function.DoubleSupplier;

import frc.robot.modules.common.drive.Motors;
import frc.robot.modules.common.drive.Motors.MotorSupplier;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.*;


public final class CargoSystemV2 {

	public static class IntakeSubsystem implements Subsystem {

		private final MotorController motor;

		public IntakeSubsystem(int p) { this(new PWMVictorSPX(p)); }
		public IntakeSubsystem(MotorController m) { this.motor = m; }
		public<M extends MotorController> IntakeSubsystem(int p, MotorSupplier<M> t) { this(t.create(p)); }
		public<M extends MotorController> IntakeSubsystem(int p, boolean invt, MotorSupplier<M> t) {
			this(t.create(p));
			this.motor.setInverted(invt);
		}

		public void set(double s, IntakeCommand c) {
			this.motor.set(s);
		}
		public void setVoltage(double v, IntakeCommand c) {
			this.motor.setVoltage(v);
		}
		public void stop(IntakeCommand c) {
			this.motor.stopMotor();
		}
		public double getLast(IntakeCommand c) {
			return this.motor.get();
		}

		public static abstract class IntakeCommand extends CommandBase {

			protected final IntakeSubsystem intake_sys;
			protected IntakeCommand(IntakeSubsystem i) {
				this.intake_sys = i;
				super.addRequirements(i);
			}
			@Override public void end(boolean i) { this.intake_sys.stop(this); }


		}


	}
	public static class TransferSubsystem implements Subsystem {

		private final MotorControllerGroup motors;
		private final DigitalInput input, output;
		private int cargo_cnt = 0;
		private boolean last_input = false, last_output = false;

		public TransferSubsystem(MotorController... ms) { this(null, null, ms); }
		public TransferSubsystem(DigitalInput i, DigitalInput o, MotorController... ms) {
			this.motors = new MotorControllerGroup(ms);
			this.input = i;
			this.output = o;
			if(i != null && o != null) {
				this.register();
			}
		}
		public TransferSubsystem(int... ps) {
			this(null, null, Motors.pwm_victorspx, ps);
		}
		public TransferSubsystem(int i, int o, int... ps) {
			this(new DigitalInput(i), new DigitalInput(o), Motors.pwm_victorspx, ps);
			this.register();
		}
		public<M extends MotorController> TransferSubsystem(MotorSupplier<M> t, int... ps) {
			this(null, null, t, ps);
		}
		public<M extends MotorController> TransferSubsystem(int i, int o, MotorSupplier<M> t, int... ps) {
			this(new DigitalInput(i), new DigitalInput(o), t, ps);
			this.register();
		}
		public<M extends MotorController> TransferSubsystem(DigitalInput i, DigitalInput o, MotorSupplier<M> t, int... ps) {
			MotorController[] temp = new MotorController[ps.length];
			for(int k = 0; k < ps.length; k++) {
				temp[k] = t.create(ps[k]);
			}
			this.motors = new MotorControllerGroup(temp);
			this.input = i;
			this.output = o;
		}

		@Override public void periodic() {
			//if(this.input != null && this.output != null) {	// should be safe because of check within constructors -> uncomment if issue
				if(this.last_input && !this.input.get()) {
					this.cargo_cnt++;
				}
				if(this.last_output && !this.output.get()) {
					this.cargo_cnt--;
				}
				this.last_input = this.input.get();
				this.last_output = this.output.get();
			//}
		}

		public void set(double s, TransferCommand c) {
			this.motors.set(s);
		}
		public void setVoltage(double v, TransferCommand c) {
			this.motors.setVoltage(v);
		}
		public void stop(TransferCommand c) {
			this.motors.stopMotor();
		}
		public boolean getCurrentInput() {
			if(this.input != null) {
				return this.input.get();
			}
			return false;
		}
		public boolean getCurrentOutput() {
			if(this.output != null) {
				return this.output.get();
			}
			return false;
		}
		public int getCargoCount() {
			return this.cargo_cnt;
		}

		public static class TransferCommand extends CommandBase {

			protected final TransferSubsystem transfer_sys;
			protected TransferCommand(TransferSubsystem t) {
				this.transfer_sys = t;
				super.addRequirements(t);
			}
			@Override public void end(boolean i) { this.transfer_sys.stop(this); }


		}


	}
	public static class ShooterSubsystem implements Subsystem {

		private final WPI_TalonFX main, secondary;
		private final MotorController feed, base_main;

		private void configureFalcons() {
			if(this.main != null) {
				this.main.configFactoryDefault();
				this.main.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
				if(this.secondary != null) {
					this.secondary.configFactoryDefault();
					this.secondary.follow(this.main);
					this.secondary.setInverted(InvertType.OpposeMaster);
				}
			}
		}

		public ShooterSubsystem(int m) {
			this(new WPI_TalonFX(m));
		}
		public<Mf extends MotorController> ShooterSubsystem(int m, MotorSupplier<Mf> t, int f) {
			this(new WPI_TalonFX(m), t.create(f));
		}
		public<Mf extends MotorController> ShooterSubsystem(int m, int s, MotorSupplier<Mf> t, int f) {
			this(new WPI_TalonFX(m), new WPI_TalonFX(s), t.create(f));
		}
		public<Ms extends MotorController> ShooterSubsystem(MotorSupplier<Ms> t, int m) {
			this(t.create(m));
		}
		public<Mf extends MotorController, Ms extends MotorController> ShooterSubsystem(MotorSupplier<Ms> ts, int m, MotorSupplier<Mf> tf, int f) {
			this(ts.create(m), tf.create(f));
		}
		public<Mf extends MotorController, Ms extends MotorController> ShooterSubsystem(MotorSupplier<Ms> ts, int m, int s, MotorSupplier<Mf> tf, int f) {
			this(ts.create(m), ts.create(s), tf.create(f));
		}
		public ShooterSubsystem(WPI_TalonFX m) { this(m, null, null); }
		public ShooterSubsystem(WPI_TalonFX m, MotorController f) { this(m, null, f); }
		public ShooterSubsystem(WPI_TalonFX m, WPI_TalonFX s, MotorController f) {
			this.main = m;
			this.secondary = s;
			this.feed = f;
			if(!(m == null || s == null)) {
				this.base_main = new MotorControllerGroup(m, s);
			} else {
				this.base_main = m;
			}
			this.configureFalcons();

		}
		public ShooterSubsystem(MotorController m) { this(m, null, null); }
		public ShooterSubsystem(MotorController m, MotorController f) { this(m, null, f); }
		public ShooterSubsystem(MotorController m, MotorController s, MotorController f) {
			this.main = null;
			this.secondary = null;
			this.feed = f;
			if(!(m == null || s == null)) {
				this.base_main = new MotorControllerGroup(m, s);
			} else {
				this.base_main = m;
			}
		}

		public void setFeed(double s, ShooterCommand c) {
			if(this.feed != null) {
				this.feed.set(s);
			}
		}
		public void setFeedVoltage(double v, ShooterCommand c) {
			if(this.feed != null) {
				this.feed.setVoltage(v);
			}
		}
		public void stopFeed(ShooterCommand c) {
			if(this.feed != null) {
				this.feed.stopMotor();
			}
		}
		public void setShooter(double s, ShooterCommand c) {
			if(this.main != null) {
				this.main.set(ControlMode.PercentOutput, s);
			} else if(this.base_main != null) {
				this.base_main.set(s);
			}
		}
		public void setShooterVoltage(double v, ShooterCommand c) {
			if(this.main != null) {
				this.main.setVoltage(v);
			} else if(this.base_main != null) {
				this.base_main.setVoltage(v);
			}
		}
		public void setShooterVelocity(double vr, ShooterCommand c) {
			if(this.main != null) {
				this.main.set(ControlMode.Velocity, vr);
			}
		}
		public void stopShooter(ShooterCommand c) {
			if(this.main != null) {
				this.main.stopMotor();
			} else if(this.base_main != null) {
				this.base_main.stopMotor();
			}
		}
		public double getShooterRawPosition() {
			if(this.main != null) {
				return this.main.getSelectedSensorPosition();
			}
			return 0.0;
		}
		public double getShooterRawVelocity() {
			if(this.main != null) {
				return this.main.getSelectedSensorVelocity();
			}
			return 0.0;
		}
		public double getShooterVelocity() {
			if(this.main != null) {
				return this.getShooterRawVelocity() * 
					(Constants.shooter_wheel_diameter_meters * Math.PI / Constants.falcon_encoder_units_per_revolution) * 10;
			}
			return 0.0;
		}

		public static abstract class ShooterCommand extends CommandBase {

			protected final ShooterSubsystem shooter_sys;
			protected ShooterCommand(ShooterSubsystem s) {
				this.shooter_sys = s;
				super.addRequirements(s);
			}
			@Override public void end(boolean i) {
				this.shooter_sys.stopFeed(this);
				this.shooter_sys.stopShooter(this);
			}


		}


	}



	private final IntakeSubsystem intake;
	private final TransferSubsystem transfer;
	private final ShooterSubsystem shooter;

	public CargoSystemV2(IntakeSubsystem i, TransferSubsystem t, ShooterSubsystem s) {
		this.intake = i;
		this.transfer = t;
		this.shooter = s;
	}

	public BasicIntake basicIntake(double s) { return new BasicIntake(this.intake, s); }



	public static class BasicIntake extends IntakeSubsystem.IntakeCommand {

		private final DoubleSupplier speed;
		private BasicIntake(IntakeSubsystem i, double s) { this(i, ()->s); }
		private BasicIntake(IntakeSubsystem i, DoubleSupplier s) {
			super(i);
			this.speed = s;
		}

		@Override public void initialize() {
			System.out.println(getName() + ": Running...");
		}
		@Override public void execute() {
			super.intake_sys.set(this.speed.getAsDouble(), this);
		}
		@Override public void end(boolean i) {
			System.out.println(i ? getName() + ": Terminated." : getName() + "Completed.");
			super.end(i);
		}
		@Override public boolean isFinished() { return false; }


	}


}