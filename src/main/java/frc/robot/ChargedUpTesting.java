package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.ColorSensorV3;

import frc.robot.team3407.ADIS16470_3X;
import frc.robot.team3407.ADIS16470_3X.IMUAxis;
import frc.robot.team3407.Input.*;
import frc.robot.team3407.commandbased.EventTriggers.*;


public final class ChargedUpTesting extends TimedRobot {

	private final class Robot implements Sendable {
		private final ADIS16470_3X
			imu_3x = new ADIS16470_3X();
		private final ColorSensorV3
			front_color = new ColorSensorV3(Constants.front_colorsensor),
			back_color = new ColorSensorV3(Constants.back_colorsensor);

		private final ClosedLoopDifferentialDrive
			drivebase = new ClosedLoopDifferentialDrive(
				Constants.drivebase_map_2022,
				this.imu_3x.getGyroAxis(IMUAxis.kZ),
				Constants.cl_params, Constants.cl_encoder_inversions
			);
		// CargoSubsystem and ClimberSubsystem went here.

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleArrayProperty("Front Colorsensor/Color [R,G,B]", ()->{
				Color c = this.front_color.getColor();
				return new double[]{ c.red, c.green, c.blue };
			}, null);
			b.addDoubleArrayProperty("Back Colorsensor/Color [R,G,B]", ()->{
				Color c = this.back_color.getColor();
				return new double[]{ c.red, c.green, c.blue };
			}, null);
			b.addDoubleProperty("Front Colorsensor/Proximity", this.front_color::getProximity, null);
			b.addDoubleProperty("Back Colorsensor/Proximity", this.back_color::getProximity, null);
		}
		public void startLogging() {
			SmartDashboard.putData("Robot", this);
			SmartDashboard.putData("Robot/Drivebase", this.drivebase);
			SmartDashboard.putData("Robot/IMU", this.imu_3x);
		}
	}

	private final Robot
		robot = new Robot();

	private final InputDevice
		xbox = new InputDevice(0),
		stick_left = new InputDevice(1),
		stick_right = new InputDevice(2),
		bbox = new InputDevice(3)
	;

	public static final double
		IMU_RATE_FILTER = 0.25,
		ACTIVE_PARK_VOLTS_PER_METER = 100.0,
		BALANCE_PARK_VOLTS_PER_DEGREE = 0.2,
		AUTO_PAD_INCLINE_VELOCITY = 0.2;


	public ChargedUpTesting() {
		this.robot.imu_3x.configRateFilter(IMU_RATE_FILTER);
	}

	@Override	// put init actions here rather than constructor because some parts of wpilib are not ready when the constructor is called
	public void robotInit() {
		DataLogManager.start();
		NetworkTableInstance.getDefault().setUpdateRate(1.0 / 30.0);
		this.robot.startLogging();

		Gyro pitch = this.robot.imu_3x.getGyroAxis(IMUAxis.kX);

		SendableChooser<Command> auto = new SendableChooser<Command>();
		auto.addOption("Active Park", new ActivePark(
			this.robot.drivebase,
			ACTIVE_PARK_VOLTS_PER_METER
		));
		auto.addOption("Balancing Park", new BalancePark(
			this.robot.drivebase, pitch,
			ACTIVE_PARK_VOLTS_PER_METER,
			BALANCE_PARK_VOLTS_PER_DEGREE
		));
		AutoPad autopad = new AutoPad(
			this.robot.drivebase, pitch,
			AUTO_PAD_INCLINE_VELOCITY
		);
		SmartDashboard.putData("Commands/AutoPad", autopad);
		auto.addOption("Auto Pad Balance", autopad);

		SmartDashboard.putData("Auto Command", auto);
		AutonomousTrigger.Get().whenActive(()->auto.getSelected().schedule());

		if(this.xbox.isConnected()) {
			this.xboxControls();
			System.out.println("Xbox Bindings Initialized.");
		} else if(this.stick_left.isConnected() && this.stick_right.isConnected()) {
			this.arcadeControls();
			System.out.println("Arcade Bindings Initialized.");
		} else {
			new Thread(()->{
				System.out.println("No inputs found. Waiting for connections...");
				for(;;) {
					try{ Thread.sleep(500); }	// half a second
					catch(InterruptedException e) { System.out.println(e.getMessage()); }
					if(this.xbox.isConnected()) {
						this.xboxControls();
						System.out.println("Xbox Bindings Initialized.");
						return;
					} else if(this.stick_left.isConnected() && this.stick_right.isConnected()) {
						this.arcadeControls();
						System.out.println("Arcade Bindings Initialized.");
						return;
					}
				}
			}).start();
		}
	}
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}



	private void xboxControls() {
		TeleopTrigger.Get().whenActive(
			this.robot.drivebase.tankDriveVelocity(
				Xbox.Analog.LY.getDriveInputSupplier(this.xbox,
					Constants.teleop_drivebase_deadband, -2.5, 1.0),
				Xbox.Analog.RY.getDriveInputSupplier(this.xbox,
					Constants.teleop_drivebase_deadband, -2.5, 1.0)
			)
		);
		TestTrigger.Get().or(AutonomousTrigger.Get()).whenActive(
			new ServoTest.Percentage(this.servo,
				()->Xbox.Analog.RY.getValueOf(this.xbox) * 0.5 + 0.5)
		);
	}
	private void arcadeControls() {
		TeleopTrigger.Get().whenActive(
			this.robot.drivebase.tankDriveVelocity(
				Attack3.Analog.Y.getDriveInputSupplier(this.stick_left,
					Constants.teleop_drivebase_deadband, -2.5, 1.0),
				Attack3.Analog.Y.getDriveInputSupplier(this.stick_right,
					Constants.teleop_drivebase_deadband, -2.5, 1.0)
			)
		);
		TestTrigger.Get().or(AutonomousTrigger.Get()).whenActive(
			new ServoTest.Percentage(this.servo,
				()->Attack3.Analog.Y.getValueOf(this.stick_right) * 0.5 + 0.5)
		);
	}




	public static class ActivePark extends ClosedLoopDifferentialDrive.CLDriveCommand {

		private final double volts_per_meter;
		private double linit, rinit;

		public ActivePark(ClosedLoopDifferentialDrive db, double p) { // p is the proportional gain, in volts per meter [error]
			super(db);
			this.volts_per_meter = p;
		}

		@Override
		public void initialize() {
			this.linit = super.drivebase_cl.getLeftPositionMeters();
			this.rinit = super.drivebase_cl.getRightPositionMeters();
		}
		@Override
		public void execute() {
			double le = super.drivebase_cl.getLeftPositionMeters() - this.linit;
			double re = super.drivebase_cl.getRightPositionMeters() - this.rinit;
			super.setDriveVoltage(
				-le * this.volts_per_meter,     // we are assuming that positive position for the encoders is the same direction as positive voltage
				-re * this.volts_per_meter
			);
		}
		@Override
		public void end(boolean interrupted) {
			super.setDriveVoltage(0.0, 0.0);
		}
		@Override
		public boolean isFinished() {
			return false;
		}

	}
	public static class BalancePark extends ClosedLoopDifferentialDrive.CLDriveCommand {

		private final Gyro pitch_axis;
		private final double
			volts_per_meter,
			volts_per_degree;
		private double
			left_balanced, right_balanced,
			pitch_init;

		private double out = 0.0;

		public BalancePark(ClosedLoopDifferentialDrive db, Gyro pitch, double pv, double pa) {
			super(db);
			this.pitch_axis = pitch;
			this.volts_per_meter = pv;
			this.volts_per_degree = pa;
		}

		@Override
        public void initialize() {
            this.left_balanced = super.drivebase_cl.getLeftPositionMeters();
            this.right_balanced = super.drivebase_cl.getRightPositionMeters();
			this.pitch_init = this.pitch_axis.getAngle();
        }
		@Override
		public void execute() {
			double le = super.drivebase_cl.getLeftPositionMeters() - this.left_balanced;
			double re = super.drivebase_cl.getRightPositionMeters() - this.right_balanced;
			double ae = this.pitch_axis.getAngle() - this.pitch_init;
			this.out = -ae * this.volts_per_degree;
			super.setDriveVoltage(
				this.out,
				this.out
			);
			// super.setDriveVoltage(
			// 	(-le * this.volts_per_meter) + (ae * volts_per_degree),
			// 	(-re * this.volts_per_meter) + (ae * volts_per_degree)
			// );
			// if(Math.abs(ae) < 1e-3) {
			// 	this.left_balanced = super.drivebase_cl.getLeftPositionMeters();
           	// 	this.right_balanced = super.drivebase_cl.getRightPositionMeters();
			// }
			// if(Math.abs(le) < 1e-5 && Math.abs(re) < 1e-5 && Math.abs(ae) > 1.0) {
			// 	this.pitch_init = this.pitch_axis.getAngle();
			// }
		}
		@Override
        public void end(boolean interrupted) {
            super.setDriveVoltage(0.0, 0.0);
        }
        @Override
        public boolean isFinished() {
            return false;
        }

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("AE Output Volts", ()->this.out, null);
		}

	}
	public static class AutoPad extends ClosedLoopDifferentialDrive.TankDriveVelocity {

		public static final double
			DEFAULT_INCLINE_VELOCITY = 0.2,
			DELTA_ANGLE_THRESH = 14.0,	// the charging pad main incline is 15 degrees, so give some room for error
			DELTA_ANGLE_RATE_THRESH = 20,
			STABLE_ANGLE_THRESH = 0.5,
			STABLE_VELOCITY_THRESH = 0.05;

		private static enum State {
			ENGAGING	("Engaging"),
			CLIMBING	("Climbing"),
			STABILIZING	("Stabilizing"),
			OVERSHOT	("Overshot"),
			STABLE		("Stablized");

			public final String desc;
			private State(String s) { this.desc = s; }
		}

		private final Gyro pitch;
		private double pitch_init = 0.0;
		private State state = State.STABLE;

		public AutoPad(ClosedLoopDifferentialDrive db, Gyro pa)
			{ this(db, pa, DEFAULT_INCLINE_VELOCITY); }
		public AutoPad(ClosedLoopDifferentialDrive db, Gyro pa, double iv) {
			super(db, ()->iv, ()->iv);
			this.pitch = pa;
		}

		@Override
		public void initialize() {
			super.initialize();
			this.state = State.ENGAGING;
			this.pitch_init = this.pitch.getAngle();
		}
		@Override
		public void execute() {
			switch(this.state) {
				case ENGAGING: {
					super.execute();
					if(pitch_init - this.pitch.getAngle() > DELTA_ANGLE_THRESH) {
						this.state = State.CLIMBING;
					}
					break;
				}
				case CLIMBING: {
					super.execute();
					if(this.pitch.getRate() < -DELTA_ANGLE_RATE_THRESH) {
						this.state = State.STABILIZING;
						// maybe store the position here, use position locking in the stabilization block?
					}
					break;
				}
				case STABILIZING: {
					super.applyVelocity(0.0, 0.0);	// or use position lock
					if(Math.abs(this.pitch_init - this.pitch.getAngle()) < STABLE_ANGLE_THRESH &&
						Math.abs(super.drivebase_cl.getLeftVelocity()) < STABLE_VELOCITY_THRESH &&
						Math.abs(super.drivebase_cl.getRightVelocity()) < STABLE_VELOCITY_THRESH)
					{
						this.state = State.STABLE;
					} else if(this.pitch_init - this.pitch.getAngle() < -DELTA_ANGLE_THRESH) {
						this.state = State.OVERSHOT;
					} else if(this.pitch_init - this.pitch.getAngle() > DELTA_ANGLE_THRESH) {
						this.state = State.CLIMBING;
					}
					break;
				}
				case OVERSHOT: {
					super.applyVelocity(
						-super.left.getAsDouble(),
						-super.right.getAsDouble()
					);
					if(this.pitch.getRate() > DELTA_ANGLE_RATE_THRESH) {
						this.state = State.STABILIZING;
					}
					break;
				}
				default:
				case STABLE: {
					break;
				}
			}
		}
		@Override
		public boolean isFinished() {
			return this.state == State.STABLE;
		}

		@Override
		public void initSendable(SendableBuilder b) {
			super.initSendable(b);
			b.addStringProperty("Control State", ()->this.state.desc, null);
		}

	}

	public static final class ServoTest {
	
		public static class ConfigServo extends Servo {
			public ConfigServo(int c, double min_ms, double max_ms, double center_ms) {
				super(c);
				super.setBounds(max_ms, 0, center_ms, 0, min_ms);
			}
			public ConfigServo(int c, double min_ms, double max_ms) {
				this(c, min_ms, max_ms, 0);
			}
		}
	
		public static class Percentage extends CommandBase {
			private final Servo servo;
			private final DoubleSupplier percent;
			private double last;
			public Percentage(Servo s, DoubleSupplier p) {
				this.servo = s;
				this.percent = p;
			}
			public Percentage(Servo s, double p) {
				this(s, ()->p);
			}
			@Override
			public void initialize() {
				this.last = this.percent.getAsDouble();
				this.servo.set(this.last);
			}
			@Override
			public void execute() {
				double n = this.percent.getAsDouble();
				if(this.last != n) {
					this.last = n;
					this.servo.set(n);
				}
			}
			@Override
			public void end(boolean i) {
				this.servo.setDisabled();
			}
		}
		public static class Angle extends CommandBase {
			private final Servo servo;
			private final DoubleSupplier angle;
			private double last;
			public Angle(Servo s, DoubleSupplier a) {
				this.servo = s;
				this.angle = a;
			}
			public Angle(Servo s, double a) {
				this(s, ()->a);
			}
			@Override
			public void initialize() {
				this.last = this.angle.getAsDouble();
				this.servo.set(this.last);
			}
			@Override
			public void execute() {
				double n = this.angle.getAsDouble();
				if(this.last != n) {
					this.last = n;
					this.servo.setAngle(n);
				}
			}
			@Override
			public void end(boolean i) {
				this.servo.setDisabled();
			}
		}
	
	}
	private final ServoTest.ConfigServo
		servo = new ServoTest.ConfigServo(9, 0.5, 2.5);

}