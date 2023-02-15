package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.networktables.*;

import com.revrobotics.*;

import frc.robot.commands.*;
import frc.robot.vision.java.*;
import frc.robot.team3407.ADIS16470_3X;
import frc.robot.team3407.Input.*;
import frc.robot.team3407.drive.*;
import frc.robot.team3407.drive.Types.DriveMode;
import frc.robot.team3407.commandbased.*;
import frc.robot.team3407.commandbased.EventTriggers.*;


/* TODO:
 x? ColorSensor for transfer system?
 - Test new hub/cargo targeting algos
 - Rate limit tank drive turning (somehow)
 x ramp-up for shooter
*/

public class Runtime extends TimedRobot {

	/* The input devices should always be setup with these port-id's on the driverstation so we can distinguish between input types */
	private final InputDevice
		input = new InputDevice(0),			// xbox controller

		stick_left = new InputDevice(1),	// acrade stick (left)
		stick_right = new InputDevice(2)	// arcade stick (right)
	;

	private final ADIS16470_3X
		spi_imu = new ADIS16470_3X()
	;
	private final ColorSensorV3
		front_color = new ColorSensorV3(Constants.front_colorsensor),
		back_color = new ColorSensorV3(Constants.back_colorsensor)
	;
	private final ClosedLoopDifferentialDrive
		drivebase = new ClosedLoopDifferentialDrive(
			Constants.drivebase_map_2022,
			this.spi_imu.getGyroAxis(ADIS16470_3X.IMUAxis.kZ),
			Constants.cl_params,
			Constants.cl_encoder_inversions
		)
	;
	private final CargoSystem
		cargo_sys = new CargoSystem(
			new CargoSystem.IntakeSubsystem(Constants.intake_port),
			new CargoSystem.TransferSubsystem(
				this.front_color, this.back_color,
				Constants.red_cargo_colormatch, Constants.blue_cargo_colormatch, Constants.nothing_colormatch,
				Motors.pwm_victorspx, Constants.transfer_ports
			),
			new CargoSystem.ShooterSubsystem(Motors.pwm_victorspx, Constants.shooter_port/*, Motors.pwm_victorspx, Constants.feed_port*/)
		)
	;
	private final ClimberSubsystem
		climb_sys = new ClimberSubsystem(
			Constants.climber_left_port,
			Constants.climber_right_port
		)
	;

	private final SendableChooser<Constants.StartingPose>
		starting_pose;
	private final SendableChooser<Command>
		auto_command = new SendableChooser<Command>();


	private final NetworkTable
		parameters_nt = NetworkTableInstance.getDefault().getTable("Parameters");
	private final NetworkTableEntry
		shooter_volts_nt = parameters_nt.getEntry("Shooter voltage"),
		intake_volts_nt = parameters_nt.getEntry("Intake voltage"),
		transfer_volts_nt = parameters_nt.getEntry("Transfer voltage")
	;





	public Runtime() {
		System.out.println("RUNTIME INITIALIZATION");

		this.drivebase.setSpeedScaling(Constants.teleop_drivebase_scaling);
		this.drivebase.setSpeedDeadband(Constants.teleop_drivebase_deadband);
		this.drivebase.setSpeedSquaring(Constants.teleop_drivebase_speed_squaring);

		this.cargo_sys.shooter.rateLimit(Constants.shooter_ramp_limit);
		this.cargo_sys.startAutomaticTransfer(Constants.transfer_voltage);

		this.shooter_volts_nt.setDouble(Constants.shooter_max_voltage);
		this.intake_volts_nt.setDouble(Constants.intake_voltage);
		this.transfer_volts_nt.setDouble(Constants.transfer_voltage);

		Constants.hub_camera_preset.initializeNT(parameters_nt, "Hub Presets");
		Constants.cargo_red_camera_preset.initializeNT(parameters_nt, "Red Cargo Presets");
		Constants.cargo_blue_camera_preset.initializeNT(parameters_nt, "Blue Cargo Presets");

		RapidReactVision.setHubPipelineMinRange(Constants.min_hub_range_inches);
		RapidReactVision.setHubPipelineMaxRange(Constants.max_hub_range_inches);

		Constants.vision_cargo.run();

		this.starting_pose = Constants.StartingPose.getSelectable(DriverStation.getAlliance());
		SmartDashboard.putData("Starting Position", this.starting_pose);

		this.auto_command.setDefaultOption("CL Auto", new Auto.ClosedLoop(this.drivebase, this.cargo_sys));
		this.auto_command.addOption("Basic-Taxi", new Auto.OpenLoop(this.drivebase, this.cargo_sys));
		this.auto_command.addOption("Test Deploy", this.cargo_sys.deployIntake());
		this.auto_command.addOption("Test Velocity Drive", this.drivebase.tankDriveVelocity(()->1.0, ()->1.0));
		this.auto_command.addOption("Test Active Park", this.drivebase.activePark(100.0));
		ClosedLoopDifferentialDrive.BalancePark bp = new ClosedLoopDifferentialDrive.BalancePark(
			this.drivebase, this.spi_imu.getGyroAxis(ADIS16470_3X.IMUAxis.kX), 100.0, 0.2
		);
		SmartDashboard.putData("Balance Park", bp);
		this.auto_command.addOption("Test Balance Park", bp);
		SmartDashboard.putData("Auto Command", this.auto_command);

		SmartDashboard.putData("IMU", this.spi_imu);
		SmartDashboard.putData("DriveBase", this.drivebase);
	}

	@Override public void robotInit() {

		NetworkTableInstance.getDefault().setUpdateRate(1.0 / 30.0);

		new Trigger(()->VisionServer.isConnected()).whenActive(
			new LambdaCommand(()->System.out.println("VisionServer Connected!"))
		);
		AutonomousTrigger.Get().whenActive(()->this.auto_command.getSelected().schedule());
		EnabledTrigger.Get().whenActive(new LambdaCommand.Singular(()->this.drivebase.setInitial(this.starting_pose.getSelected().pose)));

		if(this.input.isConnected()) {
			this.xboxControls();
			System.out.println("Xbox Bindings Initialized.");
		} else if(this.stick_left.isConnected() && this.stick_right.isConnected()) {
			this.arcadeControls();
			System.out.println("Arcade Bindings Initialized.");
		} else {
			new Thread(()->{
				for(;;) {
					try{ Thread.sleep(500); }	// half a second
					catch(InterruptedException e) { System.out.println(e.getMessage()); }
					if(this.input.isConnected()) {
						this.xboxTestControls();
						//this.xboxControls();
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

		// ColorMatch matcher = new ColorMatch();
		// matcher.addColorMatch(Constants.blue_cargo_colormatch);
		// matcher.addColorMatch(Constants.red_cargo_colormatch);
		// matcher.addColorMatch(Constants.nothing_colormatch);
		// //matcher.setConfidenceThreshold(0.92);
		// TestTrigger.Get().whileActiveOnce(
		// 	new LambdaCommand.Continuous(
		// 		()->{
		// 			Color
		// 				fraw = this.front_color.getColor(),
		// 				braw = this.back_color.getColor()
		// 			;
		// 			ColorMatchResult r = matcher.matchColor(fraw);
		// 			if(r == null) {
		// 				System.out.print("F: Null\t");
		// 			} else {
		// 				System.out.print(
		// 					"F: " + (r.color == Constants.blue_cargo_colormatch ? "Blue" : 
		// 						r.color == Constants.red_cargo_colormatch ? "Red" : "Noth") + '\t'
		// 				);
		// 			}					
		// 			r = matcher.matchColor(braw);
		// 			if(r == null) {
		// 				System.out.println("B: Null");
		// 			} else {
		// 				System.out.println(
		// 					"B: " + (r.color == Constants.blue_cargo_colormatch ? "Blue" : 
		// 						r.color == Constants.red_cargo_colormatch ? "Red" : "Noth")
		// 				);
		// 			}
					
		// 			// System.out.println(
		// 			// 	"F: {r: " + (int)(fraw.red * 1000) + " g: " + (int)(fraw.green * 1000) + " g: " + (int)(fraw.blue * 1000) +
		// 			// 	"}\tB: {r: " + (int)(braw.red * 1000) + " g: " + (int)(braw.green * 1000) + " g: " + (int)(braw.blue * 1000) + "}"
		// 			// );
		// 		}
		// 	)
		// );

	}
	@Override public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override public void disabledInit() {}
	@Override public void disabledPeriodic() {}
	@Override public void disabledExit() {}

	@Override public void autonomousInit() {}
	@Override public void autonomousPeriodic() {}
	@Override public void autonomousExit() {}

	@Override public void teleopInit() {}
	@Override public void teleopPeriodic() {}
	@Override public void teleopExit() {}

	@Override public void testInit() {}
	@Override public void testPeriodic() {}
	@Override public void testExit() {}








	public void xboxTestControls() {
		Trigger a = Xbox.Digital.A.getToggleFrom(this.input);
		Trigger b = Xbox.Digital.B.getToggleFrom(this.input);
		Trigger x = Xbox.Digital.X.getToggleFrom(this.input);
		ServoTest.ConfigServo s = new ServoTest.ConfigServo(9, 0.5, 2.5);
		TeleopTrigger.Get().and(a).and(b.negate()).and(x.negate()).whenActive(
			new ServoTest.Angle(s, 45)
		);
		TeleopTrigger.Get().and(b).and(a.negate()).and(x.negate()).whenActive(
			new ServoTest.Percentage(s, 0.75)
		);
		TeleopTrigger.Get().and(x).and(a.negate()).and(b.negate()).whenActive(
			new ServoTest.Percentage(s,
				()->Xbox.Analog.RX.getValueOf(this.input) * 0.5 + 0.5	// [-1,1] --> [0,1]
			)
		);
	}

	private void xboxControls() {	// setup bindings for xbox controller

		Xbox.Digital		// the button binds
			shoot_toggle =			Xbox.Digital.LB,
			actuate =				Xbox.Digital.RB,
			hub_assist_toggle =		Xbox.Digital.BACK,
			cargo_assist_toggle =	Xbox.Digital.START,
			change_drivemode =		Xbox.Digital.RS,
			change_camera =			Xbox.Digital.LS,
			climb_toggle =			Xbox.Digital.A,
			transfer =				Xbox.Digital.B,
			//alt_vision =			Xbox.Digital.X,
			invert =				Xbox.Digital.Y,
			pipe_increment =		Xbox.Digital.DR,
			pipe_decrement =		Xbox.Digital.DL,
			toggle_vision =			Xbox.Digital.DB,
			toggle_stats =			Xbox.Digital.DT
		;

		DigitalSupplier dm = change_drivemode.getPressedSupplier(this.input);
		TeleopTrigger.Get().whenActive(
			Constants.vision_driving
		).whenActive(	// we can't compose a sequentialcommandgroup here because the modedrive command is used elsewhere
			// this.drivebase.modeDrive(
			// 	Xbox.Analog.LX.getExponentialLimitedSupplier(this.input, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	Xbox.Analog.LY.getExponentialLimitedSupplier(this.input, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	Xbox.Analog.LT.getExponentialLimitedSupplier(this.input, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	Xbox.Analog.RX.getExponentialLimitedSupplier(this.input, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	Xbox.Analog.RY.getExponentialLimitedSupplier(this.input, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	Xbox.Analog.RT.getExponentialLimitedSupplier(this.input, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	dm, dm		// supply the same input for inc and dec so that wrap-around is used instead
			// )	// schedule mode drive when in teleop mode
			this.drivebase.tankDriveVelocity(	// use this code for velocity drive
				Xbox.Analog.LY.getDriveInputSupplier(this.input,
					Constants.teleop_drivebase_deadband, -2.5, 1.0
				),
				Xbox.Analog.RY.getDriveInputSupplier(this.input,
					Constants.teleop_drivebase_deadband, -2.5, 1.0
				)
			)
		).whileActiveOnce(
			new ClimberSubsystem.HoldToggleControl.FullLoop(
				this.climb_sys, climb_toggle.getSupplier(this.input),
				Constants.climber_extend_voltage,
				Constants.climber_retract_voltage,
				Constants.climber_hold_ext_voltage,
				Constants.climber_hold_ret_voltage
			)
		);
		// limit drive options
		this.drivebase.modeDrive().setDriveOptions(new DriveMode[]{DriveMode.TANK, DriveMode.ARCADE});

		toggle_vision.getCallbackFrom(this.input).whenActive(VisionSubsystem.ToggleProcessing.Get());
		toggle_stats.getCallbackFrom(this.input).whenActive(VisionSubsystem.ToggleStatistics.Get());
		pipe_increment.getCallbackFrom(this.input).whenActive(VisionSubsystem.IncrementPipeline.Get());
		pipe_decrement.getCallbackFrom(this.input).whenActive(VisionSubsystem.DecrementPipeline.Get());
		change_camera.getCallbackFrom(this.input).whenPressed(VisionSubsystem.IncrementCamera.Get());

		StaticTrigger
			hub_assist_state = new StaticTrigger(false),
			cargo_assist_state = new StaticTrigger(false)
		;
		Trigger
			teleop_trigger = TeleopTrigger.Get(),

			shoot_trigger = shoot_toggle.getToggleFrom(this.input),
			actuate_trigger = actuate.getCallbackFrom(this.input),
			transfer_trigger = transfer.getCallbackFrom(this.input),
			invert_trigger = invert.getCallbackFrom(this.input),
			hub_assist_trigger = hub_assist_toggle.getToggleFrom(this.input),
			cargo_assist_trigger = cargo_assist_toggle.getToggleFrom(this.input),

			routines_trigger = hub_assist_state.or(cargo_assist_state)
		;

	// when the override button is not pressed...
		// and the shooter is toggled on
		teleop_trigger.and(shoot_trigger).and(invert_trigger.negate()).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.managedShoot(
				//actuate.getSupplier(this.stick_right),
				()->false,
				Constants.feed_voltage,
				Constants.shooter_max_voltage
			)
		);
		// and the shooter is toggled off
		teleop_trigger.and(shoot_trigger.negate()).and(invert_trigger.negate()).and(actuate_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicIntake(Constants.intake_voltage)
		);
	// when the override button is pressed...
		// and the shooter is toggled on
		teleop_trigger.and(shoot_trigger).and(invert_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.managedShoot(
				//actuate.getSupplier(this.stick_right),
				()->false,
				Constants.feed_voltage * -1,
				Constants.shooter_max_voltage * -1
			)
		);
		// and the shooter is toggled off
		teleop_trigger.and(shoot_trigger.negate()).and(invert_trigger).and(actuate_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicIntake(Constants.intake_voltage * -1)
		);

		// transfer override
		teleop_trigger.and(transfer_trigger.and(invert_trigger).or(shoot_trigger.and(actuate_trigger).and(invert_trigger))).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicTransfer(Constants.transfer_voltage * -1)
		);
		teleop_trigger.and(shoot_trigger.and(actuate_trigger).and(invert_trigger.negate())).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicTransfer(Constants.transfer_voltage)
		);

		teleop_trigger.and(hub_assist_trigger).and(cargo_assist_state.negate()).whenActive(
			()->{
				hub_assist_state.enable();
				System.out.println("\tHUB ASSIST RUNNING...");
				this.drivebase.modeDrive().cancel();
				Constants.vision_hub.run();
			}
		).whileActiveOnce(
			new ParallelCommandGroup(
				this.cargo_sys.visionShoot(							// control the shooter with velocity determined by vision
					//actuate.getSupplier(this.input),				// press right trigger to feed
					()->false,
					Constants.feed_voltage,
					Constants.inches2volts_shooter
				),
				// new RapidReactVision.HubAssistRoutine(
				// 	this.drivebase,
				// 	()->Xbox.Analog.RT.getValueOf(this.input) - Xbox.Analog.LT.getValueOf(this.input),
				// 	3.0, 10.0	// max turning voltage and max voltage ramp
				new RapidReactVision.HubAssistRoutineV2(
					this.drivebase, this.drivebase.modeDrive(), 
					Constants.hub_targeting_inches,
					40,
					Constants.auto_max_forward_voltage,
					Constants.auto_max_turn_voltage,
					Constants.auto_max_voltage_ramp
				)
			)
		).whenInactive(
			()->{
				this.drivebase.modeDrive().schedule();
				Constants.vision_driving.run();
				hub_assist_state.disable();
				System.out.println("\tHUB ASSIST TERMINATED.");
			}
		).and(actuate_trigger).whileActiveOnce(
			this.cargo_sys.basicTransfer(Constants.transfer_voltage)
		);

		teleop_trigger.and(cargo_assist_trigger).and(hub_assist_state.negate()).whenActive(
			()->{
				cargo_assist_state.enable();
				System.out.println("\tCARGO ASSIST RUNNING...");
				this.drivebase.modeDrive().cancel();
				Constants.vision_cargo.run();
			}
		).whileActiveOnce(
			new RapidReactVision.CargoAssistRoutine(
				this.drivebase,
				this.drivebase.modeDrive(),
				this.cargo_sys.managedIntake(Constants.intake_voltage),
				DriverStation.getAlliance(),
				Constants.cargo_follow_target_inches,
				Constants.auto_max_forward_voltage + 1.0,
				Constants.auto_max_turn_voltage + 1.0,
				Constants.auto_max_voltage_ramp
			)
		).whenInactive(
			()->{
				this.drivebase.modeDrive().schedule();
				Constants.vision_driving.run();
				System.out.println("\tCARGO ASSIST TERMINATED.");
				cargo_assist_state.disable();
			}
		);

	}








	public void arcadeControls() {

		Attack3.Digital		// the button binds
			shoot_toggle =			Attack3.Digital.TRI,	// left stick
			hub_assist_toggle =		Attack3.Digital.TT,		// left stick
			change_camera =			Attack3.Digital.TR,		// left stick
			change_drivemode =		Attack3.Digital.TL,		// left stick

			actuate =				Attack3.Digital.TRI,	// right stick
			cargo_assist_toggle =	Attack3.Digital.TT,		// right stick
			invert =				Attack3.Digital.TL,		// right stick
			climb_toggle = 			Attack3.Digital.TB,		// right stick

			toggle_vision =			Attack3.Digital.B1,
			toggle_stats =			Attack3.Digital.B2,
			pipe_increment =		Attack3.Digital.B4,
			pipe_decrement =		Attack3.Digital.B3
		;

		DigitalSupplier dm = change_drivemode.getPressedSupplier(this.input);
		TeleopTrigger.Get().whenActive(
			Constants.vision_driving
		).whenActive(		// schedule mode drive when in teleop mode
			// this.drivebase.modeDrive(
			// 	Attack3.Analog.X.getExponentialLimitedSupplier(this.stick_left, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	Attack3.Analog.Y.getExponentialLimitedSupplier(this.stick_left, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	Attack3.Analog.X.getExponentialLimitedSupplier(this.stick_right, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	Attack3.Analog.Y.getExponentialLimitedSupplier(this.stick_right, Constants.teleop_max_input_ramp, Constants.teleop_input_power),
			// 	dm, dm	// supply the same input for inc and dec so that wrap-around is used instead
			// )
			this.drivebase.tankDriveVelocity(	// use this code for velocity drive
				Attack3.Analog.Y.getDriveInputSupplier(this.stick_left,
					Constants.teleop_drivebase_deadband, -2.5, 1.0
				),
				Attack3.Analog.Y.getDriveInputSupplier(this.stick_right,
					Constants.teleop_drivebase_deadband, -2.5, 1.0
				)
			)
		).whileActiveOnce(		// start climb controls
			new ClimberSubsystem.HoldToggleControl(
				this.climb_sys, climb_toggle.getSupplier(this.input),
				Constants.climber_extend_voltage,
				Constants.climber_retract_voltage,
				Constants.climber_hold_ext_voltage,
				Constants.climber_hold_ret_voltage
			)
		);
		// limit drive options
		this.drivebase.modeDrive().setDriveOptions(new DriveMode[]{DriveMode.TANK, DriveMode.ARCADE});

		toggle_vision.getCallbackFrom(this.stick_left).whenActive(VisionSubsystem.ToggleProcessing.Get());
		toggle_stats.getCallbackFrom(this.stick_left).whenActive(VisionSubsystem.ToggleStatistics.Get());
		pipe_increment.getCallbackFrom(this.stick_left).whenActive(VisionSubsystem.IncrementPipeline.Get());
		pipe_decrement.getCallbackFrom(this.stick_left).whenActive(VisionSubsystem.DecrementPipeline.Get());
		change_camera.getCallbackFrom(this.stick_left).whenPressed(VisionSubsystem.IncrementCamera.Get());

		StaticTrigger
			hub_assist_state = new StaticTrigger(false),
			cargo_assist_state = new StaticTrigger(false)
		;
		Trigger
			teleop_trigger = TeleopTrigger.Get(),

			shoot_trigger = shoot_toggle.getToggleFrom(this.stick_left),
			actuate_trigger = actuate.getCallbackFrom(this.stick_right),
			invert_trigger = invert.getCallbackFrom(this.stick_right),
			hub_assist_trigger = hub_assist_toggle.getToggleFrom(this.stick_left),
			cargo_assist_trigger = cargo_assist_toggle.getToggleFrom(this.stick_right),

			routines_trigger = hub_assist_state.or(cargo_assist_state)
		;

	// when the override button is not pressed...
		// and the shooter is toggled on
		teleop_trigger.and(shoot_trigger).and(invert_trigger.negate()).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.managedShoot(
				//actuate.getSupplier(this.stick_right),
				()->false,
				Constants.feed_voltage,
				Constants.shooter_max_voltage
			)
		);
		// and the shooter is toggled off
		teleop_trigger.and(shoot_trigger.negate()).and(invert_trigger.negate()).and(actuate_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicIntake(Constants.intake_voltage)
		);
	// when the override button is pressed...
		// and the shooter is toggled on
		teleop_trigger.and(shoot_trigger).and(invert_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.managedShoot(
				//actuate.getSupplier(this.stick_right),
				()->false,
				Constants.feed_voltage * -1,
				Constants.shooter_max_voltage * -1
			)
		);
		// and the shooter is toggled off
		teleop_trigger.and(shoot_trigger.negate()).and(invert_trigger).and(actuate_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicIntake(Constants.intake_voltage * -1)
		);

		// transfer override
		teleop_trigger.and(shoot_trigger.and(actuate_trigger).and(invert_trigger.negate())).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicTransfer(Constants.transfer_voltage)
		);
		teleop_trigger.and(shoot_trigger.and(actuate_trigger).and(invert_trigger)).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicTransfer(Constants.transfer_voltage * -1)
		);

		teleop_trigger.and(hub_assist_trigger).and(cargo_assist_state.negate()).whenActive(
			()->{
				hub_assist_state.enable();
				System.out.println("\tHUB ASSIST RUNNING...");
				this.drivebase.modeDrive().cancel();
				Constants.vision_hub.run();
			}
		).whileActiveOnce(
			new ParallelCommandGroup(
				this.cargo_sys.visionShoot(							// control the shooter with velocity determined by vision
					//actuate.getSupplier(this.stick_right),		// press right trigger to feed
					()->false,
					Constants.feed_voltage,
					Constants.inches2volts_shooter
				),
				// new RapidReactVision.HubAssistRoutine(
				// 	this.drivebase,
				// 	Attack3.Analog.X.getSupplier(this.stick_right),
				// 	4.0, 10.0	// max turning voltage and max voltage ramp
				// )
				new RapidReactVision.HubAssistRoutineV2(
					this.drivebase, this.drivebase.modeDrive(), 
					Constants.hub_targeting_inches,
					40,
					Constants.auto_max_forward_voltage,
					Constants.auto_max_turn_voltage,
					Constants.auto_max_voltage_ramp
				)
			)
		).whenInactive(
			()->{
				this.drivebase.modeDrive().schedule();
				Constants.vision_driving.run();
				hub_assist_state.disable();
				System.out.println("\tHUB ASSIST TERMINATED.");
			}
		).and(actuate_trigger).whileActiveOnce(
			this.cargo_sys.basicTransfer(Constants.transfer_voltage)
		);

		teleop_trigger.and(cargo_assist_trigger).and(hub_assist_state.negate()).whenActive(
			()->{
				cargo_assist_state.enable();
				System.out.println("\tCARGO ASSIST RUNNING...");
				this.drivebase.modeDrive().cancel();
				Constants.vision_cargo.run();
			}
		).whileActiveOnce(
			new RapidReactVision.CargoAssistRoutine(
				this.drivebase,
				this.drivebase.modeDrive(),
				this.cargo_sys.managedIntake(Constants.intake_voltage),
				DriverStation.getAlliance(),
				Constants.cargo_follow_target_inches,
				Constants.auto_max_forward_voltage + 1.0,
				Constants.auto_max_turn_voltage + 1.0,
				Constants.auto_max_voltage_ramp
			)
		).whenInactive(
			()->{
				this.drivebase.modeDrive().schedule();
				Constants.vision_driving.run();
				System.out.println("\tCARGO ASSIST TERMINATED.");
				cargo_assist_state.disable();
			}
		);

	}


}