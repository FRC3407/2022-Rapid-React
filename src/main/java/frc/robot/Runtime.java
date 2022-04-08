package frc.robot;

import frc.robot.commands.*;
import frc.robot.modules.common.*;
import frc.robot.modules.common.drive.*;
import frc.robot.modules.common.Input.*;
import frc.robot.modules.common.EventTriggers.*;
import frc.robot.modules.vision.java.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
// import edu.wpi.first.math.trajectory.*;
// import edu.wpi.first.networktables.*;


/* TODO:
 x Split VisionServer into base and an extension that integrates command-based structure (this would extend SubsystemBase)
 x Verify/imporve singleton for each ^^^
 x Create "helper" container for methods at the start of DriveBase
 x Make better names for "DB#..." and split into port map and motorcontroller object containers (this would be an extension)
 x Commands for CLDifferential -> Ramsete controller (characterization first)
 x Update C++ VisionServer-Robot API
 - Methods/impelemtation to search DriverStation for a certain input (common.Input.InputDevice) and return object/port
 x max output/scaling method for drivebase
 - make a spreadsheet for camera presets (each pipeline) under different lighting conditions
 x fix controls being f'ed when not in sim mode and not connected on program startup
 - finalize/test Velocity-CL (TalonFX) shooter commands
 x Polish cargo manipulation controls
 x? AUTO!!!!
 x make AnalogSupplier and DigitalSupplier extend BooleanSupplier and DigitalSupplier respectively
 x? Path planning and drivebase cl (all of it...)

 x? Tune hubturn p-loop
 x Change camera params / configure switching when camera positions are finalized

 x Polish "Vision Assist" -> re-schedule operator-turning after overcorrection or "dead zone"
 - Auto -> "Hierarchy" of closed-loop
 x Add cargo-following routine to vision assist
*/

public class Runtime extends TimedRobot {

	/* The input devices should always be setup with these port-id's on the driverstation so we can distinguish between input types */
	private final InputDevice
		input = new InputDevice(0),			// xbox controller

		stick_left = new InputDevice(1),	// acrade stick (left)
		stick_right = new InputDevice(2);	// arcade stick (right)

	private final ADIS16470
		spi_imu = new ADIS16470();
	private final ClosedLoopDifferentialDrive
		drivebase = new ClosedLoopDifferentialDrive(
			Constants.drivebase_map_2022,
			this.spi_imu,
			Constants.cl_params,
			Constants.cl_encoder_inversions
		);
	private final CargoSystem
		cargo_sys = new CargoSystem(
			new CargoSystem.IntakeSubsystem(Constants.intake_port),
			new CargoSystem.TransferSubsystem(Constants.input_entering_dio, Constants.input_exiting_dio, Constants.transfer_ports),
			new CargoSystem.ShooterSubsystem(Motors.pwm_victorspx, Constants.shooter_port/*, Motors.pwm_victorspx, Constants.feed_port*/)
		);
	private final ClimberSubsystem
		climb_sys = new ClimberSubsystem(
			Constants.climber_left_port,
			Constants.climber_right_port
		);

	private final SendableChooser<Constants.StartingPose>
		starting_pose;
	private final SendableChooser<Command>
		auto_command = new SendableChooser<Command>();





	public Runtime() {
		System.out.println("RUNTIME INITIALIZATION");

		this.drivebase.setSpeedScaling(Constants.teleop_drivebase_scaling);
		this.drivebase.setSpeedDeadband(Constants.teleop_drivebase_deadband);
		this.drivebase.setSpeedSquaring(Constants.teleop_drivebase_speed_squaring);

		this.cargo_sys.startAutomaticTransfer(Constants.transfer_voltage);

		this.starting_pose = Constants.StartingPose.getSelectable(DriverStation.getAlliance());
		SmartDashboard.putData("Starting Position", this.starting_pose);

		this.auto_command.setDefaultOption("Basic-Taxi", new Auto.OpenLoop(this.drivebase, this.cargo_sys));
		this.auto_command.addOption("CL Auto", new Auto.ClosedLoop(this.drivebase, this.cargo_sys));
		SmartDashboard.putData("Auto Command", this.auto_command);
	}

	@Override public void robotInit() {

		new Trigger(()->VisionServer.isConnected()).whenActive(
			new LambdaCommand(()->System.out.println("Coprocessor Connected!"))
		);
		AutonomousTrigger.Get().whenActive(()->this.auto_command.getSelected().schedule());
		EnabledTrigger.Get().whenActive(new LambdaCommand.Singular(()->this.drivebase.setInitial(this.starting_pose.getSelected().pose)));

		if(this.input.isConnected()) {
			this.xboxControls();
			System.out.println("Xbox Bindings Initialized.");
		} else {
			this.input.connectionTrigger().whenActive(
				new LambdaCommand.Singular(()->{		// bindings should only be bound once
					this.xboxControls();
					System.out.println("Xbox Bindings Initialized.");
				}, true)
			); 
		}
		// if(this.stick_left.isConnected() && this.stick_right.isConnected()) {
		// 	this.arcadeControls();
		// 	System.out.println("Arcade Bindings Initialized.");
		// } else {
		// 	this.stick_left.connectionTrigger().and(this.stick_right.connectionTrigger()).whenActive(
		// 		new LambdaCommand.Singular(()->{
		// 			this.arcadeControls();
		// 			System.out.println("Arcade Bindings Initialized.");
		// 		}, true)
		// 	);
		// }

		// EnabledTrigger.Get().whileActiveOnce(	// beam break test
		// 	new Test.InputTest(
		// 		25, 	// 2 times per second @ a loop frequency of 50
		// 		()->this.cargo_sys.transfer.getCurrentInput(),
		// 		()->this.cargo_sys.transfer.getCurrentOutput()
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








	private void xboxControls() {	// setup bindings for xbox controller

		Xbox.Digital			// the button binds
			shoot_toggle =			Xbox.Digital.LB,
			actuate =				Xbox.Digital.RB,
			hub_assist_toggle =		Xbox.Digital.BACK,
			cargo_assist_toggle =	Xbox.Digital.START,
			drivemode_increment =	Xbox.Digital.RS,
			drivemode_decrement =	Xbox.Digital.LS,
			climb_toggle = 			Xbox.Digital.A,
			transfer =				Xbox.Digital.B,
			alt_vision =			Xbox.Digital.X,
			override =				Xbox.Digital.Y,
			camera_increment =		Xbox.Digital.DR,
			camera_decrement =		Xbox.Digital.DL,
			toggle_vision =			Xbox.Digital.DB,
			toggle_stats =			Xbox.Digital.DT
		;

		TeleopTrigger.Get().whenActive(
			Constants.vision_driving
		).whenActive(	// we can't compose a sequentialcommandgroup here because the modedrive command is used elsewhere
			this.drivebase.modeDrive(
				Xbox.Analog.LX.getLimitedSupplier(this.input, Constants.teleop_max_input_ramp),
				Xbox.Analog.LY.getLimitedSupplier(this.input, Constants.teleop_max_input_ramp),
				Xbox.Analog.LT.getLimitedSupplier(this.input, Constants.teleop_max_input_ramp),
				Xbox.Analog.RX.getLimitedSupplier(this.input, Constants.teleop_max_input_ramp),
				Xbox.Analog.RY.getLimitedSupplier(this.input, Constants.teleop_max_input_ramp),
				Xbox.Analog.RT.getLimitedSupplier(this.input, Constants.teleop_max_input_ramp),
				drivemode_increment.getPressedSupplier(this.input),
				drivemode_decrement.getPressedSupplier(this.input)
			)	// schedule mode drive when in teleop mode
		).whileActiveOnce(
			new ClimberSubsystem.HoldToggleControl(
				this.climb_sys, climb_toggle.getSupplier(this.input),
				Constants.climber_extend_voltage,
				Constants.climber_retract_voltage,
				Constants.climber_hold_voltage
			)
		);

		toggle_vision.getCallbackFrom(this.input).and(alt_vision.getCallbackFrom(this.input).negate()).whenActive(VisionSubsystem.ToggleProcessing.Get());
		toggle_stats.getCallbackFrom(this.input).and(alt_vision.getCallbackFrom(this.input).negate()).whenActive(VisionSubsystem.ToggleStatistics.Get());
		alt_vision.getCallbackFrom(this.input).and(toggle_stats.getCallbackFrom(this.input)).whenActive(VisionSubsystem.IncrementPipeline.Get());
		alt_vision.getCallbackFrom(this.input).and(toggle_vision.getCallbackFrom(this.input)).whenActive(VisionSubsystem.DecrementPipeline.Get());
		camera_increment.getCallbackFrom(this.input).whenPressed(VisionSubsystem.IncrementCamera.Get());
		camera_decrement.getCallbackFrom(this.input).whenPressed(VisionSubsystem.DecrementCamera.Get());

		StaticTrigger
			hub_assist_state = new StaticTrigger(false),
			cargo_assist_state = new StaticTrigger(false)
		;
		Trigger
			teleop_trigger = TeleopTrigger.Get(),

			shoot_trigger = shoot_toggle.getToggleFrom(this.input),
			actuate_trigger = actuate.getCallbackFrom(this.input),
			transfer_trigger = transfer.getCallbackFrom(this.input),
			override_trigger = override.getCallbackFrom(this.input),
			hub_assist_trigger = hub_assist_toggle.getToggleFrom(this.input),
			cargo_assist_trigger = cargo_assist_toggle.getToggleFrom(this.input),

			routines_trigger = hub_assist_state.or(cargo_assist_state)
		;

	// when the override button is not pressed...
		// and the shooter is toggled on
		teleop_trigger.and(shoot_trigger).and(override_trigger.negate()).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.managedShoot(
				//actuate.getSupplier(this.input),
				()->false,
				Constants.feed_voltage,
				Constants.shooter_default_voltage
			)
		);
		// and the shooter is toggled off
		teleop_trigger.and(shoot_trigger.negate()).and(override_trigger.negate()).and(actuate_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.managedIntake(Constants.intake_voltage)
		);
	// when the override button is pressed...
		// and the shooter is toggled on
		teleop_trigger.and(shoot_trigger).and(override_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicShoot(
				//actuate.getSupplier(this.input),
				()->false,
				Constants.feed_voltage,
				Constants.shooter_default_voltage
			)
		);
		// and the shooter is toggled off
		teleop_trigger.and(shoot_trigger.negate()).and(override_trigger).and(actuate_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicIntake(Constants.intake_voltage)
		);

		// transfer override
		teleop_trigger.and(transfer_trigger.and(override_trigger).or(shoot_trigger.and(actuate_trigger))).and(routines_trigger.negate()).whileActiveOnce(
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
					//actuate.getSupplier(this.input),				// press RB to feed
					()->false,
					Constants.feed_voltage,
					Constants.inches2volts_shooter
				),
				new RapidReactVision.HubAssistRoutine(
					this.drivebase,
					()->Xbox.Analog.RT.getValueOf(this.input) - Xbox.Analog.LT.getValueOf(this.input),
					3.0, 10.0	// max turning voltage and max voltage ramp
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
				Constants.auto_max_forward_voltage,
				Constants.auto_max_turn_voltage,
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

		Attack3.Digital			// the button binds
			shoot_toggle =			Attack3.Digital.TRI,	// left stick
			actuate =				Attack3.Digital.TRI,	// right stick
			hub_assist_toggle =		Attack3.Digital.TT,		// left stick
			cargo_assist_toggle =	Attack3.Digital.TT,		// right stick
			drivemode_increment =	Attack3.Digital.TR,		// right stick
			drivemode_decrement =	Attack3.Digital.TL,		// right stick
			override =				Attack3.Digital.TB,		// right stick
			camera_increment =		Attack3.Digital.TR,		// left stick
			camera_decrement =		Attack3.Digital.TL,		// left stick
			climb_toggle = 			Attack3.Digital.TB,		// left stick

			toggle_vision =			Attack3.Digital.B1,
			toggle_stats =			Attack3.Digital.B2,
			pipeline_increment =	Attack3.Digital.B4,
			pipeline_decrement =	Attack3.Digital.B3
		;

		TeleopTrigger.Get().whenActive(
			Constants.vision_driving
		).whenActive(
			this.drivebase.modeDrive(
				Attack3.Analog.X.getLimitedSupplier(this.stick_left, Constants.teleop_max_input_ramp),
				Attack3.Analog.Y.getLimitedSupplier(this.stick_left, Constants.teleop_max_input_ramp),
				Attack3.Analog.X.getLimitedSupplier(this.stick_right, Constants.teleop_max_input_ramp),
				Attack3.Analog.Y.getLimitedSupplier(this.stick_right, Constants.teleop_max_input_ramp),
				drivemode_increment.getPressedSupplier(this.stick_right),
				drivemode_decrement.getPressedSupplier(this.stick_right)
			)
		).whileActiveOnce(
			new ClimberSubsystem.HoldToggleControl(
				this.climb_sys, climb_toggle.getSupplier(this.stick_left),
				Constants.climber_extend_voltage,
				Constants.climber_retract_voltage,
				Constants.climber_hold_voltage
			)
		);	// schedule mode drive when in teleop mode

		toggle_vision.getCallbackFrom(this.stick_left).whenActive(VisionSubsystem.ToggleProcessing.Get());
		toggle_stats.getCallbackFrom(this.stick_left).whenActive(VisionSubsystem.ToggleStatistics.Get());
		pipeline_increment.getCallbackFrom(this.stick_left).whenActive(VisionSubsystem.IncrementPipeline.Get());
		pipeline_decrement.getCallbackFrom(this.stick_left).whenActive(VisionSubsystem.DecrementPipeline.Get());
		camera_increment.getCallbackFrom(this.stick_left).whenPressed(VisionSubsystem.IncrementCamera.Get());
		camera_decrement.getCallbackFrom(this.stick_left).whenPressed(VisionSubsystem.DecrementCamera.Get());

		StaticTrigger
			hub_assist_state = new StaticTrigger(false),
			cargo_assist_state = new StaticTrigger(false)
		;
		Trigger
			teleop_trigger = TeleopTrigger.Get(),

			shoot_trigger = shoot_toggle.getToggleFrom(this.stick_left),
			actuate_trigger = actuate.getCallbackFrom(this.stick_right),
			override_trigger = override.getCallbackFrom(this.stick_right),
			hub_assist_trigger = hub_assist_toggle.getToggleFrom(this.stick_left),
			cargo_assist_trigger = cargo_assist_toggle.getToggleFrom(this.stick_right),

			routines_trigger = hub_assist_state.or(cargo_assist_state)
		;

	// when the override button is not pressed...
		// and the shooter is toggled on
		teleop_trigger.and(shoot_trigger).and(override_trigger.negate()).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.managedShoot(
				//actuate.getSupplier(this.stick_right),
				()->false,
				Constants.feed_voltage,
				Constants.shooter_default_voltage
			)
		);
		// and the shooter is toggled off
		teleop_trigger.and(shoot_trigger.negate()).and(override_trigger.negate()).and(actuate_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.managedIntake(Constants.intake_voltage)
		);
	// when the override button is pressed...
		// and the shooter is toggled on
		teleop_trigger.and(shoot_trigger).and(override_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicShoot(
				//actuate.getSupplier(this.stick_right),
				()->false,
				Constants.feed_voltage,
				Constants.shooter_default_voltage
			)
		);
		// and the shooter is toggled off
		teleop_trigger.and(shoot_trigger.negate()).and(override_trigger).and(actuate_trigger).and(routines_trigger.negate()).whileActiveOnce(
			this.cargo_sys.basicIntake(Constants.intake_voltage)
		);

		// transfer override
		teleop_trigger.and(/*transfer_trigger.and(override_trigger).or*/(shoot_trigger.and(actuate_trigger))).and(routines_trigger.negate()).whileActiveOnce(
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
					//actuate.getSupplier(this.stick_right),		// press right trigger to feed
					()->false,
					Constants.feed_voltage,
					(double inches)-> Constants.shooter_static_voltage + (inches / Constants.max_hub_range_inches * (12 - Constants.shooter_static_voltage))
				),
				new RapidReactVision.HubAssistRoutine(
					this.drivebase,
					Attack3.Analog.X.getSupplier(this.stick_right),
					3.0, 10.0	// max turning voltage and max voltage ramp
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
				Constants.auto_max_forward_voltage,
				Constants.auto_max_turn_voltage,
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