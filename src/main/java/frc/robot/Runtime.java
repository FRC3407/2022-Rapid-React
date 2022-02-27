package frc.robot;

import frc.robot.commands.*;
import frc.robot.modules.common.*;
import frc.robot.modules.common.Input.*;
import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.common.drive.*;
import frc.robot.modules.common.EventTriggers.*;
import frc.robot.modules.vision.java.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.NetworkTableInstance;

// import frc.robot.modules.common.drive.Types.*;
// import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;


/* TODO:
 x Split VisionServer into base and an extension that integrates command-based structure (this would extend SubsystemBase)
 x Verify/imporve singleton for each ^^^
 x Create "helper" container for methods at the start of DriveBase
 x Make better names for "DB#..." and split into port map and motorcontroller object containers (this would be an extension)
 - Commands for CLDifferential -> Ramsete controller (characterization first)
 - Update C++ VisionServer-Robot API (actually already safe because of the use of LANGUAGE SUPPORT FOR UNSIGNED ~> smh java :| )
 - Methods/impelemtation to search DriverStation for a certain input (common.Input.InputDevice) and return object/port
 x max output/scaling method for drivebase
 - make a spreadsheet for camera presets (each pipeline) under different lighting conditions
 x? fix controls being f'ed when not in sim mode and not connected on program startup

 - Tune hubturn p-loop
 - Change camera params / configure switching when camera positions are finalized
*/

public class Runtime extends TimedRobot {

	private final InputDevice
		input = new InputDevice(0),		// controller
		stick_left = new InputDevice(1),	// acrade stick (left)
		stick_right = new InputDevice(2);	// arcade stick (right)
	private final DriveBase drivebase = new DriveBase(Constants.drivebase_map_2022);
	private final CargoSystemV2 cargo_sys = new CargoSystemV2(
		new CargoSystemV2.IntakeSubsystem(Constants.intake_port),
		new CargoSystemV2.TransferSubsystem(Constants.transfer_ports),
		new CargoSystemV2.ShooterSubsystem(Constants.w0_shooter_port)
	);
	// private final CargoSystem.WeekZero w0_cargo_sys = new CargoSystem.WeekZero(
	// 	Constants.intake_port,
	// 	Constants.feed_port,
	// 	Constants.w0_shooter_port,
	// 	Constants.transfer_ports
	// );

	public Runtime() {
		System.out.println("RUNTIME INITIALIZATION");
		VisionServer.Get();		// init VS before it needs to be used

		this.drivebase.setSpeedScaling(Constants.teleop_drivebase_scaling);
		this.drivebase.setSpeedDeadband(Constants.teleop_drivebase_deadband);
		this.drivebase.setSpeedSquaring(Constants.teleop_drivebase_speed_squaring);
	}

	private void xboxControls() {	// bindings for xbox controller

		Xbox.Digital.DT.getCallbackFrom(input).whenPressed(VisionSubsystem.IncrementPipeline.Get());	// dpad top -> increment pipeline
		Xbox.Digital.DB.getCallbackFrom(input).whenPressed(VisionSubsystem.DecrementPipeline.Get());	// dpad bottom -> decrement pipeline
		Xbox.Digital.DR.getCallbackFrom(input).whenPressed(VisionSubsystem.IncrementCamera.Get());		// dpad right -> increment camera
		Xbox.Digital.DL.getCallbackFrom(input).whenPressed(VisionSubsystem.DecrementCamera.Get());		// dpad left -> decrement camera
		Xbox.Digital.START.getCallbackFrom(input).whenPressed(VisionSubsystem.ToggleProcessing.Get());	// toggle visionserver processing
		Xbox.Digital.BACK.getCallbackFrom(input).whenPressed(VisionSubsystem.ToggleStatistics.Get());	// toggle statistics in camera view
		// Xbox.Digital.BACK.getCallbackFrom(input).and(TeleopTrigger.Get()).whileActiveOnce(
		// 	this.cargo_sys.manualOverride(
		// 		Xbox.Digital.X.getSupplier(input),	// manually operate intake (override)
		// 		Xbox.Digital.Y.getSupplier(input),	// manually operate transfer (override)
		// 		Xbox.Digital.A.getSupplier(input),	// manually feed (override)
		// 		Xbox.Digital.B.getSupplier(input)	// manually spin shooter (override)
		// 	)
		// );
		Xbox.Digital.X.getCallbackFrom(input).and(TeleopTrigger.Get())/*.and(Xbox.Digital.BACK.getCallbackFrom(input).negate())*/.whileActiveOnce(
			this.cargo_sys.basicIntake(Constants.intake_speed)
			//this.w0_cargo_sys.intakeControl()						// run the intake when 'X' is pressed
		);
		// Xbox.Digital.Y.getCallbackFrom(input).and(TeleopTrigger.Get())/*.and(Xbox.Digital.BACK.getCallbackFrom(input).negate())*/.whileActiveOnce(
		// 	//new LambdaCommand(()->System.out.println("Manual Transfer"))	// transfer command manual (~smart)
		// 	this.w0_cargo_sys.transferControl()						// move the transfer belt when 'Y' is pressed
		// );
		// Xbox.Digital.B.getCallbackFrom(input).and(TeleopTrigger.Get())/*.and(Xbox.Digital.BACK.getCallbackFrom(input).negate())*/.toggleWhenActive(	// toggleWhenActive
		// 	//this.cargo_sys.shootAllCargo(Constants.shooter_default_speed)
		// 	this.w0_cargo_sys.shooterControl(0.75, Xbox.Digital.A.getSupplier(input), ()->false)	// spin up the shooter when B is pressed, run feed when A is pressed, stop when B is pressed again
		// );

		TeleopTrigger.Get().whenActive(
			new SequentialCommandGroup(
				new LambdaCommand(()->VisionServer.Get().setStatistics(false)),
				new LambdaCommand(()->VisionServer.Get().applyCameraPreset(Constants.cam_driving)),
				new LambdaCommand(()->VisionServer.Get().setProcessingEnabled(false)),
				// this.drivebase.tankDrive(
				// 	Xbox.Analog.LY.getSupplier(input), 
				// 	Xbox.Analog.RY.getSupplier(input)
				// )
				this.drivebase.modeDrive(
					Xbox.Analog.LX.getSupplier(input),
					Xbox.Analog.LY.getSupplier(input),
					Xbox.Analog.LT.getSupplier(input),
					Xbox.Analog.RX.getSupplier(input),
					Xbox.Analog.RY.getSupplier(input),
					Xbox.Analog.RT.getSupplier(input),
					Xbox.Digital.RS.getPressedSupplier(input),
					Xbox.Digital.LS.getPressedSupplier(input)
				)
			), false	// not interruptable because the drivebase should always be drivable in teleop mode
		);

		System.out.println("Xbox Bindings Scheduled.");

	}
	private void arcadeControls() {	// bindings for arcade board

		// right stick top buttons control vision stuff -> basically the same as the dpad bindings
		Attack3.Digital.TT.getCallbackFrom(this.stick_left).whenPressed(VisionSubsystem.IncrementPipeline.Get());
		Attack3.Digital.TB.getCallbackFrom(this.stick_left).whenPressed(VisionSubsystem.DecrementPipeline.Get());
		Attack3.Digital.TR.getCallbackFrom(this.stick_left).whenPressed(VisionSubsystem.IncrementCamera.Get());
		Attack3.Digital.TL.getCallbackFrom(this.stick_left).whenPressed(VisionSubsystem.DecrementCamera.Get());
		Attack3.Digital.TB.getCallbackFrom(this.stick_right).whenPressed(VisionSubsystem.ToggleProcessing.Get());
		//Attack3.Digital.TB.getCallbackFrom(this.stick_right).whenPressed(VisionSubsystem.ToggleStatistics.Get());
		
		// Attack3.Digital.TRI.getCallbackFrom(this.stick_left).and(TeleopTrigger.Get()).whileActiveOnce(
		// 	this.w0_cargo_sys.intakeControl()
		// );
		// Attack3.Digital.TT.getCallbackFrom(this.stick_right).and(TeleopTrigger.Get()).whileActiveOnce(
		// 	this.w0_cargo_sys.transferControl()
		// );
		// Attack3.Digital.TRI.getCallbackFrom(this.stick_right).and(TeleopTrigger.Get()).toggleWhenActive(
		// 	this.w0_cargo_sys.shooterControl(0.85, /*Attack3.Digital.TB.getSupplier(this.stick_right)*/()->false, ()->false)
		// );

		
		TeleopTrigger.Get().whenActive(
			new SequentialCommandGroup(
				new LambdaCommand(()->VisionServer.Get().setStatistics(false)),
				new LambdaCommand(()->VisionServer.Get().applyCameraPreset(Constants.cam_driving)),
				new LambdaCommand(()->VisionServer.Get().setProcessingEnabled(false)),
				this.drivebase.modeDrive(
					Attack3.Analog.X.getSupplier(this.stick_left),
					Attack3.Analog.Y.getSupplier(this.stick_left),
					Attack3.Analog.X.getSupplier(this.stick_right),
					Attack3.Analog.Y.getSupplier(this.stick_right),
					Attack3.Digital.TR.getPressedSupplier(this.stick_right),
					Attack3.Digital.TL.getPressedSupplier(this.stick_right)
				)
			), false	// not interruptable because the drivebase should always be drivable in teleop mode
		);
		
		System.out.println("Arcade Bindings Scheduled.");
	}

	@Override public void robotPeriodic() { CommandScheduler.getInstance().run(); }
	@Override public void robotInit() {
		//AutonomousTrigger.Get().whenActive( new Auto.WeekZero(this.drivebase, this.w0_cargo_sys) );
		//TestTrigger.Get().whenActive( new CargoFollow.Demo(this.drivebase, DriverStation.getAlliance(), Constants.cargo_cam_name) );

		new Trigger(()->VisionServer.Get().isConnected()).whenActive(new LambdaCommand(()->System.out.println("VisionServer Connected")));

		if(!this.input.isConnected()) {
			this.input.connectionTrigger().whenActive(new LambdaCommand.Singular(()->{
				this.xboxControls();
			}, true));
		} else {
			this.xboxControls();
		}
		// if(!(this.stick_left.isConnected() && this.stick_right.isConnected())) {
		// 	this.stick_left.connectionTrigger().and(this.stick_right.connectionTrigger()).whenActive(new LambdaCommand.Singular(()->{
		// 		this.arcadeControls();
		// 	}));
		// } else {
		// 	this.arcadeControls();
		// }
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


}