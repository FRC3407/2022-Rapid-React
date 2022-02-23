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

	private final InputDevice input = new InputDevice(0);
	private final DriveBase drivebase = new DriveBase(Constants.drivebase_map_testbot);
	//private final Robot.Intake intake = new Robot.Intake(Constants.intake_port);
	private final CargoSystem cargo_sys = new CargoSystem(
		Constants.intake_port,
		Constants.feed_port,
		Constants.shooter_canid,
		Constants.lim_entering_dio,
		Constants.lim_exiting_dio,
		Constants.transfer_ports
	);

	public Runtime() {
		System.out.println("RUNTIME INITIALIZATION");
		VisionServer.Get();		// init VS before it needs to be used

		this.drivebase.setSpeedScaling(Constants.teleop_drivebase_scaling);
		this.drivebase.setSpeedDeadband(Constants.teleop_drivebase_deadband);
		this.drivebase.setSpeedSquaring(Constants.teleop_drivebase_speed_squaring);
	}

	private void buildRuntime() {	// schedules all button bindings and events for runtime

		Xbox.Digital.DT.getCallbackFrom(input).whenPressed(VisionSubsystem.IncrementPipeline.Get());
		Xbox.Digital.DB.getCallbackFrom(input).whenPressed(VisionSubsystem.DecrementPipeline.Get());
		Xbox.Digital.DR.getCallbackFrom(input).whenPressed(VisionSubsystem.IncrementCamera.Get());
		Xbox.Digital.DL.getCallbackFrom(input).whenPressed(VisionSubsystem.DecrementCamera.Get());
		Xbox.Digital.START.getCallbackFrom(input).whenPressed(VisionSubsystem.ToggleProcessing.Get());
		//Xbox.Digital.BACK.getCallbackFrom(input).whenPressed(VisionSubsystem.ToggleStatistics.Get());
		Xbox.Digital.BACK.getCallbackFrom(input).and(TeleopTrigger.Get()).whileActiveOnce(
			this.cargo_sys.manualOverride(
				Xbox.Digital.X.getSupplier(input),	// manually operate intake (override)
				Xbox.Digital.Y.getSupplier(input),	// manually operate transfer (override)
				Xbox.Digital.B.getSupplier(input)	// manually shoot (override)
			)
		);
		Xbox.Digital.X.getCallbackFrom(input).and(TeleopTrigger.Get()).and(Xbox.Digital.BACK.getCallbackFrom(input).negate()).whileActiveOnce(
			new LambdaCommand(()->System.out.println("Smart Intake"))	// intake command (smart)
		);
		Xbox.Digital.Y.getCallbackFrom(input).and(TeleopTrigger.Get()).and(Xbox.Digital.BACK.getCallbackFrom(input).negate()).whileActiveOnce(
			new LambdaCommand(()->System.out.println("Manual Transfer"))	// transfer command manual (~smart)
		);
		Xbox.Digital.B.getCallbackFrom(input).and(TeleopTrigger.Get()).and(Xbox.Digital.BACK.getCallbackFrom(input).negate()).whenActive(
			new LambdaCommand(()->System.out.println("Shoot (smart, vision?)"))	// shoot command (smart)
		);

		AutonomousTrigger.Get().whenActive(
			new Auto(this.drivebase)
		);
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
		//TestTrigger.Get().whenActive(null);

		System.out.println("Bindings Scheduled.");

	}

	@Override public void robotPeriodic() { CommandScheduler.getInstance().run(); }
	@Override public void robotInit() {
		if(!this.input.isConnected()) {
			this.input.connectionTrigger().whenActive(new LambdaCommand.Singular(()->{
				this.buildRuntime();
				CommandScheduler.getInstance();
			}, true));
		} else {
			this.buildRuntime();
		}
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