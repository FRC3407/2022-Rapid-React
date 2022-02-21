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

 - Tune hubturn p-loop
 - Change camera params / configure switching when camera positions are finalized
*/

public class Runtime extends TimedRobot {

	private final InputDevice input = new InputDevice(0);
	private final DriveBase drivebase = new DriveBase(Constants.drivebase_map_testbot);
	private final Robot.Intake intake = new Robot.Intake(Constants.intake_port);

	public Runtime() {
		System.out.println("RUNTIME INITIALIZATION");
		VisionServer.Get();		// init VS before it needs to be used

		this.drivebase.setSpeedScaling(Constants.teleop_drivebase_scaling);
		this.drivebase.setSpeedDeadband(Constants.teleop_drivebase_deadband);
		this.drivebase.setSpeedSquaring(Constants.teleop_drivebase_speed_squaring);
	}


	@Override public void robotPeriodic() { CommandScheduler.getInstance().run(); }
	@Override public void robotInit() {

		Xbox.Digital.DT.getCallbackFrom(input).whenPressed(VisionSubsystem.IncrementPipeline.Get());
		Xbox.Digital.DB.getCallbackFrom(input).whenPressed(VisionSubsystem.DecrementPipeline.Get());
		Xbox.Digital.DR.getCallbackFrom(input).whenPressed(VisionSubsystem.IncrementCamera.Get());
		Xbox.Digital.DL.getCallbackFrom(input).whenPressed(VisionSubsystem.DecrementCamera.Get());
		Xbox.Digital.START.getCallbackFrom(input).whenPressed(VisionSubsystem.ToggleProcessing.Get());
		Xbox.Digital.BACK.getCallbackFrom(input).whenPressed(VisionSubsystem.ToggleStatistics.Get());
		Xbox.Digital.A.getCallbackFrom(input).whenPressed(
			()->System.out.println("Current Pipeline: " + VisionServer.Get().getCurrentPipeline().getName())
		);
		Xbox.Digital.B.getCallbackFrom(input).and(TeleopTrigger.Get()).whileActiveContinuous(	// when in teleop and 'B' button pressed
			()->{ this.intake.set(0.5); }
		).whenInactive(
			()->{ this.intake.set(0.0); }
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
			)
		);
		//TestTrigger.Get().whenActive(null);

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