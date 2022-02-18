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
 - Verify/imporve singleton for each ^^^
 x Create "helper" container for methods at the start of DriveBase
 x Make better names for "DB#..." and split into port map and motorcontroller object containers (this would be an extension)
 - Commands for CLDifferential -> Ramsete controller (characterization first)
 - Update C++ VisionServer-Robot API (actually already safe because of the use of LANGUAGE SUPPORT FOR UNSIGNED ~> smh java :| )
 - Methods/impelemtation to search DriverStation for a certain input (common.Input.InputDevice) and return object/port
 - max output/scaling method for drivebase
*/

public class Runtime extends TimedRobot {

	private final InputDevice input = new InputDevice(0);
	private final DriveBase drivebase = new DriveBase(Constants.drivebase_map);

	public Runtime() {
		System.out.println("RUNTIME INITIALIZATION");

		this.drivebase.setSpeedScaling(Constants.teleop_drivebase_scaling);
		this.drivebase.setSpeedDeadband(Constants.teleop_drivebase_deadband);
		this.drivebase.setSpeedSquaring(Constants.teleop_drivebase_speed_squaring);
	}


	@Override public void robotPeriodic() { CommandScheduler.getInstance().run(); }
	@Override public void robotInit() {

		Xbox.Digital.DT.getCallbackFrom(input).whenPressed(VisionSubsystem.getPipelineIncrementCommand());
		Xbox.Digital.DB.getCallbackFrom(input).whenPressed(VisionSubsystem.getPipelineDecrementCommand());
		Xbox.Digital.DR.getCallbackFrom(input).whenPressed(VisionSubsystem.getCameraIncrementCommand());
		Xbox.Digital.DL.getCallbackFrom(input).whenPressed(VisionSubsystem.getCameraDecrementCommand());
		Xbox.Digital.START.getCallbackFrom(input).whenPressed(VisionSubsystem.getProcessingToggleCommand());
		Xbox.Digital.BACK.getCallbackFrom(input).whenPressed(VisionSubsystem.getToggleStatisticsCommand());
		Xbox.Digital.A.getCallbackFrom(input).whenPressed(
			()->System.out.println("Current Pipeline: " + VisionServer.Get().getCurrentPipeline().getName())
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
				this.drivebase.modeDrive(	// <<- fix buttons triggering too many times
					Xbox.Analog.LX.getSupplier(input),
					Xbox.Analog.LY.getSupplier(input),
					Xbox.Analog.LT.getSupplier(input),
					Xbox.Analog.RX.getSupplier(input),
					Xbox.Analog.RY.getSupplier(input),
					Xbox.Analog.RT.getSupplier(input),
					Xbox.Digital.RB.getPressedSupplier(input),
					Xbox.Digital.LB.getPressedSupplier(input)
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