package frc.robot;

import frc.robot.commands.*;
import frc.robot.modules.common.*;
import frc.robot.modules.common.Input.*;
import frc.robot.modules.common.EventTriggers.*;
import frc.robot.modules.vision.java.VisionServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTableInstance;


/* TODO:
 - Split VisionServer into base and an extension that integrates command-based structure (this would extend SubsystemBase)
 - Create "helper" container for methods at the start of DriveBase
 - Make better names for "DB#..." and split into port map and motorcontroller object containers (this would be an extension)
 - Commands for CLDifferential -> Ramsete controller (characterization first)
 - Update C++ VisionServer-Robot API
*/

public class Runtime extends TimedRobot {

	private final InputDevice input = new InputDevice(0);
	private final DriveBase drivebase = new DriveBase(Constants.drivebase_map_2019, Constants.drivebase_settings);

	public Runtime() {
		System.out.println("RUNTIME INITIALIZATION");
	}


	@Override public void robotPeriodic() { CommandScheduler.getInstance().run(); }
	@Override public void robotInit() {

		Xbox.Digital.LB.getCallbackFrom(input).whenPressed(VisionServer.getPipelineIncrementCommand());
		Xbox.Digital.RB.getCallbackFrom(input).whenPressed(VisionServer.getPipelineDecrementCommand());
		Xbox.Digital.DT.getCallbackFrom(input).whenPressed(VisionServer.getCameraIncrementCommand());
		Xbox.Digital.DB.getCallbackFrom(input).whenPressed(VisionServer.getCameraDecrementCommand());
		Xbox.Digital.START.getCallbackFrom(input).whenPressed(VisionServer.getPipelineToggleCommand());
		Xbox.Digital.BACK.getCallbackFrom(input).whenPressed(VisionServer.getToggleStatisticsCommand());

		TeleopTrigger.Get().whenActive(
			new SetCameras(
				Constants.cam_driving
			).andThen(
				()->{ VisionServer.Get().setProcessingEnabled(false); }
			).andThen(
				this.drivebase.tankDrive(Xbox.Analog.LY.getSupplier(input), Xbox.Analog.RY.getSupplier(input))
			)
		);
		AutonomousTrigger.Get().whenActive(new Auto(this.drivebase));	// also re-enable processing (make command to do so)
		Xbox.Digital.A.getCallbackFrom(input).whenPressed(
			()->{
				System.out.println("Current Pipeline: " + VisionServer.Get().getCurrentPipeline().getName());
			}
		);
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