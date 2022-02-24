package frc.robot.commands;

import frc.robot.CargoSystem;
import frc.robot.Constants;
import frc.robot.RapidReactVision;
import frc.robot.modules.common.LambdaCommand;
import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.vision.java.VisionServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto extends SequentialCommandGroup {

	public Auto(DriveBase db, CargoSystem cs) {
		super.addCommands(
			new LambdaCommand(()->VisionServer.Get().setStatistics(true)),
			new LambdaCommand(()->VisionServer.Get().setProcessingEnabled(true)),
		// move based on position on field and where a ball likely will be -> replace with trajectory movement when closed-loop is functional
			new ConditionalCommand(	// whether or not a ball is detected by vision -> run closed-loop(vision) auto or not
				new ParallelCommandGroup(
					new CargoFollow(db, DriverStation.getAlliance()),
					cs.intakeCargo(Constants.intake_speed)
				),
				new ParallelRaceGroup(	// open loop
					new BasicAutoDrive(db, 0.25, 0.25).withTimeout(2),
					cs.intakeCargo(Constants.intake_speed)
				),
				()->RapidReactVision.isAllianceCargoDetected(DriverStation.getAlliance())
			),
			//new BasicAutoDrive(db, 0.25, 0.25).withTimeout(2),
			//new CargoFind(db, DriverStation.getAlliance()),		// turn and try to find a cargo (will just end if there is already one in view)
			//new CargoFollow(db, DriverStation.getAlliance()),	// drive towards cargo
			// intake here
			new HubFind(db),	// turn until the hub is in view/detected
			new HubTurn(db),	// turn so the hub is straight ahead
			cs.shootAllCargo(Constants.shooter_default_speed)	// replace with vision shoot 
		);
	}


}