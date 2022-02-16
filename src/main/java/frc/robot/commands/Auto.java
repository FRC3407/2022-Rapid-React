package frc.robot.commands;

import frc.robot.modules.common.LambdaCommand;
import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.vision.java.VisionServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto extends SequentialCommandGroup {

	public Auto(DriveBase db) {
		super.addCommands(
			new LambdaCommand(()->VisionServer.Get().setStatistics(true)),
			new LambdaCommand(()->VisionServer.Get().setProcessingEnabled(true)),
		// move based on position on field and where a ball likely will be -> replace with trajectory movement when closed-loop is functional
			new BasicAutoDrive(db, 0.2, 0.2).withTimeout(2.0),
			new CargoTurn(db, DriverStation.getAlliance()),
			new CargoFollow(db, DriverStation.getAlliance()),
			// intake here
			// basic turn or integrate into HubTurn >>
			new HubTurn(db)
			// shoot
		);
	}


}