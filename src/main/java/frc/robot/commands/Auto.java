package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.modules.common.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto extends SequentialCommandGroup {

	public Auto(DriveBase db) {
		super.addCommands(
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