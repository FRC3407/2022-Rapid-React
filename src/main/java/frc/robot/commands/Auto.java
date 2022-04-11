package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.*;
import frc.robot.team3407.commandbased.*;
import frc.robot.team3407.drive.DriveBase;


/** This class contains various composed commands that are either tests or for actual competition autonomous */
public class Auto {

	public static class OpenLoop extends SequentialCommandGroup {

		public OpenLoop(DriveBase db, CargoSystem cs) {
			super.addCommands(
				cs.deployIntake(),
				new ParallelCommandGroup(
					new DriveBase.BasicDriveControl.Voltage(db, 3.0, 3.0),	// drive forward "taxi"
					cs.basicIntake(Constants.intake_voltage)		// run intake
				).withTimeout(3.5),
				new DriveBase.BasicDriveControl.Voltage(db, -4.5, 4.5).withTimeout(2.25),	// turn
				//cs.shootAll(Constants.feed_voltage, 11.0, Constants.transfer_voltage)	// shoot all balls
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.5),
						cs.basicTransfer(Constants.transfer_voltage).withTimeout(2.5)
					),
					cs.basicShoot(
						()->false,
						0, Constants.shooter_max_voltage
					)
				)
			);
		}


	}
	public static class ClosedLoop extends SequentialCommandGroup {

		public ClosedLoop(ClosedLoopDifferentialDrive db, CargoSystem cs) {
			super.addCommands(
				new ConditionalCommand(
					new SequentialCommandGroup(
						cs.deployIntake(),
						new RapidReactVision.CargoAssistRoutine.EndOnIntake(		// find and intake the nearest cargo
							db,
							new RapidReactVision.CargoFind(db),
							cs.basicIntake(Constants.intake_voltage),
							cs.transfer
						),
						new RapidReactVision.HubFind(db),	// find hub
						new EnsureFinishCommand(new RapidReactVision.HubTarget(
							db,
							((Constants.min_hub_range_inches + Constants.max_hub_range_inches) / 2.0),
							40,
							Constants.auto_max_forward_voltage,
							Constants.auto_max_turn_voltage,
							Constants.auto_max_voltage_ramp
						), 10),	// hub aim
						new ParallelDeadlineGroup(
							new SequentialCommandGroup(
								new WaitCommand(0.5),	// wait for shooter to spin up
								cs.basicTransfer(Constants.transfer_voltage).withTimeout(2.5)	// actuate transfer
							),
							cs.visionShoot(	// spin shooter based on distance
								()->false, 0,
								Constants.inches2volts_shooter
							)
						)
					),
					new OpenLoop(db, cs),
					()->{
						Constants.vision_cargo.run();
						return RapidReactVision.isAllianceCargoDetected(DriverStation.getAlliance());
					}
				)
			);
		}


	}


}