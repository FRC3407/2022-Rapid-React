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
				new TaxiAndIntake(db, cs),
				new TurnAndShoot(db, cs)
			);
		}

		public static class TaxiAndIntake extends SequentialCommandGroup {

			public TaxiAndIntake(DriveBase db, CargoSystem cs) {
				super.addCommands(
					cs.deployIntake(),
					new ParallelCommandGroup(
						new DriveBase.BasicDriveControl.Voltage(db, 3.0, 3.0),	// drive forward "taxi"
						cs.basicIntake(Constants.intake_voltage)		// run intake
					).withTimeout(3.5)
				);
			}


		}
		public static class TurnAndShoot extends SequentialCommandGroup {

			public TurnAndShoot(DriveBase db, CargoSystem cs) {
				super.addCommands(
					new DriveBase.BasicDriveControl.Voltage(db, -4.5, 4.5).withTimeout(2.5),	// turn
					new ParallelDeadlineGroup(
						new SequentialCommandGroup(
							new WaitCommand(1.0),
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


	}
	public static class ClosedLoop extends SequentialCommandGroup {

		private boolean cargo_detected = false;

		public ClosedLoop(ClosedLoopDifferentialDrive db, CargoSystem cs) {
			Constants.vision_cargo.run();
			super.addCommands(
				new RunForCommand(
					()->{ this.cargo_detected |= RapidReactVision.isAllianceCargoDetected(DriverStation.getAlliance()); },
					20	// check for cargo 20 times
				),
				new ConditionalCommand(
					// if cargo detected
					new SequentialCommandGroup(
						new TaxiAndIntakeVision(db, cs),
						new HubShootVision(db, cs)
					),
					// if cargo not detected
					new SequentialCommandGroup(
						new OpenLoop.TaxiAndIntake(db, cs),
						new ConditionalCommand(
							new HubShootVision(db, cs),
							new OpenLoop.TurnAndShoot(db, cs),
							// depends of if vision is available (connected)
							()->RapidReactVision.hasHubPipeline()
						)
					),
					// cargo detected condition
					()->this.cargo_detected
				)
			);
		}

		@Override public void initialize() {
			super.initialize();
			this.cargo_detected = false;
		}

		public static class TaxiAndIntakeVision extends SequentialCommandGroup {

			public TaxiAndIntakeVision(DriveBase db, CargoSystem cs) {
				super.addCommands(
					cs.deployIntake(),
					new RapidReactVision.CargoAssistRoutine.EndOnIntake(		// find and intake the nearest cargo
						db,
						new RapidReactVision.CargoFind(db),
						cs.basicIntake(Constants.intake_voltage),
						cs.transfer
					)
				);
			}


		}
		public static class HubShootVision extends SequentialCommandGroup {

			public HubShootVision(DriveBase db, CargoSystem cs) {
				super.addCommands(
					new RapidReactVision.HubFind(db),	// find hub
					new EnsureFinishCommand(new RapidReactVision.HubTarget(
						db,
						Constants.hub_targeting_inches,
						40,
						Constants.auto_max_forward_voltage,
						Constants.auto_max_turn_voltage,
						Constants.auto_max_voltage_ramp
					), 10),		// aim towards hub, drive to optimal distance
					new ParallelDeadlineGroup(
						new SequentialCommandGroup(
							new WaitCommand(1.0),	// wait for shooter to spin up
							cs.basicTransfer(Constants.transfer_voltage).withTimeout(2.5)	// actuate transfer
						),
						cs.visionShoot(	// spin shooter based on distance
							()->false, 0,
							Constants.inches2volts_shooter
						)
					)
				);
			}


		}


	}


}