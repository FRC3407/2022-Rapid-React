package frc.robot.commands;

import frc.robot.*;
import frc.robot.modules.common.*;
import frc.robot.modules.common.drive.DriveBase;
import frc.robot.modules.vision.java.VisionServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.*;


/** This class contains various composed commands that are either tests or for actual competition autonomous */
public class Auto {

	public static class OpenLoop extends SequentialCommandGroup {

		public OpenLoop(DriveBase db, CargoSystem cs) {
			super.addCommands(
				new ParallelCommandGroup(
					new BasicDriveControl.Voltage(db, 3.0, 3.0),	// drive forward "taxi"
					cs.managedIntake(Constants.intake_voltage)		// run intake
				).withTimeout(3.5),
				new BasicDriveControl.Voltage(db, -3.0, 3.0).withTimeout(1.0),	// turn
				cs.shootAll(Constants.feed_voltage, 10.0, Constants.transfer_voltage)	// shoot all balls
			);
		}

	}
	public static class ClosedLoop extends SequentialCommandGroup {

		// /** Basic Driving, Vision CL when possible */
		// public ClosedLoop(DriveBase db, CargoSystem cs) {
			
		// }
		// /** Open-loop driving with closed loop turning and Vision when possible */
		// public ClosedLoop(DriveBase db, CargoSystem cs, Gyro gy) {

		// }
		/** Closed-loop driving (trajectories) with Vision CL when available */
		public ClosedLoop(ClosedLoopDifferentialDrive db, CargoSystem cs) {
			super.addCommands(
				//new BasicDriveControl(db, 0.6, 0.6).withTimeout(0.2),		// get the intake to pop down
				//new BasicDriveControl(db, -0.6, -0.6).withTimeout(0.1),
				cs.deployIntake(),
				new RapidReactVision.CargoAssistRoutine.EndOnIntake(		// find and intake the nearest cargo
					db,
					new RapidReactVision.CargoFind(db),
					cs.managedIntake(Constants.intake_voltage),
					cs.transfer
				),
				new RapidReactVision.HubFind(db),	// find hub
				new RapidReactVision.HubTurn(db),	// hub aim
				cs.visionShootAll(	// shoot all cargo
					Constants.feed_voltage, Constants.transfer_voltage,
					Constants.inches2volts_shooter
				)
			);
		}

	}

	// public static class WeekZero extends SequentialCommandGroup {

	// 	public WeekZero(DriveBase db, CargoSystem.WeekZero cs) {
	// 		super.addCommands(
	// 			new LambdaCommand(()->System.out.println("Autonomous Running...")),
	// 			new LambdaCommand(()->VisionServer.setStatistics(true)),
	// 			new LambdaCommand(()->VisionServer.setProcessingEnabled(true)),

	// 			// new ConditionalCommand(
	// 			// 	new ParallelCommandGroup(	// cargo detected -> drive towards it and intake
	// 			// 		new CargoFollow(db, DriverStation.getAlliance(), Constants.cargo_cam_name).withTimeout(5),
	// 			// 		cs.intakeControl().withTimeout(6),
	// 			// 		new SequentialCommandGroup(
	// 			// 			new WaitCommand(4),
	// 			// 			cs.transferControl().withTimeout(2)
	// 			// 		)
	// 			// 	),
	// 			// 	new ParallelCommandGroup(	// cargo not detected -> drive forward for "taxi" points, spin intake just incase
	// 			// 		new BasicAutoDrive(db, 0.25, 0.25).withTimeout(2),
	// 			// 		cs.intakeControl().withTimeout(3),
	// 			// 		new SequentialCommandGroup(
	// 			// 			new WaitCommand(1),
	// 			// 			cs.transferControl().withTimeout(2)
	// 			// 		)
	// 			// 	),
	// 			// 	()->VisionServer.isConnected() && RapidReactVision.isAllianceCargoDetected(DriverStation.getAlliance())
	// 			// ),
	// 			new ParallelRaceGroup(
	// 				new ConditionalCommand(
	// 					new CargoFollow(db, DriverStation.getAlliance(), Constants.cargo_cam_name).withTimeout(5),
	// 					new BasicDriveControl(db, 0.25, 0.25).withTimeout(2),
	// 					()->VisionServer.isConnected() && RapidReactVision.isAllianceCargoDetected(DriverStation.getAlliance())
	// 				),
	// 				cs.intakeControl()
	// 			),

	// 			new HubFind(db, Constants.hub_cam_name).withTimeout(5),		// turn to look for the hub, 5 seconds should put the robot @ 180 degrees, so good for a shot even if vision fails
	// 			new HubTurn(db, Constants.hub_cam_name).withTimeout(1),		// fine-tune angle to hub
	// 			// new ParallelCommandGroup(
	// 			// 	cs.shooterControl().withTimeout(8),	// ramp up the shooter
	// 			// 	new SequentialCommandGroup(			// wait for shooter to ramp up then send cargo to be shot
	// 			// 		new WaitCommand(1),
	// 			// 		cs.transferControl().withTimeout(5)
	// 			// 	)
	// 			// ),
	// 			new LambdaCommand(()->System.out.println("Autonomous Completed."))
	// 		);
	// 	}


	// }

	// public Auto(DriveBase db, CargoSystem cs) {
	// 	super.addCommands(
	// 		new LambdaCommand(()->VisionServer.setStatistics(true)),
	// 		new LambdaCommand(()->VisionServer.setProcessingEnabled(true)),
	// 		new LambdaCommand(()->RapidReactVision.setCargoPipelineActive()),
	// 		//new LambdaCommand(()->RapidReactVision.setCargoAllianceColorMode(DriverStation.getAlliance())),
	// 	// move based on position on field and where a ball likely will be -> replace with trajectory movement when closed-loop is functional
	// 		// new ConditionalCommand(	// whether or not a ball is detected by vision -> run closed-loop(vision) auto or not
	// 		// 	new CargoFollow(db, DriverStation.getAlliance()),
	// 		// 	new BasicAutoDrive(db, 0.25, 0.25).withTimeout(2),	
	// 		// // new SequentialCommandGroup(
	// 		// 	// 	new CargoFollow(db, DriverStation.getAlliance()),
	// 		// 	// 	cs.intakeCargo(Constants.intake_speed)
	// 		// 	// ),
	// 		// 	// new ParallelRaceGroup(	// open loop
	// 		// 	// 	new BasicAutoDrive(db, 0.25, 0.25).withTimeout(2),
	// 		// 	// 	cs.intakeCargo(Constants.intake_speed)
	// 		// 	// ),
	// 		// 	()->{ 
	// 		// 		System.out.println(RapidReactVision.isAllianceCargoDetected(DriverStation.getAlliance()));
	// 		// 		return RapidReactVision.isAllianceCargoDetected(DriverStation.getAlliance()); 
	// 		// 	}
	// 		// ),
	// 		new CargoFollow(db, DriverStation.getAlliance()).withTimeout(5),
	// 		//new BasicAutoDrive(db, 0.25, 0.25).withTimeout(2),
	// 		//new CargoFind(db, DriverStation.getAlliance()),		// turn and try to find a cargo (will just end if there is already one in view)
	// 		//new CargoFollow(db, DriverStation.getAlliance()),	// drive towards cargo
	// 		// intake here
	// 		new HubFind(db),	// turn until the hub is in view/detected
	// 		new HubTurn(db),	// turn so the hub is straight ahead
	// 		cs.shootAllCargo(Constants.shooter_default_speed)	// replace with vision shoot 
	// 	);
	// }


}