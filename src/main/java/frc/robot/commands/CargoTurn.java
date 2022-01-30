package frc.robot.commands;

import frc.robot.modules.common.DriveBase;
import frc.robot.modules.vision.java.VisionServer;
import frc.robot.RapidReactVision;

import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class CargoTurn extends DriveBase.DriveCommandBase {

	private final Alliance team;
	private VisionServer.TargetData position;

	public CargoTurn(DriveBase db, Alliance a) {
		super(db);
		this.team = a;
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		this.position = RapidReactVision.getClosestAllianceCargo(this.team);
		if(this.position != null) {
			System.out.println(String.valueOf(this.position.lr));
			super.tankDrive((this.position.lr/25)*0.2, -(this.position.lr/25)*0.2);
		} else {
			super.tankDrive(0, 0);
		}
	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return false;
	}

}