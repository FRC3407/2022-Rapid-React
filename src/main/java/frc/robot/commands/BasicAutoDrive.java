package frc.robot.commands;

import frc.robot.modules.common.drive.DriveBase;


public class BasicAutoDrive extends DriveBase.DriveCommandBase {
	
	private final double left, right;

	public BasicAutoDrive(DriveBase db, double l, double r) {
		super(db);
		this.left = l;
		this.right = r;
	}

	@Override public void initialize() {}
	@Override public void execute() {
		super.autoDrive(this.left, this.right);
	}
	@Override public void end(boolean interrupted) {}
	@Override public boolean isFinished() {
		return false;
	}
}
