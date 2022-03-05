package frc.robot.commands;

import frc.robot.modules.common.drive.DriveBase;


public class BasicDriveControl extends DriveBase.RateLimitedAutoDrive {
	
	private final double left, right;

	public BasicDriveControl(DriveBase db, double l, double r) {
		super(db);
		this.left = l;
		this.right = r;
	}
	public BasicDriveControl(DriveBase db, double l, double r, double mr) {
		super(db, mr);
		this.left = l;
		this.right = r;
	}

	@Override public void execute() {
		super.autoDrive(this.left, this.right);
	}
	@Override public void end(boolean interrupted) {}
	@Override public boolean isFinished() { return false; }


}