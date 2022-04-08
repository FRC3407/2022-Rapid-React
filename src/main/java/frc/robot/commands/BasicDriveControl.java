package frc.robot.commands;

import frc.robot.modules.common.drive.DriveBase;


public class BasicDriveControl extends DriveBase.DriveCommandBase {
	
	private final double left, right;

	public BasicDriveControl(DriveBase db, double l, double r) {
		super(db);
		this.left = l;
		this.right = r;
	}
	// public BasicDriveControl(DriveBase db, double l, double r, double mr) {
	// 	super(db, mr);
	// 	this.left = l;
	// 	this.right = r;
	// }

	@Override public void initialize() {
		System.out.println("BasicDrive: Running...");
	}
	@Override public void execute() {
		super.autoDrive(this.left, this.right);
	}
	@Override public void end(boolean i) {
		System.out.println("BasicDrive: " + (i ? "Terminated." : "Completed."));
	}
	@Override public boolean isFinished() { return false; }


	public static class Voltage extends BasicDriveControl {

		public Voltage(DriveBase db, double lv, double rv) { super(db, lv, rv); }
		//public Voltage(DriveBase db, double lv, double rv, double mr) { super(db, lv, rv, mr); }

		@Override public void initialize() {
			System.out.println("BasicVoltageDrive: Running...");
		}
		@Override public void execute() {
			super.autoDriveVoltage(super.left, super.right);
		}
		@Override public void end(boolean i) {
			System.out.println("BasicVoltageDrive: " + (i ? "Terminated." : "Completed."));
		}


	}


}