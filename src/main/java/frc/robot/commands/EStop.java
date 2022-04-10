package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class EStop extends CommandBase {

	private EStop() {}
	private static EStop instance = new EStop();
	public static EStop get() { return EStop.instance; }

	@Override
	public void initialize() {
		CommandScheduler.getInstance().cancelAll();
		System.out.println("All commands cancelled");
	}
	@Override public boolean isFinished() { return true; }
	@Override public boolean runsWhenDisabled() { return true; }


}