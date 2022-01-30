package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestCommand extends CommandBase {
	private final String identifier;
	public TestCommand(String id) { this.identifier = id; }

	@Override public void initialize() { System.out.println("Command Init: " + this.identifier); }
	@Override public void execute() { System.out.println("\tCommand Exec: " + this.identifier); }
	@Override public void end(boolean i) { System.out.println("Command End(" + i + "): " + this.identifier); }
	@Override public boolean isFinished() { return true; }
	@Override public boolean runsWhenDisabled() { return true; }

}