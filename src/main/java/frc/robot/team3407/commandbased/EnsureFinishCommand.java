package frc.robot.team3407.commandbased;

import edu.wpi.first.wpilibj2.command.*;


public class EnsureFinishCommand extends CommandBase {

	private final Command c;
	private final int threshold;
	private int count = 0;

	public EnsureFinishCommand(Command c, int f_thresh) {
		this.c = c;
		this.threshold = f_thresh;
	}

	@Override public void initialize() {
		this.count = 0;
		this.c.initialize();
	}
	@Override public void execute() {
		this.c.execute();
	}
	@Override public void end(boolean i) {
		this.c.end(i);
	}
	@Override public boolean isFinished() {
		if(this.c.isFinished()) {
			this.count++;
		}
		return this.count >= this.threshold;
	}
	@Override public boolean runsWhenDisabled() {
		return this.c.runsWhenDisabled();
	}


}