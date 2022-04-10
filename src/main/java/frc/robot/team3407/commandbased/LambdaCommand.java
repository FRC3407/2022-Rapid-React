package frc.robot.team3407.commandbased;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;


// Runnable can already be passed into most commandscheduler methods, but this acts as a wrapper for use in creating command groups
public class LambdaCommand extends CommandBase {

	private final Runnable run;
	private final boolean when_disabled;
	public LambdaCommand(Runnable r) {
		this.run = r;
		this.when_disabled = true;
	}
	public LambdaCommand(Runnable r, boolean wd) {
		this.run = r;
		this.when_disabled = wd;
	}
	@Override public void initialize() { this.run.run(); }
	@Override public boolean isFinished() { return true; }
	@Override public boolean runsWhenDisabled() { return this.when_disabled; }


	public static class Continuous extends LambdaCommand {

		private final BooleanSupplier is_finished;
		public Continuous(Runnable r, BooleanSupplier f, boolean wd) {
			super(r, wd);
			this.is_finished = f;
		}
		public Continuous(Runnable r) { this(r, ()->false, true); }
		public Continuous(Runnable r, boolean wd) { this(r, ()->false, wd); }
		public Continuous(Runnable r, BooleanSupplier f) { this(r, f, true); }
		
		@Override public void initialize() {}
		@Override public void execute() { super.run.run(); }
		@Override public boolean isFinished() { return this.is_finished.getAsBoolean(); }


	}

	public static class Singular extends LambdaCommand {

		boolean hasrun = false;
		public Singular(Runnable r) {
			super(r);
		}
		public Singular(Runnable r, boolean wd) {
			super(r, wd);
		}

		@Override public void initialize() {
			if(!this.hasrun) {
				super.initialize();
				this.hasrun = true;
			}
		}


	}


}