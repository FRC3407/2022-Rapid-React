package frc.robot.team3407.commandbased;

import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * Bascially just wraps a boolean
 */
public class StaticTrigger extends Trigger {

	private boolean state;

	public StaticTrigger(boolean init) {
		this.state = init;
	}

	public void enable() { this.state = true; }
	public void disable() { this.state = false; }
	public void setState(boolean val) { this.state = val; }

	@Override public boolean get() {
		return this.state;
	}


}