package frc.robot.team3407.commandbased;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Trigger becomes active after the condition is true, then never becomes unactive
 */
public class EternalTrigger extends Trigger{

	private boolean has_triggered = false;

	public EternalTrigger(Trigger t) {
		t.whenActive(()->this.has_triggered = true);
	}
	public EternalTrigger(BooleanSupplier t) { this(new Trigger(t)); }

	@Override public boolean get() {
		return this.has_triggered;
	}


}