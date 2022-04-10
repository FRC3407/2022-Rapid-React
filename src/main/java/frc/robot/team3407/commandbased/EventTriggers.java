package frc.robot.team3407.commandbased;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;


public class EventTriggers {

	public static class EnabledTrigger extends Trigger {
		private static EnabledTrigger singleton = new EnabledTrigger();
		private EnabledTrigger() {
			super(()->DriverStation.isEnabled());
		}
		public static EnabledTrigger Get() { return EnabledTrigger.singleton; }
	}
	
	public static class DisabledTrigger extends Trigger {
		private static DisabledTrigger singleton = new DisabledTrigger();
		private DisabledTrigger() {
			super(()->DriverStation.isDisabled());
		}
		public static DisabledTrigger Get() { return DisabledTrigger.singleton; }
	}

	public static class TeleopTrigger extends Trigger {
		private static TeleopTrigger singleton = new TeleopTrigger();
		private TeleopTrigger() {
			super(()->DriverStation.isTeleopEnabled());
		}
		public static TeleopTrigger Get() { return TeleopTrigger.singleton; }
	}

	public static class AutonomousTrigger extends Trigger {
		private static AutonomousTrigger singleton = new AutonomousTrigger();
		private AutonomousTrigger() {
			super(()->DriverStation.isAutonomousEnabled());
		}
		public static AutonomousTrigger Get() { return AutonomousTrigger.singleton; }
	}

	public static class TestTrigger extends Trigger {
		private static TestTrigger singleton = new TestTrigger();
		private TestTrigger() {
			super(()->DriverStation.isTest());
		}
		public static TestTrigger Get() { return TestTrigger.singleton; }
	}

}