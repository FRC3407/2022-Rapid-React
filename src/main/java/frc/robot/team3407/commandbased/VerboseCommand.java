package frc.robot.team3407.commandbased;

public interface VerboseCommand {

	default void message(String message) {
		System.out.println(getClass().getSimpleName() + ": " + message);
	}

	default void start() {
		System.out.println(getClass().getSimpleName() + ": Running...");
	}
	default void finish(boolean i) {
		System.out.println(getClass().getSimpleName() +  (i ? ": Terminated." : ": Completed."));
	}


}