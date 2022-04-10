package frc.robot.vision.java;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;


public final class VisionSubsystem {

	private static class InstantGlobal extends CommandBase {
		@Override public boolean isFinished() { return true; }
		@Override public boolean runsWhenDisabled() { return true; }
	}

	public static class IncrementCamera extends InstantGlobal {
		private static final IncrementCamera inst = new IncrementCamera();
		private IncrementCamera() {}
		public static IncrementCamera Get() { return inst; }
		@Override public void initialize() { 
			System.out.println("INCREMENT CAMERA");
			VisionServer.incrementCamera(); 
		}
	}
	public static class DecrementCamera extends InstantGlobal {
		private static final DecrementCamera inst = new DecrementCamera();
		private DecrementCamera() {}
		public static DecrementCamera Get() { return inst; }
		@Override public void initialize() { 
			System.out.println("DECREMENT CAMERA");
			VisionServer.decrementCamera(); 
		}
	}
	public static class IncrementPipeline extends InstantGlobal {
		private static final IncrementPipeline inst = new IncrementPipeline();
		private IncrementPipeline() {}
		public static IncrementPipeline Get() { return inst; }
		@Override public void initialize() { 
			System.out.println("INCREMENT PIPELINE");
			VisionServer.incrementPipeline(); }
	}
	public static class DecrementPipeline extends InstantGlobal {
		private static final DecrementPipeline inst = new DecrementPipeline();
		private DecrementPipeline() {}
		public static DecrementPipeline Get() { return inst; }
		@Override public void initialize() { 
			System.out.println("DECREMENT PIPELINE");
			VisionServer.decrementPipeline(); 
		}
	}
	public static class ToggleStatistics extends InstantGlobal {
		private static final ToggleStatistics inst = new ToggleStatistics();
		private ToggleStatistics() {}
		public static ToggleStatistics Get() { return inst; }
		@Override public void initialize() { 
			System.out.println("TOGGLE STATISTICS");
			VisionServer.toggleStatistics(); 
		}
	}
	public static class ToggleProcessing extends InstantGlobal {
		private static final ToggleProcessing inst = new ToggleProcessing();
		private ToggleProcessing() {}
		public static ToggleProcessing Get() { return inst; }
		@Override public void initialize() { 
			System.out.println("TOGGLE PROCESSING");
			VisionServer.toggleProcessingEnabled(); 
		}
	}


}