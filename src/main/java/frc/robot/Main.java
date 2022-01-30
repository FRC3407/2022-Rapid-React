package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
	public static void main(String... args) 
		{ RobotBase.startRobot(Runtime::new); }
		//	^^^ 'Robot::new' changed to 'Runtime::new', as this is the class that extends TimedRobot

}