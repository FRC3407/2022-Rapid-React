package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.commands.*;
import frc.robot.modules.common.*;
import frc.robot.modules.common.Input.*;
import frc.robot.modules.common.EventTriggers.*;
import frc.robot.modules.vision.java.VisionServer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Runtime extends TimedRobot {

	private final InputDevice input = new InputDevice(0);
	private final DriveBase drivebase = new DriveBase(Constants.drivebase_map, Constants.drivebase_settings);

	private final Positions.LinearMotionIntegrator 
		posX = new Positions.LinearMotionIntegrator(1, 0.1, 0.1), 
		posY = new Positions.LinearMotionIntegrator(1, 0.005, 0.005);
	private final Timer timer = new Timer();
	private final Positions gyro = new Positions();

	public Runtime() {
		System.out.println("RUNTIME INITIALIZATION");
	}


	@Override public void robotInit() {
		VisionServer.Get();

		Xbox.Digital.LB.getButton(input).whenPressed(VisionServer.getPipelineIncrementCommand());
		Xbox.Digital.RB.getButton(input).whenPressed(VisionServer.getPipelineDecrementCommand());
		Xbox.Digital.DT.getButton(input).whenPressed(VisionServer.getCameraIncrementCommand());
		Xbox.Digital.DB.getButton(input).whenPressed(VisionServer.getCameraDecrementCommand());
		Xbox.Digital.START.getButton(input).whenPressed(VisionServer.getPipelineToggleCommand());
		Xbox.Digital.BACK.getButton(input).whenPressed(VisionServer.getToggleStatisticsCommand());

		Xbox.Digital.DR.getButton(input).whenPressed(new TestCommand("Dpad right"));
		Xbox.Digital.DL.getButton(input).whenPressed(new TestCommand("Dpad left"));

		TeleopTrigger.Get().whenActive(this.drivebase.tankDrive(Xbox.Analog.LY.getCallback(input), Xbox.Analog.RY.getCallback(input)));
		AutonomousTrigger.Get().whenActive(new CargoTurn(this.drivebase, DriverStation.getAlliance()));
	}
	@Override public void robotPeriodic() { CommandScheduler.getInstance().run(); }

	@Override public void disabledInit() {}
	@Override public void disabledPeriodic() {}

	@Override public void autonomousInit() {}
	@Override public void autonomousPeriodic() {
		//System.out.println(RapidReactVision.getClosestAllianceCargo(DriverStation.getAlliance()).lr);
	}

	@Override public void teleopInit() {}
	@Override public void teleopPeriodic() {}
	@Override public void teleopExit() {
		//this.drivebase.tankDrive().cancel();
		this.drivebase.getDecelerateCommand().schedule();
	}

	@Override public void testInit() {
		// Cancels all running commands at the start of test mode.
		//CommandScheduler.getInstance().cancelAll();
		this.timer.reset();
		this.timer.start();
	}
	@Override public void testPeriodic() {
		//double x = this.input.getRawAxis(Xbox.Analog.LX.value), y = this.input.getRawAxis(Xbox.Analog.LY.value);
		double x = this.gyro.getBuiltIn().getX()*9.81*3.2, y = this.gyro.getBuiltIn().getY()*9.81*3.2;
		if(Math.abs(x) > 0.05) {
			this.posX.update(x, this.timer.get());
		} else {
			this.posX.update(0, this.timer.get());
		}
		if(Math.abs(y) > 0.05) {
			this.posY.update(y, this.timer.get());
		} else {
			this.posY.update(0, this.timer.get());
		}
		timer.reset();
		timer.start();

		System.out.println("Angle" + this.gyro.getIMU().getAngle());

		NetworkTableInstance.getDefault().getTable("Motion").getSubTable("Velocity").getEntry("X").setNumber(this.posX.getVelocity());
		NetworkTableInstance.getDefault().getTable("Motion").getSubTable("Velocity").getEntry("Y").setNumber(this.posY.getVelocity());
		NetworkTableInstance.getDefault().getTable("Motion").getSubTable("Position").getEntry("X").setNumber(this.posX.getPosition());
		NetworkTableInstance.getDefault().getTable("Motion").getSubTable("Position").getEntry("Y").setNumber(this.posY.getPosition());
		NetworkTableInstance.getDefault().getTable("Motion").getEntry("Angle").setNumber(this.gyro.getAngle());
	}

}