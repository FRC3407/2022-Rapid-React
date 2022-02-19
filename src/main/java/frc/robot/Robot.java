// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final WPI_TalonSRX m_leftfrontDrive = new WPI_TalonSRX(0);
  private final WPI_TalonSRX m_rightfrontDrive = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_leftbackDrive = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_rightbackDrive = new WPI_TalonSRX(2);
  //private final PWMVictorSPX m_leftfrontDrive = new PWMVictorSPX(0);
  //private final PWMVictorSPX m_rightfrontDrive = new PWMVictorSPX(3);
  //private final PWMVictorSPX m_leftbackDrive = new PWMVictorSPX(1);
  //private final PWMVictorSPX m_rightbackDrive = new PWMVictorSPX(2);
  //private final PWMVictorSPX ballShooter = new PWMVictorSPX(4);
  private final MotorControllerGroup left_side = new MotorControllerGroup(m_leftfrontDrive, m_leftbackDrive);
  private final MotorControllerGroup right_side = new MotorControllerGroup(m_rightfrontDrive, m_rightbackDrive);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(left_side, right_side);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    right_side.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  // At 0.5 speed it went 3 tiles at 1 sec, 5.25 tiles at 2 secs, and 7.555 tiles at 3 secs
  // At 0.5 speed .375 secs = 45 degrees, .75=90 degrees, 1.5 secs=180 degrees
  @Override
  public void autonomousPeriodic() {
    // Drive for 3 seconds
    if (m_timer.get() < 5) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
      //ballShooter.set(0.5);//shoot ball
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
