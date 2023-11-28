// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.CANCoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private CANSparkMax translateMotor1 = new CANSparkMax(2, MotorType.kBrushless); // rotation - evens
  private CANSparkMax rotateMotor2 = new CANSparkMax(1, MotorType.kBrushless); // translation - odds
  private CANSparkMax translateMotor3 = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax rotateMotor4 = new CANSparkMax(3, MotorType.kBrushless); 
  private CANSparkMax translateMotor5 = new CANSparkMax(6, MotorType.kBrushless); 
  private CANSparkMax rotateMotor6 = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax translateMotor7 = new CANSparkMax(8, MotorType.kBrushless);
  private CANSparkMax rotateMotor8 = new CANSparkMax(7, MotorType.kBrushless); 

  private CANCoder coder1 = new CANCoder(1);

  private final PS4Controller PS4joystick = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station
  
  private final Timer m_timer = new Timer();
  private double filteredJoystickLeftY;
  private double filteredJoystickLeftX;
  private double filteredJoystickRightY;
  private double filteredJoystickRightX;

  private final double MAX_LINEAR_VELOCITY = 0.3;
  private final double MAX_ANGULAR_VELOCITY = 0.5;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightDrive.setInverted(true);
    // CameraServer.startAutomaticCapture();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    // if (m_timer.get() < 2.0) {
    //   // Drive forwards half speed, make sure to turn input squaring off
    //   m_robotDrive.arcadeDrive(0.5, 0.0, false);
    // } else {
    //   m_robotDrive.stopMotor(); // stop robot
    // }

    // m_robotDrive.arcadeDrive(0.30, 0.0);
    // rotateMotor.set(0.1);
    // translateMotor.set(0.1);
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  private double clamp(double raw, double min, double max){
    return Math.max(min, Math.min(max, raw));
  }

  private double highpassFilter(double rawInput){
    if(Math.abs(rawInput) < 0.1){
      return 0.0;
    }else{
      return rawInput;
    }
  }

  private double filterJoystick(double rawInput, boolean linear){
    double filtered = highpassFilter(rawInput);
    if(linear){
      filtered *= MAX_LINEAR_VELOCITY;
    }else{
      filtered *= MAX_ANGULAR_VELOCITY;
    }
    return filtered;
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // m_robotDrive.arcadeDrive(-PS4joystick.getLeftY(), -PS4joystick.getRightX());
    //System.out.printf("Left Joystick Y: %.2f", PS4joystick.getLeftY());
    //System.out.println("example");
    filteredJoystickLeftY = filterJoystick(PS4joystick.getLeftY(), true);
    filteredJoystickLeftX = filterJoystick(PS4joystick.getLeftX(), true);
    filteredJoystickRightY = filterJoystick(PS4joystick.getRightY(), true);
    filteredJoystickRightX = filterJoystick(PS4joystick.getRightX(), false);

    SmartDashboard.putNumber("Joystick Left Y", filteredJoystickLeftY);
    SmartDashboard.putNumber("Joystick Left X", filteredJoystickLeftX);
    SmartDashboard.putNumber("Joystick Right Y", filteredJoystickRightY);
    SmartDashboard.putNumber("Joystick Right X", filteredJoystickRightX);

    double cancodervalue = coder1.getPosition();

    SmartDashboard.putNumber("CANCODER 1", cancodervalue);


    translateMotor1.set(filteredJoystickLeftX);
    translateMotor3.set(filteredJoystickLeftX);
    translateMotor5.set(filteredJoystickLeftX);
    translateMotor7.set(filteredJoystickLeftX);

    rotateMotor2.set(filteredJoystickRightX);
    rotateMotor4.set(filteredJoystickRightX);
    rotateMotor6.set(filteredJoystickRightX);
    rotateMotor8.set(filteredJoystickRightX);
  }

  protected void execute() {
    SmartDashboard.putNumber("Joystick Left Y", PS4joystick.getLeftY());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
