// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;

import com.ctre.phoenix.sensors.CANCoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private CANSparkMax translateMotor1 = new CANSparkMax(1, MotorType.kBrushless); // rotation - evens
  private CANSparkMax rotateMotor2 = new CANSparkMax(2, MotorType.kBrushless); // translation - odds
  private CANSparkMax translateMotor3 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rotateMotor4 = new CANSparkMax(4, MotorType.kBrushless); 
  private CANSparkMax translateMotor5 = new CANSparkMax(5, MotorType.kBrushless); 
  private CANSparkMax rotateMotor6 = new CANSparkMax(6, MotorType.kBrushless);
  private CANSparkMax translateMotor7 = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax rotateMotor8 = new CANSparkMax(8, MotorType.kBrushless); 

  private CANCoder coder1 = new CANCoder(1);
  private CANCoder coder2 = new CANCoder(2);
  private CANCoder coder3 = new CANCoder(3);
  private CANCoder coder4 = new CANCoder(4);

  private final PS4Controller PS4joystick = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station
  
  private final Timer m_timer = new Timer();

  private double filteredJoystickLeftY;
  private double filteredJoystickLeftX;
  // private double filteredJoystickRightY;
  private double filteredJoystickRightX;

  private final double MAX_LINEAR_VELOCITY = 0.23;
  private final double MAX_ANGULAR_VELOCITY = 0.1;

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
    // UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    // usbCamera.setResolution(320, 240);

    // CvSink cvSink = CameraServer.getVideo();
    // CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);

    // cvSink.setSource(usbCamera);


    // Creates the CvSource and MjpegServer [2] and connects them
    // CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    // CvSink cvSink = CameraServer.getVideo();
    // CvSource outputStream = CameraServer.putVideo("Blur", 320, 240);
    // CameraServer.getVideo();
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

  private double mag(double x, double y){
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  private double[] angleDiff(double desired, double current){
    double res[] = new double[]{0,0};
    // SmartDashboard.putNumber("DESIRED_PARAM", desired);
    // SmartDashboard.putNumber("CURRENT_PARAM", current);
    if (Math.abs((current - desired)) < (Math.abs(desired - 180 - current))){
      res[0] = desired;
      res[1] = 1.0;
    }
    else {
      res[0] = desired - 180;
      res[1] = -1.0;
    }
    return res;
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    filteredJoystickLeftY = filterJoystick(PS4joystick.getLeftY(), true);
    filteredJoystickLeftX = filterJoystick(PS4joystick.getLeftX(), true);
    // filteredJoystickRightY = filterJoystick(PS4joystick.getRightY(), true);
    filteredJoystickRightX = filterJoystick(PS4joystick.getRightX(), false);

    double Vx = MAX_LINEAR_VELOCITY * (-1) * filteredJoystickLeftY;
    SmartDashboard.putNumber("Vx",  Vx);

    double Vy = MAX_LINEAR_VELOCITY * (-1) * filteredJoystickLeftX;
    SmartDashboard.putNumber("Vy", Vy);

    double omega = MAX_ANGULAR_VELOCITY * (-1) * filteredJoystickRightX;
    SmartDashboard.putNumber("w (omega)", omega);

    // CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);
    // Mat mat = new Mat();

    // SmartDashboard.putNumber("Average RGB", );
    double cancodervalue1 = coder1.getAbsolutePosition() - 180;
    SmartDashboard.putNumber("CANCODER 1", cancodervalue1);
    double cancodervalue2 = coder2.getAbsolutePosition() - 180;
    SmartDashboard.putNumber("CANCODER 2", cancodervalue2);
    double cancodervalue3 = coder3.getAbsolutePosition() - 180;
    SmartDashboard.putNumber("CANCODER 3", cancodervalue3);
    double cancodervalue4 = coder4.getAbsolutePosition() - 180;
    SmartDashboard.putNumber("CANCODER 4", cancodervalue4);

    double Vr = mag(Vx, Vy);
    SmartDashboard.putNumber("Vr", Vr);

    double Vr_norm[] = new double[]{Vx/Vr, Vy/Vr};
    // SmartDashboard.putNumber("Vr_norm[0]", Vr_norm[0]);
    // SmartDashboard.putNumber("Vr_norm[1]", Vr_norm[1]);

    double d;
    double x = 12.125;
    double y = 12.125;
    
    // double wheel_radius = 1.5;

    if (omega != 0) {
      SmartDashboard.putNumber("Case", 2);

      d = Vr/omega;
      // SmartDashboard.putNumber("d", d);

      double ICC[] = new double[]{d*(-1)*Vr_norm[1], d*Vr_norm[0]};
      // SmartDashboard.putNumber("ICC[0]", ICC[0]);
      // SmartDashboard.putNumber("ICC[1]", ICC[1]);

      double Vwheel1_xy[] = new double[]{ICC[0] - x, ICC[1] - y};
      double Vwheel2_xy[] = new double[]{ICC[0] - x, ICC[1] + y};
      double Vwheel3_xy[] = new double[]{ICC[0] + x, ICC[1] + y};
      double Vwheel4_xy[] = new double[]{ICC[0] + x, ICC[1] - y};

      double Vwheel1 = mag(Vwheel1_xy[0], Vwheel1_xy[1]) * omega;
      double Vwheel2 = mag(Vwheel2_xy[0], Vwheel2_xy[1]) * omega;
      double Vwheel3 = mag(Vwheel3_xy[0], Vwheel3_xy[1]) * omega;
      double Vwheel4 = mag(Vwheel4_xy[0], Vwheel4_xy[1]) * omega;

      SmartDashboard.putNumber("Vwheel1", Vwheel1);
      // SmartDashboard.putNumber("Vwheel2", Vwheel2);
      // SmartDashboard.putNumber("Vwheel3", Vwheel3);
      // SmartDashboard.putNumber("Vwheel4", Vwheel4);

      double Omega_wheel1 = Math.atan2((-1)*Vwheel1_xy[0], Vwheel1_xy[1]);
      double Omega_wheel2 = Math.atan2((-1)*Vwheel2_xy[0], Vwheel2_xy[1]);
      double Omega_wheel3 = Math.atan2((-1)*Vwheel3_xy[0], Vwheel3_xy[1]);
      double Omega_wheel4 = Math.atan2((-1)*Vwheel4_xy[0], Vwheel4_xy[1]);

    }
    else if (Vx == 0.0 & Vy == 0.0 & omega != 0) {
      // TODO: Rotate in place
      SmartDashboard.putNumber("Case", 3);
    }
    else {
      SmartDashboard.putNumber("Case", 1);
      double DESIRED = ((Math.atan2(Vy,Vx) * 180) / Math.PI);
      SmartDashboard.putNumber("DESIRED_BEFORE", DESIRED);
      
      // double AngleAtan2 = Math.atan2(Vy,Vx);
      // SmartDashboard.putNumber("AngleAtan2", AngleAtan2);

      SmartDashboard.putNumber("CURRENT_BEFORE", cancodervalue1);

      double res1[] = angleDiff(DESIRED, cancodervalue1);
      // double res2[] = angleDiff(DESIRED, cancodervalue2);
      // double res3[] = angleDiff(DESIRED, cancodervalue3);
      // double res4[] = angleDiff(DESIRED, cancodervalue4);

      SmartDashboard.putNumber("Vwheel1", Vr * res1[1]);
      
      SmartDashboard.putNumber("SetAngle1", res1[0]);
      
      // TODO: Set Rotation Motor Position based on encoders

    }

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
