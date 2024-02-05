// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// CAN + SPARKMAX
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Vision
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix.sensors.CANCoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private CANSparkMax translateMotor1 = new CANSparkMax(1, MotorType.kBrushless); // translation - odds
  private CANSparkMax rotateMotor2 = new CANSparkMax(2, MotorType.kBrushless); // rotation - evens
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

  private SparkMaxPIDController RM2_PidController;
  private SparkMaxPIDController RM4_PidController;
  private SparkMaxPIDController RM6_PidController;
  private SparkMaxPIDController RM8_PidController;

  private RelativeEncoder RM2_Encoder;
  private RelativeEncoder RM4_Encoder;
  private RelativeEncoder RM6_Encoder;
  private RelativeEncoder RM8_Encoder;
  private double convFactor = 5.62279270244;
  // private double convFactor = 1;

  private final PS4Controller PS4joystick = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station
  
  private final Timer m_timer = new Timer();

  private double filteredJoystickLeftY;
  private double filteredJoystickLeftX;
  // private double filteredJoystickRightY;
  private double filteredJoystickRightX;

  private final double MAX_LINEAR_VELOCITY = 0.23;
  private final double MAX_ANGULAR_VELOCITY = 0.1;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  

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
    new Thread(() -> {
      UsbCamera usbCamera = CameraServer.startAutomaticCapture();
      // UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
      usbCamera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

      cvSink.setSource(usbCamera);
      Point upleft = new Point(0, 0);
      Point downright = new Point(100, 100);
      Scalar color = new Scalar(0, 0, 255);
      Mat sourceMat = new Mat();
      
      while(true){
        cvSink.grabFrame(sourceMat);
        Imgproc.rectangle(sourceMat, upleft, downright, color);
        outputStream.putFrame(sourceMat);
      }
    }).start();

    // Creates the CvSource and MjpegServer [2] and connects them
    // CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    // CvSink cvSink = CameraServer.getVideo();
    // CvSource outputStream = CameraServer.putVideo("Blur", 320, 240);
    // CameraServer.getVideo();

    RM2_PidController = rotateMotor2.getPIDController();
    RM2_Encoder = rotateMotor2.getEncoder();
    RM2_Encoder.setPositionConversionFactor(convFactor);

    RM4_PidController = rotateMotor4.getPIDController();
    // RM4_PidController.setPositionPIDWrappingEnabled(true);
    // RM4_PidController.setPositionPIDWrappingMinInput(-1 * 180);
    // RM4_PidController.setPositionPIDWrappingMaxInput(180);
    // RM4_PidController.setOutputRange(-1 * 180, 180);
    RM4_Encoder = rotateMotor4.getEncoder();
    RM4_Encoder.setPositionConversionFactor(convFactor);

    RM6_PidController = rotateMotor6.getPIDController();
    RM6_Encoder = rotateMotor6.getEncoder();
    RM6_Encoder.setPositionConversionFactor(convFactor);

    RM8_PidController = rotateMotor8.getPIDController();
    RM8_Encoder = rotateMotor8.getEncoder();
    RM8_Encoder.setPositionConversionFactor(convFactor);

    // PID coefficients, RM2
    kP = 1e-6; 
    kI = 1e-8;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // // PID coefficients, RM4
    // kP = 1e-6; 
    // kI = 1e-8;
    // kD = 0; 
    // kIz = 0; 
    // kFF = 0.000156; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;

    // // PID coefficients, RM6
    // kP = 1e-6; 
    // kI = 1e-8;
    // kD = 0; 
    // kIz = 0; 
    // kFF = 0.000156; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;

    // // PID coefficients, RM8
    // kP = 1e-6; 
    // kI = 1e-8;
    // kD = 0; 
    // kIz = 0; 
    // kFF = 0.000156; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    RM2_PidController.setP(kP);
    RM2_PidController.setI(kI);
    RM2_PidController.setD(kD);
    RM2_PidController.setIZone(kIz);
    RM2_PidController.setFF(kFF);
    RM2_PidController.setOutputRange(kMinOutput, kMaxOutput);

    RM4_PidController.setP(kP);
    RM4_PidController.setI(kI);
    RM4_PidController.setD(kD);
    RM4_PidController.setIZone(kIz);
    RM4_PidController.setFF(kFF);
    RM4_PidController.setOutputRange(kMinOutput, kMaxOutput);

    RM6_PidController.setP(kP);
    RM6_PidController.setI(kI);
    RM6_PidController.setD(kD);
    RM6_PidController.setIZone(kIz);
    RM6_PidController.setFF(kFF);
    RM6_PidController.setOutputRange(kMinOutput, kMaxOutput);

    RM8_PidController.setP(kP);
    RM8_PidController.setI(kI);
    RM8_PidController.setD(kD);
    RM8_PidController.setIZone(kIz);
    RM8_PidController.setFF(kFF);
    RM8_PidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    RM2_PidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    RM2_PidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    RM2_PidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    RM2_PidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    RM4_PidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    RM4_PidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    RM4_PidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    RM4_PidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    RM6_PidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    RM6_PidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    RM6_PidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    RM6_PidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    RM8_PidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    RM8_PidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    RM8_PidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    RM8_PidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    // Get Absolute encoder postion to calculate offset
    double cancodervalue1 = coder1.getAbsolutePosition();
    double cancodervalue2 = coder2.getAbsolutePosition();
    double cancodervalue3 = coder3.getAbsolutePosition();
    double cancodervalue4 = coder4.getAbsolutePosition();

    // Apply the offset to relative encoder
    // RM2_Encoder.setPosition(cancodervalue1 + 14.589844);
    // RM4_Encoder.setPosition(cancodervalue2 - 129.023438);
    // RM6_Encoder.setPosition(cancodervalue3 + 117.949219);
    // RM8_Encoder.setPosition(cancodervalue4 - 56.425781);

    // double RM2_EncPos = wrapEncoderValues(cancodervalue1 + 14.589844);
    // RM2_Encoder.setPosition(RM2_EncPos);
    // double RM4_EncPos = wrapEncoderValues(cancodervalue2 - 129.023438);
    // RM4_Encoder.setPosition(RM4_EncPos);
    // double RM6_EncPos = wrapEncoderValues(cancodervalue3 + 117.949219);
    // RM6_Encoder.setPosition(RM6_EncPos);
    // double RM8_EncPos = wrapEncoderValues(cancodervalue4 - 56.425781);
    // RM8_Encoder.setPosition(RM8_EncPos);

    RM2_Encoder.setPosition(0);
    RM4_Encoder.setPosition(0);
    RM6_Encoder.setPosition(0);
    RM8_Encoder.setPosition(0);
  }

  // Private function defintions
  // ------------------------------------------------------------------------------

  /*
   * Returns a filtered value of some raw input by setting values in a range
   * from -0.1 to 0.1 to 0.
   */
  private double highpassFilter(double rawInput){
    if(Math.abs(rawInput) < 0.1){
      return 0.0;
    }else{
      return rawInput;
    }
  }

  /*
   * Returns a filtered value of a joystick input by scaling down 
   * values by either a maximum linear velocity or maximum angular 
   * velocity depending on which joystick input is being read. 
   */
  private double filterJoystick(double rawInput, boolean linear){
    double filtered = highpassFilter(rawInput);
    if(linear){
      filtered *= MAX_LINEAR_VELOCITY;
    }else{
      filtered *= MAX_ANGULAR_VELOCITY;
    }
    return filtered;
  }

  /*
   * Returns the magnitude of elements x and y
   */
  private double mag(double x, double y){
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  /*
   * Returns a 2x1 double array. First element is the optimized angle
   * to snap to, second element is either -1.0 to signify wheel velocity
   * must be inverted or 1.0 otherwise.
   * Both desired and current are in the range of -180 to 180
   */
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

  /*
   * Returns a (relative) encoder position value mapped onto a range
   * of -180 to 180
   * Takes in current relative encoder position as a parameter
   */
  private double wrapEncoderValues(double EncPos) {
    if ((EncPos >= -180.0) & (EncPos < 180.0)) {
      return EncPos;
    }
    
    else if ((EncPos < -180.0)){
      return 180.0 - (((-1 * EncPos) + 180.0) % 360);
    }

    else if ((EncPos > 180.0)){
      return -180.0 + ((EncPos - 180.0) % 360);
    }

    return EncPos;
  }
  // ------------------------------------------------------------------------------

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // Obtain and print absolute and relative encoder values
    double cancodervalue1 = coder1.getAbsolutePosition();
    SmartDashboard.putNumber("CANCODER 1", cancodervalue1);
    double RM2_EncPos = wrapEncoderValues(RM2_Encoder.getPosition());
    SmartDashboard.putNumber("RM2_Encoder Value", RM2_EncPos);

    double cancodervalue2 = coder2.getAbsolutePosition();
    SmartDashboard.putNumber("CANCODER 2", cancodervalue2);
    double RM4_EncPos = wrapEncoderValues(RM4_Encoder.getPosition());
    SmartDashboard.putNumber("RM4_Encoder Value", RM4_EncPos);

    double cancodervalue3 = coder3.getAbsolutePosition();
    SmartDashboard.putNumber("CANCODER 3", cancodervalue3);
    double RM6_EncPos = wrapEncoderValues(RM6_Encoder.getPosition());
    SmartDashboard.putNumber("RM6_Encoder Value", RM6_EncPos);

    double cancodervalue4 = coder4.getAbsolutePosition();
    SmartDashboard.putNumber("CANCODER 4", cancodervalue4);
    double RM8_EncPos = wrapEncoderValues(RM8_Encoder.getPosition());
    SmartDashboard.putNumber("RM8_Encoder Value", RM8_EncPos);

    // RM2_PidController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    // RM4_PidController.setReference(0, CANSparkMax.ControlType.kPosition);
    // RM6_PidController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    // RM8_PidController.setReference(0, CANSparkMax.ControlType.kSmartMotion);

    // Obtain filtered Joystick Inputs
    filteredJoystickLeftY = filterJoystick(PS4joystick.getLeftY(), true);
    filteredJoystickLeftX = filterJoystick(PS4joystick.getLeftX(), true);
    filteredJoystickRightX = filterJoystick(PS4joystick.getRightX(), false);

    // Define chassis speeds according to joystick inputs and new rotated orientation
    double Vx = MAX_LINEAR_VELOCITY * (-1) * filteredJoystickLeftY;
    SmartDashboard.putNumber("Vx",  Vx);

    double Vy = MAX_LINEAR_VELOCITY * (-1) * filteredJoystickLeftX;
    SmartDashboard.putNumber("Vy", Vy);

    double omega = MAX_ANGULAR_VELOCITY * (-1) * filteredJoystickRightX;
    SmartDashboard.putNumber("w (omega)", omega);
    
    // Chassis speed vector
    double Vr = mag(Vx, Vy);
    SmartDashboard.putNumber("Vr", Vr);

    // Norm of chassis speed vector 
    double Vr_norm[] = new double[]{Vx/Vr, Vy/Vr};
    // SmartDashboard.putNumber("Vr_norm[0]", Vr_norm[0]);
    // SmartDashboard.putNumber("Vr_norm[1]", Vr_norm[1]);

    // CvSource outputStream = CameraServer.putVideo("Rectangle", 320, 240);
    // Mat mat = new Mat();
    // SmartDashboard.putNumber("Average RGB", );

    // Constants
    double wheelradius;
    double x = 12.125;
    double y = 12.125;
    double d;
    
    // double wheel_radius = 1.5;

    if (omega == 0 & Vx != 0 & Vy != 0) {
      // No rotation, CASE 1
      SmartDashboard.putNumber("Case", 1);

      double DESIRED = ((Math.atan2(Vy,Vx) * 180) / Math.PI);
      SmartDashboard.putNumber("DESIRED_BEFORE", DESIRED);
      
      // double AngleAtan2 = Math.atan2(Vy,Vx);
      // SmartDashboard.putNumber("AngleAtan2", AngleAtan2);

      SmartDashboard.putNumber("CURRENT_BEFORE", RM6_EncPos);

      double res1[] = angleDiff(DESIRED, RM2_EncPos);
      double res2[] = angleDiff(DESIRED, RM4_EncPos);
      double res3[] = angleDiff(DESIRED, RM6_EncPos);
      double res4[] = angleDiff(DESIRED, RM8_EncPos);

      SmartDashboard.putNumber("Vwheel1", Vr * res1[1]);
      SmartDashboard.putNumber("SetAngle1", res1[0]);
      // SmartDashboard.putNumber("Vwheel2", Vr * res2[1]);
      // SmartDashboard.putNumber("SetAngle2", res2[0]);
      
      // Set Rotation Motor Position based on encoders
      // RM2_PidController.setReference(res1[0], CANSparkMax.ControlType.kSmartMotion);
      // RM4_PidController.setReference(res2[0], CANSparkMax.ControlType.kSmartMotion);
      // RM6_PidController.setReference(res3[0], CANSparkMax.ControlType.kSmartMotion);
      // RM8_PidController.setReference(res4[0], CANSparkMax.ControlType.kSmartMotion);

      // Then set translation motor speeds
      // translateMotor1.set(Vr * res1[1]);
      // translateMotor3.set(Vr * res2[1]);
      // translateMotor5.set(Vr * res3[1]);
      // translateMotor7.set(Vr * res4[1]); 

    }
    else if (omega != 0 & Vx != 0 & Vy != 0) {
      // General Case, CASE 2
      SmartDashboard.putNumber("Case", 2);

      // d = Vr/omega;
      // // SmartDashboard.putNumber("d", d);

      // double ICC[] = new double[]{d*(-1)*Vr_norm[1], d*Vr_norm[0]};
      // // SmartDashboard.putNumber("ICC[0]", ICC[0]);
      // // SmartDashboard.putNumber("ICC[1]", ICC[1]);

      // double Vwheel1_xy[] = new double[]{ICC[0] - x, ICC[1] - y};
      // double Vwheel2_xy[] = new double[]{ICC[0] - x, ICC[1] + y};
      // double Vwheel3_xy[] = new double[]{ICC[0] + x, ICC[1] + y};
      // double Vwheel4_xy[] = new double[]{ICC[0] + x, ICC[1] - y};

      // double Vwheel1 = mag(Vwheel1_xy[0], Vwheel1_xy[1]) * omega;
      // double Vwheel2 = mag(Vwheel2_xy[0], Vwheel2_xy[1]) * omega;
      // double Vwheel3 = mag(Vwheel3_xy[0], Vwheel3_xy[1]) * omega;
      // double Vwheel4 = mag(Vwheel4_xy[0], Vwheel4_xy[1]) * omega;

      // SmartDashboard.putNumber("Vwheel3", Vwheel3);

      // double Omega_wheel1 = Math.atan2((-1)*Vwheel1_xy[0], Vwheel1_xy[1]);
      // double Omega_wheel2 = Math.atan2((-1)*Vwheel2_xy[0], Vwheel2_xy[1]);
      // double Omega_wheel3 = Math.atan2((-1)*Vwheel3_xy[0], Vwheel3_xy[1]);
      // double Omega_wheel4 = Math.atan2((-1)*Vwheel4_xy[0], Vwheel4_xy[1]);

      // SmartDashboard.putNumber("Omega_wheel3", Omega_wheel3);
 
    }
    else if (Vx == 0.0 & Vy == 0.0 & omega != 0) {
      // Rotate in place, CASE 3
      SmartDashboard.putNumber("Case", 3);
      double res1[] = angleDiff(135, RM2_EncPos);
      double res2[] = angleDiff(45, RM4_EncPos);
      double res3[] = angleDiff(315, RM6_EncPos);
      double res4[] = angleDiff(225, RM8_EncPos);

      
      if (omega > 0) {
        // RM2_PidController.setReference(res1[0], CANSparkMax.ControlType.kSmartMotion);
        // RM4_PidController.setReference(res2[0], CANSparkMax.ControlType.kSmartMotion);
        // RM6_PidController.setReference(res3[0], CANSparkMax.ControlType.kSmartMotion);
        // RM8_PidController.setReference(res4[0], CANSparkMax.ControlType.kSmartMotion);

        // translateMotor1.set(res1[1] * omega * mag(x,y));
        // translateMotor3.set(res2[1] * omega * mag(x,y));
        // translateMotor5.set(res3[1] * omega * mag(x,y));
        // translateMotor7.set(res4[1] * omega * mag(x,y));
        SmartDashboard.putNumber("Vwheel1", omega * res1[1] * mag(x,y));
        SmartDashboard.putNumber("SetAngle1", res1[0]);
      }
      else if (omega < 0) {
        // RM2_PidController.setReference(res1[0], CANSparkMax.ControlType.kSmartMotion);
        // RM4_PidController.setReference(res2[0], CANSparkMax.ControlType.kSmartMotion);
        // RM6_PidController.setReference(res3[0], CANSparkMax.ControlType.kSmartMotion);
        // RM8_PidController.setReference(res4[0], CANSparkMax.ControlType.kSmartMotion);

        // translateMotor1.set(-1 * res1[1] * omega * mag(x,y));
        // translateMotor3.set(-1 * res2[1] * omega * mag(x,y));
        // translateMotor5.set(-1 * res3[1] * omega * mag(x,y));
        // translateMotor7.set(-1 * res4[1] * omega * mag(x,y));
        SmartDashboard.putNumber("Vwheel1", -1 * omega * res1[1] * mag(x,y));
        SmartDashboard.putNumber("SetAngle1", res1[0]);
      }
    }
    // else {
    //     // No joystick input, CASE 4
    //     SmartDashboard.putNumber("Case", 4);
    //     translateMotor1.set(0);
    //     translateMotor3.set(0);
    //     translateMotor5.set(0);
    //     translateMotor7.set(0);

    //     rotateMotor2.set(0);
    //     rotateMotor4.set(0);
    //     rotateMotor6.set(0);
    //     rotateMotor8.set(0);
    // }

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
