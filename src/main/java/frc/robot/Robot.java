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
  
  public final double[] offset1 = new double[]{14.589844, -165.585938};
  public final double[] offset2 = new double[]{51.064453, -130.253906};
  public final double[] offset3 = new double[]{117.949219, -60.292969};
  // public final double[] offset4 = new double[]{14.589844, -165.585938};

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
    // new Thread(() -> {
    //   UsbCamera usbCamera = CameraServer.startAutomaticCapture();
    //   // UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    //   usbCamera.setResolution(640, 480);

    //   CvSink cvSink = CameraServer.getVideo();
    //   CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

    //   cvSink.setSource(usbCamera);
    //   Point upleft = new Point(0, 0);
    //   Point downright = new Point(100, 100);
    //   Scalar color = new Scalar(0, 0, 255);
    //   Mat sourceMat = new Mat();
      
    //   while(true){
    //     cvSink.grabFrame(sourceMat);
    //     Imgproc.rectangle(sourceMat, upleft, downright, color);
    //     outputStream.putFrame(sourceMat);
    //   }
    // }).start();

    // Creates the CvSource and MjpegServer [2] and connects them
    // CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    // CvSink cvSink = CameraServer.getVideo();
    // CvSource outputStream = CameraServer.putVideo("Blur", 320, 240);
    // CameraServer.getVideo();
    
    RM2_PidController = rotateMotor2.getPIDController();
    RM2_Encoder = rotateMotor2.getEncoder();
    RM2_Encoder.setPositionConversionFactor(convFactor);

    RM4_PidController = rotateMotor4.getPIDController();
    RM4_Encoder = rotateMotor4.getEncoder();
    RM4_Encoder.setPositionConversionFactor(convFactor);

    rotateMotor6.restoreFactoryDefaults();
    RM6_PidController = rotateMotor6.getPIDController();
    RM6_Encoder = rotateMotor6.getEncoder();
    RM6_Encoder.setPositionConversionFactor(convFactor);

    RM8_PidController = rotateMotor8.getPIDController();
    RM8_Encoder = rotateMotor8.getEncoder();
    RM8_Encoder.setPositionConversionFactor(convFactor);

    // PID coefficients, RM2
    kP = 5e-5;
    // kP = 0; 
    kI = 1e-8;
    // kI = 0;
    // kD = 1e-7; 
    kD = 0;
    kIz = 1e-4; 
    // kFF = 0.00009091;
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 11000;

    // Smart Motion Coefficients
    maxVel = maxRPM; // rpm
    maxAcc = 1000;

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
    // double cancodervalue1 = coder1.getAbsolutePosition();
    // double cancodervalue2 = coder2.getAbsolutePosition();
    // double cancodervalue3 = coder3.getAbsolutePosition();

    // SmartDashboard.putNumber("RM6_Encoder Value WRAPPED", RM6_Encoder.getPosition());

    // while (Math.abs(242.050781 - RM6_Encoder.getPosition()) > 1.0) {
    //   cancodervalue3 = coder3.getAbsolutePosition();
    //   SmartDashboard.putNumber("CANCODER 3", cancodervalue3);
    //   RM6_Encoder.setPosition(cancodervalue3);
    //   SmartDashboard.putNumber("RM6_Encoder Value WRAPPED", RM6_Encoder.getPosition());
    //   RM6_PidController.setReference(242.050781, CANSparkMax.ControlType.kSmartMotion);
    // }

    // double cancodervalue4 = coder4.getAbsolutePosition();

    // SmartDashboard.putNumber("CANCODER 1", cancodervalue1);
    // SmartDashboard.putNumber("CANCODER 2", cancodervalue2);
    // SmartDashboard.putNumber("CANCODER 3", cancodervalue3);
    // SmartDashboard.putNumber("CANCODER 4", cancodervalue4);

    // Apply the offset to relative encoder
    // double RM2_EncPos;
    // double RM4_EncPos;
    // double RM6_EncPos;
    // double RM8_EncPos;

    // if (cancodervalue1 < 0) {
    //   RM2_EncPos = wrapEncoderValues(cancodervalue1 + 14.589844);
    // }
    // else {
    //   RM2_EncPos = wrapEncoderValues(cancodervalue1 - 165.585938);
    // }
    // RM2_Encoder.setPosition(RM2_EncPos);
    
    // if (cancodervalue2 < 0) {
    //   RM4_EncPos = wrapEncoderValues(cancodervalue2 + 51.064453);
    // }
    // else {
    //   RM4_EncPos = wrapEncoderValues(cancodervalue2 - 130.253906);
    // }
    // RM4_Encoder.setPosition(RM4_EncPos);

    // if (cancodervalue3 < 0) {
    //   RM6_EncPos = wrapEncoderValues(cancodervalue3 + 117.949219);
    // }
    // else {
    //   RM6_EncPos = wrapEncoderValues(cancodervalue3 - 60.292969);
    // }
    // RM6_Encoder.setPosition(RM6_EncPos);

    // if (cancodervalue4 < 0) {
    //   RM8_EncPos = wrapEncoderValues(cancodervalue4 - 56.425781);
    // }
    // else {
    //   RM8_EncPos = wrapEncoderValues(cancodervalue4 - 56.425781);
    // }
    // RM8_Encoder.setPosition(RM8_EncPos);

    // RM2_Encoder.setPosition(0);
    // RM4_Encoder.setPosition(0);
    // RM6_Encoder.setPosition(0);
    // RM8_Encoder.setPosition(0);
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
    // if (Math.abs((current - desired)) < (Math.abs(desired - 180 - current))){
    //   res[0] = desired;
    //   res[1] = 1.0;      
    // }
    // else {
    //   res[0] = desired - 180;
    //   res[1] = -1.0;
    // }
    double delta = wrapEncoderValues(desired - current);

    if (Math.abs(delta) < 90) {
      res[0] = delta;
      res[1] = 1.0;
    }
    else {
      res[0] = wrapEncoderValues(desired + 180);
      res[1] = -1;
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

  /*
   * Returns a (relative) encoder position value mapped onto a range
   * of 0 to 360
   * Takes in any current relative encoder position
   */
  private double wrapEncoderValues360 (double EncInitPos){
    if((EncInitPos >= 0.0) & (EncInitPos < 360.0)){
      return EncInitPos;
    }
    else if ((EncInitPos < 0.0)){
      return 360 - ((-1 * EncInitPos) % 360);
    }

    else if ((EncInitPos > 360.0)){
      return (EncInitPos % 360);
    }
    return EncInitPos;
  }

  // private double wrapEncoderValues360test (double EncInitPos){
  //   EncInitPos += 180;
  //   if (EncInitPos < 0){
  //     EncInitPos = 360 - ((-1*EncInitPos)%360);
  //   }
  //   else{
  //     EncInitPos = EncInitPos % 360;
  //     EncInitPos -= 180.0;
  //   }
  //   return EncInitPos;
  // }

  // ------------------------------------------------------------------------------

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // Obtain and print absolute and relative encoder values
    double new_offset3;

    double cancodervalue3 = coder3.getAbsolutePosition();
    SmartDashboard.putNumber("CANCODER 3", cancodervalue3);
    double newval = wrapEncoderValues(cancodervalue3 + offset3[0]);
    SmartDashboard.putNumber("newOFFSET", newval);

    RM6_Encoder.setPosition(newval);

    SmartDashboard.putNumber("RM6_Encoder Value WRAPPED", RM6_Encoder.getPosition());

    double setValue = -1*offset3[1];
    // if (cancodervalue3 > 0 & 180 - cancodervalue3 > 90) {
    //   new_offset3 = offset3[0];
    // }
    // else {
    //   new_offset3 = offset3[1];
    // }
    // RM6_PidController.setReference(setValue, CANSparkMax.ControlType.kSmartMotion);

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
