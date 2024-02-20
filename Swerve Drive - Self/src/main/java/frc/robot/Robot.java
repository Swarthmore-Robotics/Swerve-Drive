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

import javax.management.Descriptor;

import org.opencv.core.Core;

import com.ctre.phoenix.sensors.CANCoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  
  // indices into arrays of length 4
  public final int WHEEL_FL = 0;
  public final int WHEEL_FR = 1;
  public final int WHEEL_BR = 2;
  public final int WHEEL_BL = 3;

   // wheel encoders
  private CANCoder[] coders = new CANCoder[] {
    new CANCoder (1),
    new CANCoder(2),
    new CANCoder(3),
    new CANCoder (4),
  };
  
  // translation motors (odd)
  private CANSparkMax[] TranslationMotors = new CANSparkMax[] {
    new CANSparkMax(1, MotorType.kBrushless),
    new CANSparkMax(3, MotorType.kBrushless),
    new CANSparkMax(5, MotorType.kBrushless),
    new CANSparkMax(7, MotorType.kBrushless), 
  };

  // rotation motors (even)
  private CANSparkMax[] RotationMotors = new CANSparkMax[] {
    new CANSparkMax(2, MotorType.kBrushless),
    new CANSparkMax(4, MotorType.kBrushless),
    new CANSparkMax(6, MotorType.kBrushless),
    new CANSparkMax(8, MotorType.kBrushless), 
  };

  // TM Pid Controllers
  private SparkMaxPIDController TM1_PidController;
  private SparkMaxPIDController TM3_PidController;
  private SparkMaxPIDController TM5_PidController;
  private SparkMaxPIDController TM7_PidController;
  
  // RM Pid Controllers
  private SparkMaxPIDController RM2_PidController;
  private SparkMaxPIDController RM4_PidController;
  private SparkMaxPIDController RM6_PidController;
  private SparkMaxPIDController RM8_PidController;

  // RM Relative Encoders
  private RelativeEncoder RM2_Encoder;
  private RelativeEncoder RM4_Encoder;
  private RelativeEncoder RM6_Encoder;
  private RelativeEncoder RM8_Encoder;

  // TM Relative Encoders
  private RelativeEncoder TM1_Encoder;
  private RelativeEncoder TM3_Encoder;
  private RelativeEncoder TM5_Encoder;
  private RelativeEncoder TM7_Encoder;

  private double rotConvFactor = 5.62279270244;
  private double transConvFactor = 0.1875; // RPM
  // private double transConvFactor = 0.0014961835; // m/s

  // joystick controller
  private final PS4Controller PS4joystick = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station
  private double filteredJoystickLeftY;
  private double filteredJoystickLeftX;
  private double filteredJoystickRightX;

  // Rotation Motor PID Constants
  public double Rot_kP = 0.00008;
  public double Rot_kI = 0;
  public double Rot_kD = 0.0004;
  public double Rot_kIz = 0; 
  public double Rot_kFF = 0; 
  public double Rot_kMaxOutput = 1; 
  public double Rot_kMinOutput = -1;
  public double Rot_maxRPM = 11000;

  // Rotation Motor Smart Motion Coefficients
  public double Rot_maxVel = Rot_maxRPM; // rpm
  public double Rot_minVel = 0;
  public double Rot_maxAcc = 30000;
  public double Rot_allowedErr = 0.8; 

  // Translation Motor PID Constants
  public double Trans_kP = 0.001;
  public double Trans_kI = 1e-6;
  public double Trans_kD = 0;
  public double Trans_kIz = 0; 
  public double Trans_kFF = 0; 
  public double Trans_kMaxOutput = 1; 
  public double Trans_kMinOutput = -1;
  public double Trans_maxRPM = 11000;

  // Translation Motor Smart Motion Coefficients
  public double Trans_maxVel = Trans_maxRPM; // rpm
  public double Trans_minVel = 0;
  public double Trans_maxAcc = 11000;
  public double Trans_allowedErr = 0.5; 

  private final Timer m_timer = new Timer();

  private final double MAX_LINEAR_VELOCITY = 0.23;
  private final double MAX_ANGULAR_VELOCITY = 0.1;

  // encoder offset variables, should have two arrays of length 4: 
  // alpha to store abs offsets, delta to store rel offsets, alpha be final 
  public final double[] alpha_Motor = new double[]{14.589844, 51.064453, 117.949219, 124.365234};
  public double[] delta_Motor = new double[]{0.0, 0.0, 0.0, 0.0};

  // Vision
  private Thread visionThread;
  private final int imgWidth = 640;
  private final int imgHeight = 480;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // initialize vision
    initVision();

    // initialize PID
    initPID();
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
  * Returns modified desired to be within 180 degrees of current.
  * desired_rel is a relative encoder destination that could be modified.
  * current_rel is the current encoder position in relative degrees.
  * Both of the above can be arbitrary since encoders count up or down continuously.
  */
  private double[] wrapWheelCommand(double desired_rel, double current_rel){

    // diff is always in [-180, 180]
    double diff = wrapEncoderValues( desired_rel - current_rel );

    double[] res = new double[]{0.0, 0.0};
    if (diff > 90.0) {

      res[0] = current_rel + diff - 180.0;
      res[1] = -1.0;

    } else if (diff < -90.0) {

      res[0] = current_rel + diff + 180.0;
      res[1] = -1.0;

    } else {

      res[0] = current_rel + diff;
      res[1] = 1.0;
    }

    return res;
  }

  /*
   * Wrapper functions to more easily output values to SmartDashboard
   * cleaner code
   */
  private void printDB(String name, double val){ // print 1 value
    SmartDashboard.putNumber(name, val);
  }
  private void printDB(String name, boolean bool){ // print 1 value
    SmartDashboard.putBoolean(name, bool);
  }
  private void printDB(String name, String s){ // print 1 value
    SmartDashboard.putString(name, s);
  }
  // private void printDB(String[] names, double[] vals){ // print a list of values
  //   for(int i = 0; i < names.size(); i++){
  //     printDB(names[i], vals[i]);
  //   }
  // }

  /*
   * Initializes PID constants to all motor controllers
   */
  private void initPID(boolean verbose){

    // PID controllers
    RM2_PidController = RotationMotors[WHEEL_FL].getPIDController();
    RM2_Encoder = RotationMotors[WHEEL_FL].getEncoder();
    RM2_Encoder.setPositionConversionFactor(rotConvFactor);

    RM4_PidController = RotationMotors[WHEEL_FR].getPIDController();
    RM4_Encoder = RotationMotors[WHEEL_FR].getEncoder();
    RM4_Encoder.setPositionConversionFactor(rotConvFactor);

    RM6_PidController = RotationMotors[WHEEL_BR].getPIDController();
    RM6_Encoder = RotationMotors[WHEEL_BR].getEncoder();
    RM6_Encoder.setPositionConversionFactor(rotConvFactor);

    RM8_PidController = RotationMotors[WHEEL_BL].getPIDController();
    RM8_Encoder = RotationMotors[WHEEL_BL].getEncoder();
    RM8_Encoder.setPositionConversionFactor(rotConvFactor);

    TM1_PidController = TranslationMotors[WHEEL_FL].getPIDController();
    TM1_Encoder = TranslationMotors[WHEEL_FL].getEncoder();
    TM1_Encoder.setPositionConversionFactor(transConvFactor);

    TM3_PidController = TranslationMotors[WHEEL_FR].getPIDController();
    TM3_Encoder = TranslationMotors[WHEEL_FR].getEncoder();
    TM3_Encoder.setPositionConversionFactor(transConvFactor);
    
    TM5_PidController = TranslationMotors[WHEEL_BR].getPIDController();
    TM5_Encoder = TranslationMotors[WHEEL_BR].getEncoder();
    TM5_Encoder.setPositionConversionFactor(transConvFactor);
    
    TM7_PidController = TranslationMotors[WHEEL_BL].getPIDController();
    TM7_Encoder = TranslationMotors[WHEEL_BL].getEncoder();
    TM7_Encoder.setPositionConversionFactor(transConvFactor);

    RM2_PidController.setP(Rot_kP);
    RM2_PidController.setI(Rot_kI);
    RM2_PidController.setD(Rot_kD);
    RM2_PidController.setIZone(Rot_kIz);
    RM2_PidController.setFF(Rot_kFF);
    RM2_PidController.setOutputRange(Rot_kMinOutput, Rot_kMaxOutput);

    RM4_PidController.setP(Rot_kP);
    RM4_PidController.setI(Rot_kI);
    RM4_PidController.setD(Rot_kD);
    RM4_PidController.setIZone(Rot_kIz);
    RM4_PidController.setFF(Rot_kFF);
    RM4_PidController.setOutputRange(Rot_kMinOutput, Rot_kMaxOutput);

    RM6_PidController.setP(Rot_kP);
    RM6_PidController.setI(Rot_kI);
    RM6_PidController.setD(Rot_kD);
    RM6_PidController.setIZone(Rot_kIz);
    RM6_PidController.setFF(Rot_kFF);
    RM6_PidController.setOutputRange(Rot_kMinOutput, Rot_kMaxOutput);

    RM8_PidController.setP(Rot_kP);
    RM8_PidController.setI(Rot_kI);
    RM8_PidController.setD(Rot_kD);
    RM8_PidController.setIZone(Rot_kIz);
    RM8_PidController.setFF(Rot_kFF);
    RM8_PidController.setOutputRange(Rot_kMinOutput, Rot_kMaxOutput);

    TM1_PidController.setP(Trans_kP);
    TM1_PidController.setI(Trans_kI);
    TM1_PidController.setD(Trans_kD);
    TM1_PidController.setIZone(Trans_kIz);
    TM1_PidController.setFF(Trans_kFF);
    TM1_PidController.setOutputRange(Trans_kMinOutput, Trans_kMaxOutput);

    TM3_PidController.setP(Trans_kP);
    TM3_PidController.setI(Trans_kI);
    TM3_PidController.setD(Trans_kD);
    TM3_PidController.setIZone(Trans_kIz);
    TM3_PidController.setFF(Trans_kFF);
    TM3_PidController.setOutputRange(Trans_kMinOutput, Trans_kMaxOutput);

    TM5_PidController.setP(Trans_kP);
    TM5_PidController.setI(Trans_kI);
    TM5_PidController.setD(Trans_kD);
    TM5_PidController.setIZone(Trans_kIz);
    TM5_PidController.setFF(Trans_kFF);
    TM5_PidController.setOutputRange(Trans_kMinOutput, Trans_kMaxOutput);

    TM7_PidController.setP(Trans_kP);
    TM7_PidController.setI(Trans_kI);
    TM7_PidController.setD(Trans_kD);
    TM7_PidController.setIZone(Trans_kIz);
    TM7_PidController.setFF(Trans_kFF);
    TM7_PidController.setOutputRange(Trans_kMinOutput, Trans_kMaxOutput);

    int smartMotionSlot = 0;
    RM2_PidController.setSmartMotionMaxVelocity(Rot_maxVel, smartMotionSlot);
    RM2_PidController.setSmartMotionMinOutputVelocity(Rot_minVel, smartMotionSlot);
    RM2_PidController.setSmartMotionMaxAccel(Rot_maxAcc, smartMotionSlot);
    RM2_PidController.setSmartMotionAllowedClosedLoopError(Rot_allowedErr, smartMotionSlot);

    RM4_PidController.setSmartMotionMaxVelocity(Rot_maxVel, smartMotionSlot);
    RM4_PidController.setSmartMotionMinOutputVelocity(Rot_minVel, smartMotionSlot);
    RM4_PidController.setSmartMotionMaxAccel(Rot_maxAcc, smartMotionSlot);
    RM4_PidController.setSmartMotionAllowedClosedLoopError(Rot_allowedErr, smartMotionSlot);

    RM6_PidController.setSmartMotionMaxVelocity(Rot_maxVel, smartMotionSlot);
    RM6_PidController.setSmartMotionMinOutputVelocity(Rot_minVel, smartMotionSlot);
    RM6_PidController.setSmartMotionMaxAccel(Rot_maxAcc, smartMotionSlot);
    RM6_PidController.setSmartMotionAllowedClosedLoopError(Rot_allowedErr, smartMotionSlot);

    RM8_PidController.setSmartMotionMaxVelocity(Rot_maxVel, smartMotionSlot);
    RM8_PidController.setSmartMotionMinOutputVelocity(Rot_minVel, smartMotionSlot);
    RM8_PidController.setSmartMotionMaxAccel(Rot_maxAcc, smartMotionSlot);
    RM8_PidController.setSmartMotionAllowedClosedLoopError(Rot_allowedErr, smartMotionSlot);

    TranslationMotors[WHEEL_FL].setSmartCurrentLimit(40);
    TranslationMotors[WHEEL_FR].setSmartCurrentLimit(40);
    TranslationMotors[WHEEL_BR].setSmartCurrentLimit(40);
    TranslationMotors[WHEEL_BL].setSmartCurrentLimit(40);

    RotationMotors[WHEEL_FL].setSmartCurrentLimit(20);
    RotationMotors[WHEEL_FR].setSmartCurrentLimit(20);
    RotationMotors[WHEEL_BR].setSmartCurrentLimit(20);
    RotationMotors[WHEEL_BL].setSmartCurrentLimit(20);

    if(verbose){
      // display PID coefficients on SmartDashboard
      // SmartDashboard.putNumber("P Gain", kP);
      // SmartDashboard.putNumber("I Gain", kI);
      // SmartDashboard.putNumber("D Gain", kD);
      // SmartDashboard.putNumber("I Zone", kIz);
      // SmartDashboard.putNumber("Feed Forward", kFF);
      // SmartDashboard.putNumber("Max Output", kMaxOutput);
      // SmartDashboard.putNumber("Min Output", kMinOutput);

      // // display Smart Motion coefficients
      // SmartDashboard.putNumber("Max Velocity", maxVel);
      // SmartDashboard.putNumber("Min Velocity", minVel);
      // SmartDashboard.putNumber("Max Acceleration", maxAcc);
      // SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
      // SmartDashboard.putNumber("Set Position", 0);
      // SmartDashboard.putNumber("Set Velocity", 0);

      // button to toggle between velocity and smart motion modes
      SmartDashboard.putBoolean("Mode", true);
    }

    delta_Motor[0] = wrapEncoderValues((-1*coders[WHEEL_FL].getAbsolutePosition()) - RM2_Encoder.getPosition());
    delta_Motor[1] = wrapEncoderValues((-1*coders[WHEEL_FR].getAbsolutePosition()) - RM4_Encoder.getPosition());
    delta_Motor[2] = wrapEncoderValues((-1*coders[WHEEL_BR].getAbsolutePosition()) - RM6_Encoder.getPosition());
    delta_Motor[3] = wrapEncoderValues((-1*coders[WHEEL_BL].getAbsolutePosition()) - RM8_Encoder.getPosition());
  }

  private void initPID(){
     // init PID without outputting to dashboard
    initPID(false);
  }

  /*
   * Initializes Computer Vision
   */
  private void initVision(){

    visionThread = new Thread(() -> {
      // add USB camera, create server for SmartDashboard
      UsbCamera usbCamera = CameraServer.startAutomaticCapture("Main Camera", 0);
      // UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
      usbCamera.setResolution(imgWidth, imgHeight);

      CvSink cvSink = CameraServer.getVideo(); // grab images from camera
      CvSource outputStream = CameraServer.putVideo("Processed Image", imgWidth, imgHeight);

      Point upleft = new Point(50, 50);
      Point downright = new Point(100, 100);
      Scalar color = new Scalar(255, 255, 255);
      Mat sourceMat = new Mat();
      Mat mask = new Mat();
      Mat mask2 = new Mat();

      // For HSV, hue range is [0,179]
      // saturation range is [0,255]\// value range is [0,255]
      Scalar redLower = new Scalar(155, 25, 0);
      Scalar redUpper = new Scalar(180, 255, 255);

      // 15 - 330 not red
      Scalar redLower2 = new Scalar(0, 50, 50);
      Scalar redUpper2 = new Scalar(15, 255, 255);
      
      while(true){ /// TODO: change condition later
        if (cvSink.grabFrame(sourceMat) == 0) {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          // skip the rest of the current iteration
          continue; 
        }
        Scalar avg = Core.mean(sourceMat);
        // debug statements
        //System.out.println(avg);
        //Imgproc.rectangle(sourceMat, upleft, downright, avg, -1, 8, 0); // draw a rectangle
        Imgproc.cvtColor(sourceMat, sourceMat, Imgproc.COLOR_BGR2HSV); //convert to graysc ale
        Core.inRange(sourceMat, redLower, redUpper, mask);
        Core.inRange(sourceMat, redLower2, redUpper2, mask2);
        Core.bitwise_or(mask, mask2, mask);
        // Core.bitwise_and(sourceMat, sourceMat, sourceMat, mask);
        // Core.bitwise_and(sourceMat, sourceMat, sourceMat, mask);
        //  480*640*CV_8UC3

        outputStream.putFrame(mask); // put processed image to smartdashboard
      }

    });

    visionThread.setDaemon(true); // set as daemon thread (low priority)
    visionThread.start(); // start vision thread
  }
  // ------------------------------------------------------------------------------

  private void set_desired(double[] desired_body, double[] DESIRED, double[] current_rel, double[] desired_rel1, double[] desired_translation, int CASE) {

    if (CASE == 1) {
      desired_body[WHEEL_FL] = DESIRED[WHEEL_FL];
      desired_body[WHEEL_FR] = DESIRED[WHEEL_FR];
      desired_body[WHEEL_BR] = DESIRED[WHEEL_BR];
      desired_body[WHEEL_BL] = DESIRED[WHEEL_BL];

      double[] desired_abs = new double[]{
        wrapEncoderValues(desired_body[WHEEL_FL] - alpha_Motor[WHEEL_FL]),
        wrapEncoderValues(desired_body[WHEEL_FR] - alpha_Motor[WHEEL_FR]),
        wrapEncoderValues(desired_body[WHEEL_BR] - alpha_Motor[WHEEL_BR]),
        wrapEncoderValues(desired_body[WHEEL_BL] - alpha_Motor[WHEEL_BL])
      };
  
      double[] desired_rel = new double[]{
        -(desired_abs[WHEEL_FL] + delta_Motor[WHEEL_FL]),
        -(desired_abs[WHEEL_FR] + delta_Motor[WHEEL_FR]),
        -(desired_abs[WHEEL_BR] + delta_Motor[WHEEL_BR]),
        -(desired_abs[WHEEL_BL] + delta_Motor[WHEEL_BL])
      };
  
      double[] res1 = wrapWheelCommand(desired_rel[WHEEL_FL], current_rel[WHEEL_FL]);
      double[] res2 = wrapWheelCommand(desired_rel[WHEEL_FR], current_rel[WHEEL_FR]);
      double[] res3 = wrapWheelCommand(desired_rel[WHEEL_BR], current_rel[WHEEL_BR]);
      double[] res4 = wrapWheelCommand(desired_rel[WHEEL_BL], current_rel[WHEEL_BL]);
  
      desired_rel1[WHEEL_FL] = res1[0];
      desired_rel1[WHEEL_FR] = res2[0];
      desired_rel1[WHEEL_BR] = res3[0];
      desired_rel1[WHEEL_BL] = res4[0];
      }

      if (CASE == 2) {
        desired_body[WHEEL_FL] = DESIRED[WHEEL_FL];
        desired_body[WHEEL_FR] = DESIRED[WHEEL_FR];
        desired_body[WHEEL_BR] = DESIRED[WHEEL_BR];
        desired_body[WHEEL_BL] = DESIRED[WHEEL_BL];

        double[] desired_abs = new double[]{
          wrapEncoderValues(desired_body[WHEEL_FL] - alpha_Motor[WHEEL_FL]),
          wrapEncoderValues(desired_body[WHEEL_FR] - alpha_Motor[WHEEL_FR]),
          wrapEncoderValues(desired_body[WHEEL_BR] - alpha_Motor[WHEEL_BR]),
          wrapEncoderValues(desired_body[WHEEL_BL] - alpha_Motor[WHEEL_BL])
        };
    
        double[] desired_rel = new double[]{
          -(desired_abs[WHEEL_FL] + delta_Motor[WHEEL_FL]),
          -(desired_abs[WHEEL_FR] + delta_Motor[WHEEL_FR]),
          -(desired_abs[WHEEL_BR] + delta_Motor[WHEEL_BR]),
          -(desired_abs[WHEEL_BL] + delta_Motor[WHEEL_BL])
        };
    
        double[] res1 = wrapWheelCommand(desired_rel[WHEEL_FL], current_rel[WHEEL_FL]);
        double[] res2 = wrapWheelCommand(desired_rel[WHEEL_FR], current_rel[WHEEL_FR]);
        double[] res3 = wrapWheelCommand(desired_rel[WHEEL_BR], current_rel[WHEEL_BR]);
        double[] res4 = wrapWheelCommand(desired_rel[WHEEL_BL], current_rel[WHEEL_BL]);
    
        desired_rel1[WHEEL_FL] = res1[0];
        desired_rel1[WHEEL_FR] = res2[0];
        desired_rel1[WHEEL_BR] = res3[0];
        desired_rel1[WHEEL_BL] = res4[0];

        desired_translation[WHEEL_FL] = res1[1];
        desired_translation[WHEEL_FR] = res2[1];
        desired_translation[WHEEL_BR] = res3[1];
        desired_translation[WHEEL_BL] = res4[1];

      }

      else if (CASE == 3) {

        desired_body[WHEEL_FL] = -1*DESIRED[WHEEL_FL];
        desired_body[WHEEL_FR] = -1*DESIRED[WHEEL_FR];
        desired_body[WHEEL_BR] = -1*DESIRED[WHEEL_BR];
        desired_body[WHEEL_BL] = -1*DESIRED[WHEEL_BL];

        double[] desired_abs = new double[]{
          wrapEncoderValues(desired_body[WHEEL_FL] - alpha_Motor[WHEEL_FL]),
          wrapEncoderValues(desired_body[WHEEL_FR] + 90 - alpha_Motor[WHEEL_FR]),
          wrapEncoderValues(desired_body[WHEEL_BR] + 180 - alpha_Motor[WHEEL_BR]),
          wrapEncoderValues(desired_body[WHEEL_BL] + 270 - alpha_Motor[WHEEL_BL])
        };
    
        double[] desired_rel = new double[]{
          -(desired_abs[WHEEL_FL] + delta_Motor[WHEEL_FL]),
          -(desired_abs[WHEEL_FR] + delta_Motor[WHEEL_FR]),
          -(desired_abs[WHEEL_BR] + delta_Motor[WHEEL_BR]),
          -(desired_abs[WHEEL_BL] + delta_Motor[WHEEL_BL])
        };
    
        double[] res1 = wrapWheelCommand(desired_rel[WHEEL_FL], current_rel[WHEEL_FL]);
        double[] res2 = wrapWheelCommand(desired_rel[WHEEL_FR], current_rel[WHEEL_FR]);
        double[] res3 = wrapWheelCommand(desired_rel[WHEEL_BR], current_rel[WHEEL_BR]);
        double[] res4 = wrapWheelCommand(desired_rel[WHEEL_BL], current_rel[WHEEL_BL]);
    
        desired_rel1[WHEEL_FL] = res1[0];
        desired_rel1[WHEEL_FR] = res2[0];
        desired_rel1[WHEEL_BR] = res3[0];
        desired_rel1[WHEEL_BL] = res4[0];

        desired_translation[WHEEL_FL] = res1[1];
        desired_translation[WHEEL_FR] = res2[1];
        desired_translation[WHEEL_BR] = res3[1];
        desired_translation[WHEEL_BL] = res4[1];
      }

  }
  
  
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    double[]current_abs = new double[]{
      coders[0].getAbsolutePosition(),
      coders[1].getAbsolutePosition(),
      coders[2].getAbsolutePosition(),
      coders[3].getAbsolutePosition()
    };
    double[]current_rel = new double[]{
      RM2_Encoder.getPosition(),
      RM4_Encoder.getPosition(),
      RM6_Encoder.getPosition(),
      RM8_Encoder.getPosition()
    };

    SmartDashboard.putNumber("CANCODER 1", current_abs[WHEEL_FL]);
    SmartDashboard.putNumber("CANCODER 2", current_abs[WHEEL_FR]);
    SmartDashboard.putNumber("CANCODER 3", current_abs[WHEEL_BR]);
    SmartDashboard.putNumber("CANCODER 4", current_abs[WHEEL_BL]);

    SmartDashboard.putNumber("RM2_Encoder.getPosition()", current_rel[WHEEL_FL]); 
    // SmartDashboard.putNumber("hopefully close to CANCODER 1", wrapEncoderValues(-current_rel[0] - delta_Motor[0]));

    SmartDashboard.putNumber("RM4_Encoder.getPosition()", current_rel[WHEEL_FR]); 
    // SmartDashboard.putNumber("hopefully close to CANCODER 2", wrapEncoderValues(-current_rel[1] - delta_Motor[1]));

    SmartDashboard.putNumber("RM6_Encoder.getPosition()", current_rel[WHEEL_BR]); 
    // SmartDashboard.putNumber("hopefully close to CANCODER 3", wrapEncoderValues(-current_rel[2] - delta_Motor[2]));

    SmartDashboard.putNumber("RM8_Encoder.getPosition()", current_rel[WHEEL_BL]); 
    // SmartDashboard.putNumber("hopefully close to CANCODER 4", wrapEncoderValues(-current_rel[3] - delta_Motor[3]));

    // Obtain filtered Joystick Inputs
    filteredJoystickLeftY = filterJoystick(PS4joystick.getLeftY(), true);
    filteredJoystickLeftX = filterJoystick(PS4joystick.getLeftX(), true);
    filteredJoystickRightX = filterJoystick(PS4joystick.getRightX(), false);

    double Vx = (-1) * highpassFilter(PS4joystick.getLeftY());
    SmartDashboard.putNumber("Vx",  Vx);
    double Vy = (-1) * highpassFilter(PS4joystick.getLeftX());
    SmartDashboard.putNumber("Vy", Vy);
    double omega = (-1) * highpassFilter(PS4joystick.getRightX());

    SmartDashboard.putNumber("w (omega)", omega);

    double Vr = mag(Vx, Vy);
    SmartDashboard.putNumber("Vr", Vr);

    double[] desired_body = new double[]{0.0, 0.0, 0.0, 0.0};
    double[] desired_rel1 = new double[]{0.0, 0.0, 0.0, 0.0};
    double[] desired_translation = new double[]{0.0, 0.0, 0.0, 0.0};

    // Constants
    double x = 12.125;
    double y = 12.125;
    double d;
    double Vr_norm[] = new double[]{Vx/Vr, Vy/Vr};

    if (omega == 0 & Vx != 0 & Vy != 0) {
      // No rotation, CASE 1
      SmartDashboard.putNumber("Case", 1);
      // double[] DESIRED = new double[]{((Math.atan2(Vy,Vx) * 180) / Math.PI),((Math.atan2(Vy,Vx) * 180) / Math.PI) , ((Math.atan2(Vy,Vx) * 180) / Math.PI), ((Math.atan2(Vy,Vx) * 180) / Math.PI)};
      // SmartDashboard.putNumber("DESIRED_BEFORE", DESIRED);

      // set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);
      // RM2_PidController.setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      // RM4_PidController.setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      // RM6_PidController.setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      // RM8_PidController.setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);

      // TM1_PidController.setReference(desired_translation[WHEEL_FL], CANSparkMax.ControlType.kVelocity);
      // RM4_PidController.setReference(desired_translation[WHEEL_FR], CANSparkMax.ControlType.kVelocity);
      // RM6_PidController.setReference(desired_translation[WHEEL_BR], CANSparkMax.ControlType.kVelocity);
      // RM8_PidController.setReference(desired_translation[WHEEL_BL], CANSparkMax.ControlType.kVelocity);
    }

    else if (omega != 0 & Vx != 0 & Vy != 0) {
      // General Case, CASE 2
      SmartDashboard.putNumber("Case", 2);

      // d = Vr/omega;
      // SmartDashboard.putNumber("d", d);

      // double ICC[] = new double[]{d*(-1)*Vr_norm[1], d*Vr_norm[0]};
      // SmartDashboard.putNumber("ICC[0]", ICC[0]);
      // SmartDashboard.putNumber("ICC[1]", ICC[1]);

      // double Vwheel1_xy[] = new double[]{ICC[0] - x, ICC[1] - y};
      // double Vwheel2_xy[] = new double[]{ICC[0] - x, ICC[1] + y};
      // double Vwheel3_xy[] = new double[]{ICC[0] + x, ICC[1] + y};
      // double Vwheel4_xy[] = new double[]{ICC[0] + x, ICC[1] - y};
      // double Vwheel1 = mag(Vwheel1_xy[0], Vwheel1_xy[1]) * omega;
      // double Vwheel2 = mag(Vwheel2_xy[0], Vwheel2_xy[1]) * omega;
      // double Vwheel3 = mag(Vwheel3_xy[0], Vwheel3_xy[1]) * omega;
      // double Vwheel4 = mag(Vwheel4_xy[0], Vwheel4_xy[1]) * omega;
  
      // SmartDashboard.putNumber("Vwheel1", Vwheel1);
      // SmartDashboard.putNumber("Vwheel2", Vwheel2);
      // SmartDashboard.putNumber("Vwheel3", Vwheel3);
      // SmartDashboard.putNumber("Vwheel4", Vwheel4);

      // double Omega_wheel1 = Math.atan2((-1)*Vwheel1_xy[0], Vwheel1_xy[1]);
      // double Omega_wheel2 = Math.atan2((-1)*Vwheel2_xy[0], Vwheel2_xy[1]);
      // double Omega_wheel3 = Math.atan2((-1)*Vwheel3_xy[0], Vwheel3_xy[1]);
      // double Omega_wheel4 = Math.atan2((-1)*Vwheel4_xy[0], Vwheel4_xy[1]);

      // SmartDashboard.putNumber("Omega_wheel1", Omega_wheel1);
      // SmartDashboard.putNumber("Omega_wheel2", Omega_wheel2);
      // SmartDashboard.putNumber("Omega_wheel3", Omega_wheel3);
      // SmartDashboard.putNumber("Omega_wheel4", Omega_wheel4);

      // double[] DESIRED = new double[] {Omega_wheel1, Omega_wheel2, Omega_wheel3, Omega_wheel4};

      // set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 2);
      // RM2_PidController.setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      // RM4_PidController.setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      // RM6_PidController.setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      // RM8_PidController.setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);

      // TM1_PidController.setReference(desired_translation[WHEEL_FL] * Vwheel1 * Trans_maxRPM * MAX_LINEAR_VELOCITY, CANSparkMax.ControlType.kVelocity);
      // TM3_PidController.setReference(desired_translation[WHEEL_FR] * Vwheel2 * Trans_maxRPM * MAX_LINEAR_VELOCITY, CANSparkMax.ControlType.kVelocity);
      // TM5_PidController.setReference(desired_translation[WHEEL_BR] * Vwheel3 * Trans_maxRPM * MAX_LINEAR_VELOCITY, CANSparkMax.ControlType.kVelocity);
      // TM7_PidController.setReference(desired_translation[WHEEL_BL] * Vwheel4 * Trans_maxRPM * MAX_LINEAR_VELOCITY, CANSparkMax.ControlType.kVelocity);
    }

    else if (Vx == 0.0 & Vy == 0.0 & omega != 0) {
      // Rotate in place, CASE 3
      SmartDashboard.putNumber("Case", 3);
      double[] DESIRED = new double[] {45, 45, 45, 45};

      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 3);
      // RM2_PidController.setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      RM4_PidController.setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      // RM6_PidController.setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      RM8_PidController.setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);

      // SmartDashboard.putNumber("translateMotor1", omega);
      // SmartDashboard.putNumber("translateMotor2", omega);
      // SmartDashboard.putNumber("translateMotor3", omega);
      // SmartDashboard.putNumber("translateMotor4", omega);

      // TM1_PidController.setReference(desired_translation[WHEEL_FL] * omega * Trans_maxRPM, CANSparkMax.ControlType.kVelocity);
      // TM3_PidController.setReference(desired_translation[WHEEL_FR] * omega * Trans_maxRPM, CANSparkMax.ControlType.kVelocity);
      // TM5_PidController.setReference(desired_translation[WHEEL_BR] * omega * Trans_maxRPM, CANSparkMax.ControlType.kVelocity);
      // TM7_PidController.setReference(desired_translation[WHEEL_BL] * omega * Trans_maxRPM, CANSparkMax.ControlType.kVelocity);

  }
  
}

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}