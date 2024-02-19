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

   // for example 
  private CANCoder[] coders = new CANCoder[] {
    new CANCoder (1),
    new CANCoder(2),
    new CANCoder(3),
    new CANCoder (4),
  };
  // all of this gets stuffed into arrays
  
  // wheel motors
  private CANSparkMax translateMotor1 = new CANSparkMax(1, MotorType.kBrushless); // translation - odds
  private CANSparkMax rotateMotor2 = new CANSparkMax(2, MotorType.kBrushless); // rotation - evens
  private CANSparkMax translateMotor3 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rotateMotor4 = new CANSparkMax(4, MotorType.kBrushless); 
  private CANSparkMax translateMotor5 = new CANSparkMax(5, MotorType.kBrushless); 
  private CANSparkMax rotateMotor6 = new CANSparkMax(6, MotorType.kBrushless);
  private CANSparkMax translateMotor7 = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax rotateMotor8 = new CANSparkMax(8, MotorType.kBrushless); 
  
  // wheel encoders
  // private CANCoder coder1 = new CANCoder(1);
  // private CANCoder coder2 = new CANCoder(2);
  // private CANCoder coder3 = new CANCoder(3);
  // private CANCoder coder4 = new CANCoder(4);

  private SparkMaxPIDController RM2_PidController;
  private SparkMaxPIDController RM4_PidController;
  private SparkMaxPIDController RM6_PidController;
  private SparkMaxPIDController RM8_PidController;

  private SparkMaxPIDController TM1_PidController;
  private SparkMaxPIDController TM3_PidController;
  private SparkMaxPIDController TM5_PidController;
  private SparkMaxPIDController TM7_PidController;

  private RelativeEncoder RM2_Encoder;
  private RelativeEncoder RM4_Encoder;
  private RelativeEncoder RM6_Encoder;
  private RelativeEncoder RM8_Encoder;
  private double convFactor = 5.62279270244;
  // private double convFactor = 1;

  // joystick controller
  private final PS4Controller PS4joystick = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station
  private double filteredJoystickLeftY;
  private double filteredJoystickLeftX;
  // private double filteredJoystickRightY;
  private double filteredJoystickRightX;

  // PID Constants
  public double kP = 0.0001;
  // public double kI = 1e-9;
  public double kI = 0;
  public double kD = 0.0003;
  // public double kD = 0;
  public double kIz = 0; 
  public double kFF = 0; 
  public double kMaxOutput = 1; 
  public double kMinOutput = -1;
  public double maxRPM = 11000;

  // Smart Motion Coefficients
  public double maxVel = maxRPM; // rpm
  public double minVel = 0;
  public double maxAcc = 30000;
  public double allowedErr = 0.5; // TODO: placeholder, not correct value

  private final Timer m_timer = new Timer();

  // encoder offset variables
  private final double MAX_LINEAR_VELOCITY = 0.23;
  private final double MAX_ANGULAR_VELOCITY = 0.1;
  
  // public final double[] offset1 = new double[]{14.589844, -165.585938};
  // public final double[] offset2 = new double[]{51.064453, -130.253906};
  // public final double[] offset3 = new double[]{117.949219, -60.292969};
  // public final double[] offset4 = new double[]{117.949219, -60.292969};

  // public final double[] pinkoffset1 = new double[]{0,0};
  // public final double[] pinkoffset2 = new double[]{0,0};
  // public final double[] pinkoffset3 = new double[]{-90.500122, 89.499878};
  // public final double[] pinkoffset4 = new double[]{0,0};

  /* should have two arrays of length 4: alpha to store abs offsets, delta to store rel offsets, alpha be final */
  public double[] alpha_Motor = new double[]{14.589844, 51.064453, 117.949219, 117.949219};
  public double[] delta_Motor = new double[]{0.0, 0.0, 0.0, 0.0};
  //public double delta_Motor3 = 0.0;


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
    RM2_PidController = rotateMotor2.getPIDController();
    RM2_Encoder = rotateMotor2.getEncoder();
    RM2_Encoder.setPositionConversionFactor(convFactor);

    RM4_PidController = rotateMotor4.getPIDController();
    RM4_Encoder = rotateMotor4.getEncoder();
    RM4_Encoder.setPositionConversionFactor(convFactor);

    RM6_PidController = rotateMotor6.getPIDController();
    RM6_Encoder = rotateMotor6.getEncoder();
    RM6_Encoder.setPositionConversionFactor(convFactor);

    RM8_PidController = rotateMotor8.getPIDController();
    RM8_Encoder = rotateMotor8.getEncoder();
    RM8_Encoder.setPositionConversionFactor(convFactor);

    TM1_PidController = translateMotor1.getPIDController();
    TM3_PidController = translateMotor3.getPIDController();
    TM5_PidController = translateMotor5.getPIDController();
    TM7_PidController = translateMotor7.getPIDController();

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

    TM1_PidController.setP(kP);
    TM1_PidController.setI(kI);
    TM1_PidController.setD(kD);
    TM1_PidController.setIZone(kIz);
    TM1_PidController.setFF(kFF);
    TM1_PidController.setOutputRange(kMinOutput, kMaxOutput);

    TM3_PidController.setP(kP);
    TM3_PidController.setI(kI);
    TM3_PidController.setD(kD);
    TM3_PidController.setIZone(kIz);
    TM3_PidController.setFF(kFF);
    TM3_PidController.setOutputRange(kMinOutput, kMaxOutput);

    TM5_PidController.setP(kP);
    TM5_PidController.setI(kI);
    TM5_PidController.setD(kD);
    TM5_PidController.setIZone(kIz);
    TM5_PidController.setFF(kFF);
    TM5_PidController.setOutputRange(kMinOutput, kMaxOutput);

    TM7_PidController.setP(kP);
    TM7_PidController.setI(kI);
    TM7_PidController.setD(kD);
    TM7_PidController.setIZone(kIz);
    TM7_PidController.setFF(kFF);
    TM7_PidController.setOutputRange(kMinOutput, kMaxOutput);

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

    translateMotor1.setSmartCurrentLimit(40);
    translateMotor3.setSmartCurrentLimit(40);
    translateMotor5.setSmartCurrentLimit(40);
    translateMotor7.setSmartCurrentLimit(40);

    rotateMotor2.setSmartCurrentLimit(20);
    rotateMotor4.setSmartCurrentLimit(20);
    rotateMotor6.setSmartCurrentLimit(20);
    rotateMotor8.setSmartCurrentLimit(20);

    if(verbose){
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);

      // display Smart Motion coefficients
      SmartDashboard.putNumber("Max Velocity", maxVel);
      SmartDashboard.putNumber("Min Velocity", minVel);
      SmartDashboard.putNumber("Max Acceleration", maxAcc);
      SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
      SmartDashboard.putNumber("Set Position", 0);
      SmartDashboard.putNumber("Set Velocity", 0);

      // button to toggle between velocity and smart motion modes
      SmartDashboard.putBoolean("Mode", true);
    }
    delta_Motor[0] = wrapEncoderValues((-1*coders[0].getAbsolutePosition()) - RM2_Encoder.getPosition());
    delta_Motor[1] = wrapEncoderValues((-1*coders[1].getAbsolutePosition()) - RM4_Encoder.getPosition());
    delta_Motor[2] = wrapEncoderValues((-1*coders[2].getAbsolutePosition()) - RM6_Encoder.getPosition());
    delta_Motor[3] = wrapEncoderValues((-1*coders[3].getAbsolutePosition()) - RM8_Encoder.getPosition());
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
      desired_body[0] = DESIRED[0];
      desired_body[1] = DESIRED[1];
      desired_body[2] = DESIRED[2];
      desired_body[3] = DESIRED[3];

      double[] desired_abs = new double[]{
        wrapEncoderValues(desired_body[0] - alpha_Motor[0]),
        wrapEncoderValues(desired_body[1] - alpha_Motor[1]),
        wrapEncoderValues(desired_body[2] - alpha_Motor[2]),
        wrapEncoderValues(desired_body[3] - alpha_Motor[3])
      };
  
      double[] desired_rel = new double[]{
        -(desired_abs[0] + delta_Motor[0]),
        -(desired_abs[1] + delta_Motor[1]),
        -(desired_abs[2] + delta_Motor[2]),
        -(desired_abs[3] + delta_Motor[3])
      };
  
      double[] res1 = wrapWheelCommand(desired_rel[0], current_rel[0]);
      double[] res2 = wrapWheelCommand(desired_rel[1], current_rel[1]);
      double[] res3 = wrapWheelCommand(desired_rel[2], current_rel[2]);
      double[] res4 = wrapWheelCommand(desired_rel[3], current_rel[3]);
  
      desired_rel1[0] = res1[0];
      desired_rel1[1] = res2[0];
      desired_rel1[2] = res3[0];
      desired_rel1[3] = res4[0];
      }

      if (CASE == 2) {
        desired_body[0] = DESIRED[0];
        desired_body[1] = DESIRED[1];
        desired_body[2] = DESIRED[2];
        desired_body[3] = DESIRED[3];

        double[] desired_abs = new double[]{
          wrapEncoderValues(desired_body[0] - alpha_Motor[0]),
          wrapEncoderValues(desired_body[1] - alpha_Motor[1]),
          wrapEncoderValues(desired_body[2] - alpha_Motor[2]),
          wrapEncoderValues(desired_body[3] - alpha_Motor[3])
        };
    
        double[] desired_rel = new double[]{
          -(desired_abs[0] + delta_Motor[0]),
          -(desired_abs[1] + delta_Motor[1]),
          -(desired_abs[2] + delta_Motor[2]),
          -(desired_abs[3] + delta_Motor[3])
        };
    
        double[] res1 = wrapWheelCommand(desired_rel[0], current_rel[0]);
        double[] res2 = wrapWheelCommand(desired_rel[1], current_rel[1]);
        double[] res3 = wrapWheelCommand(desired_rel[2], current_rel[2]);
        double[] res4 = wrapWheelCommand(desired_rel[3], current_rel[3]);
    
        desired_rel1[0] = res1[0];
        desired_rel1[1] = res2[0];
        desired_rel1[2] = res3[0];
        desired_rel1[3] = res4[0];

        desired_translation[0] = res1[1];
        desired_translation[1] = res2[1];
        desired_translation[2] = res3[1];
        desired_translation[3] = res4[1];

      }

      else if (CASE == 3) {

        desired_body[0] = -1*DESIRED[0];
        desired_body[1] = -1*DESIRED[1];
        desired_body[2] = -1*DESIRED[2];
        desired_body[3] = -1*DESIRED[3];

        double[] desired_abs = new double[]{
          wrapEncoderValues(desired_body[0] - alpha_Motor[0]),
          wrapEncoderValues(desired_body[1] + 90 - alpha_Motor[1]),
          wrapEncoderValues(desired_body[2] + 180 - alpha_Motor[2]),
          wrapEncoderValues(desired_body[3] + 270 - alpha_Motor[3])
        };
    
        double[] desired_rel = new double[]{
          -(desired_abs[0] + delta_Motor[0]),
          -(desired_abs[1] + delta_Motor[1]),
          -(desired_abs[2] + delta_Motor[2]),
          -(desired_abs[3] + delta_Motor[3])
        };
    
        double[] res1 = wrapWheelCommand(desired_rel[0], current_rel[0]);
        double[] res2 = wrapWheelCommand(desired_rel[1], current_rel[1]);
        double[] res3 = wrapWheelCommand(desired_rel[2], current_rel[2]);
        double[] res4 = wrapWheelCommand(desired_rel[3], current_rel[3]);
    
        desired_rel1[0] = res1[0];
        desired_rel1[1] = res2[0];
        desired_rel1[2] = res3[0];
        desired_rel1[3] = res4[0];

        desired_translation[0] = res1[1];
        desired_translation[1] = res2[1];
        desired_translation[2] = res3[1];
        desired_translation[3] = res4[1];
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

    SmartDashboard.putNumber("CANCODER 1", current_abs[0]);
    SmartDashboard.putNumber("CANCODER 2", current_abs[1]);
    SmartDashboard.putNumber("CANCODER 3", current_abs[2]);
    SmartDashboard.putNumber("CANCODER 4", current_abs[3]);

    SmartDashboard.putNumber("RM2_Encoder.getPosition()", current_rel[0]); 
    SmartDashboard.putNumber("hopefully close to CANCODER 1", wrapEncoderValues(-current_rel[0] - delta_Motor[0]));

    SmartDashboard.putNumber("RM4_Encoder.getPosition()", current_rel[1]); 
    SmartDashboard.putNumber("hopefully close to CANCODER 2", wrapEncoderValues(-current_rel[1] - delta_Motor[1]));

    SmartDashboard.putNumber("RM6_Encoder.getPosition()", current_rel[2]); 
    SmartDashboard.putNumber("hopefully close to CANCODER 3", wrapEncoderValues(-current_rel[2] - delta_Motor[2]));

    SmartDashboard.putNumber("RM8_Encoder.getPosition()", current_rel[3]); 
    SmartDashboard.putNumber("hopefully close to CANCODER 4", wrapEncoderValues(-current_rel[3] - delta_Motor[3]));

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
      double[] DESIRED = new double[]{((Math.atan2(Vy,Vx) * 180) / Math.PI),((Math.atan2(Vy,Vx) * 180) / Math.PI) , ((Math.atan2(Vy,Vx) * 180) / Math.PI), ((Math.atan2(Vy,Vx) * 180) / Math.PI)};
      // SmartDashboard.putNumber("DESIRED_BEFORE", DESIRED);

      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);
      // RM2_PidController.setReference(desired_rel1[0], CANSparkMax.ControlType.kSmartMotion);
      // RM4_PidController.setReference(desired_rel1[1], CANSparkMax.ControlType.kSmartMotion);
      // RM6_PidController.setReference(desired_rel1[2], CANSparkMax.ControlType.kSmartMotion);
      // RM8_PidController.setReference(desired_rel1[3], CANSparkMax.ControlType.kSmartMotion);
    }

    else if (Vx == 0.0 & Vy == 0.0 & omega != 0) {
      // Rotate in place, CASE 3
      SmartDashboard.putNumber("Case", 3);
      // double[] DESIRED = new double[] {45, 45, 45, 45};

      // set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 3);
      // RM2_PidController.setReference(desired_rel1[0], CANSparkMax.ControlType.kSmartMotion);
      // RM4_PidController.setReference(desired_rel1[1], CANSparkMax.ControlType.kSmartMotion);
      // RM6_PidController.setReference(desired_rel1[2], CANSparkMax.ControlType.kSmartMotion);
      // RM8_PidController.setReference(desired_rel1[3], CANSparkMax.ControlType.kSmartMotion);

      // SmartDashboard.putNumber("translateMotor1", omega);
      // SmartDashboard.putNumber("translateMotor2", omega);
      // SmartDashboard.putNumber("translateMotor3", omega);
      // SmartDashboard.putNumber("translateMotor4", omega);

      // TM1_PidController.setReference(desired_translation[0] * omega * maxRPM, CANSparkMax.ControlType.kVelocity);
      // TM3_PidController.setReference(desired_translation[1] * omega * maxRPM, CANSparkMax.ControlType.kVelocity);
      // TM5_PidController.setReference(desired_translation[2] * omega * maxRPM, CANSparkMax.ControlType.kVelocity);
      // TM7_PidController.setReference(desired_translation[3] * omega * maxRPM, CANSparkMax.ControlType.kVelocity);

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
      // RM2_PidController.setReference(desired_rel1[0], CANSparkMax.ControlType.kSmartMotion);
      // RM4_PidController.setReference(desired_rel1[1], CANSparkMax.ControlType.kSmartMotion);
      // RM6_PidController.setReference(desired_rel1[2], CANSparkMax.ControlType.kSmartMotion);
      // RM8_PidController.setReference(desired_rel1[3], CANSparkMax.ControlType.kSmartMotion);

      // TM1_PidController.setReference(desired_translation[0] * Vwheel1 * maxRPM * MAX_LINEAR_VELOCITY, CANSparkMax.ControlType.kVelocity);
      // TM3_PidController.setReference(desired_translation[1] * Vwheel2 * maxRPM * MAX_LINEAR_VELOCITY, CANSparkMax.ControlType.kVelocity);
      // TM5_PidController.setReference(desired_translation[2] * Vwheel3 * maxRPM * MAX_LINEAR_VELOCITY, CANSparkMax.ControlType.kVelocity);
      // TM7_PidController.setReference(desired_translation[3] * Vwheel4 * maxRPM * MAX_LINEAR_VELOCITY, CANSparkMax.ControlType.kVelocity);

    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}