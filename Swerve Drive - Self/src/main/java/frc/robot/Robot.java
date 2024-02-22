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
import org.opencv.core.Size;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import java.util.List;
import java.util.ArrayList;
import java.util.Random;

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

  // joystick controller
  private final PS4Controller PS4joystick = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station
  private double filteredJoystickLeftY;
  private double filteredJoystickLeftX;
  // private double filteredJoystickRightY;
  private double filteredJoystickRightX;

  // PID Constants
  public double kP = 5e-5;
  public double kI = 1e-8;
  public double kD = 0;
  public double kIz = 0; 
  public double kFF = 0; 
  public double kMaxOutput = 1; 
  public double kMinOutput = -1;
  public double maxRPM = 11000;

  // Smart Motion Coefficients
  public double maxVel = maxRPM; // rpm
  public double minVel = 0;
  public double maxAcc = 5000;
  public double allowedErr = 0; // TODO: placeholder, not correct value

  private final Timer m_timer = new Timer();

  // encoder offset variables
  private final double MAX_LINEAR_VELOCITY = 0.23;
  private final double MAX_ANGULAR_VELOCITY = 0.1;
  
  public final double[] offset1 = new double[]{14.589844, -165.585938};
  public final double[] offset2 = new double[]{51.064453, -130.253906};
  public final double[] offset3 = new double[]{117.949219, -60.292969};
  public final double[] offset4 = new double[]{117.949219, -60.292969};

  public final double[] pinkoffset1 = new double[]{0,0};
  public final double[] pinkoffset2 = new double[]{0,0};
  public final double[] pinkoffset3 = new double[]{-90.500122, 89.499878};
  public final double[] pinkoffset4 = new double[]{0,0};

  public double[] delta_Motor = new double[]{0.0, 0.0, 0.0, 0.0};
  //public double delta_Motor3 = 0.0;

  /* should have two arrays of length 4: alpha to store abs offsets, delta to store rel offsets, alpha be final */

  // Vision Variables
  private Thread visionThread;
  private final int imgWidth = 320; // 320
  private final int imgHeight = 240; // 240
  private Random rng = new Random(12345);
  private final int maxObjectColors = 5;
  private final boolean verbose = false;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // initialize vision
    initVision();

    // initialize PID
    // initPID();
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
  private void initPID(boolean verbose){

    // PID controllers
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
  }

  private void initPID(){
     // init PID without outputting to dashboard
    initPID(false);
  }

  /*
   * Initializes Computer Vision
   */
  private void initVision(){

    // colors
    Scalar redColor = new Scalar(0, 0, 255);
    Scalar yellowColor = new Scalar(0, 255, 255);

    // BGR thresholding values
    Scalar redLower = new Scalar(0, 0, 55);
    Scalar redUpper = new Scalar(80, 35, 255);
    
    Scalar yellowLower = new Scalar(0, 150, 175);
    Scalar yellowUpper = new Scalar(140, 200, 230);


    // gaussian blur
    Size gb = new Size(7, 7);

    visionThread = new Thread(() -> {
      // add USB camera, create server for SmartDashboard
      UsbCamera usbCamera = CameraServer.startAutomaticCapture("Main Camera", 0);
      usbCamera.setResolution(imgWidth, imgHeight);

      CvSink cvSink = CameraServer.getVideo(); // grab images from camera
      CvSource outputStream = CameraServer.putVideo("Processed Image", imgWidth, imgHeight);

      Mat sourceMat = new Mat();
      Mat redMask = new Mat();
      Mat yellowMask = new Mat();
      // Mat mask = new Mat();
      Mat black = Mat.zeros(imgHeight, imgWidth, 16);
      // Mat kernelOpen = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5.0, 5.0));
      List<MatOfPoint> contoursRed = new ArrayList<MatOfPoint>();
      Mat hierarchyRed = new Mat();
      List<MatOfPoint> contoursYellow = new ArrayList<MatOfPoint>();
      Mat hierarchyYellow = new Mat();
      Rect br;
      Rect biggestRed = new Rect(0, 0, 0, 0);
      Rect biggestYellow = new Rect(0, 0, 0, 0);

      // ~40 ms per loop
      while(true){ /// TODO: change condition later
        long startTime = System.currentTimeMillis();
        

        if (cvSink.grabFrame(sourceMat) != 0){

          // gaussian blur
          Imgproc.GaussianBlur(sourceMat, sourceMat, gb, 0, 0);

          // red thresholding
          Core.inRange(sourceMat, redLower, redUpper, redMask);

          // yellow thresholding
          Core.inRange(sourceMat, yellowLower, yellowUpper, yellowMask);

          // find bounding boxes
          contoursRed = new ArrayList<MatOfPoint>();
          contoursYellow = new ArrayList<MatOfPoint>();
          biggestRed = new Rect(0, 0, 0, 0);
          biggestYellow = new Rect(0, 0, 0, 0);
          Imgproc.findContours(redMask, contoursRed, hierarchyRed, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
          Imgproc.findContours(yellowMask, contoursYellow, hierarchyYellow, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

          // System.out.println(contours.size());
          black.copyTo(sourceMat);
          for (int i = 0; i < Math.min(contoursRed.size(), maxObjectColors); i++) {
            // Imgproc.drawContours(sourceMat, contours, i, redColor, 2, Imgproc.LINE_8, hierarchy, 0, new Point());
            br = Imgproc.boundingRect(contoursRed.get(i));
            if(br.area() > biggestRed.area()){
              biggestRed = br;
            }
            if(verbose){
              Imgproc.rectangle(sourceMat, br.tl(), br.br(), redColor, 1);
            }
          }

          for (int i = 0; i < Math.min(contoursYellow.size(), maxObjectColors); i++) {
            // Imgproc.drawContours(sourceMat, contours, i, redColor, 2, Imgproc.LINE_8, hierarchy, 0, new Point());
            br = Imgproc.boundingRect(contoursYellow.get(i));
            if(br.area() > biggestYellow.area()){
              biggestYellow = br;
            }
            if(verbose){
              Imgproc.rectangle(sourceMat, br.tl(), br.br(), yellowColor, 1);
            }
          }

          if(!verbose){
            Imgproc.rectangle(sourceMat, biggestRed.tl(), biggestRed.br(), redColor, 1);
            Imgproc.rectangle(sourceMat, biggestYellow.tl(), biggestYellow.br(), yellowColor, 1);
          }

          // output results
          // black.copyTo(sourceMat);
          sourceMat.setTo(yellowColor, yellowMask);
          sourceMat.setTo(redColor, redMask);

          outputStream.putFrame(sourceMat); // put processed image to smartdashboard
          long endTime = System.currentTimeMillis();

          System.out.println("Total execution time: " + (endTime - startTime));
        }
      }
    

    });

    visionThread.setPriority(10); // highest priority
    visionThread.start(); // start vision thread
  }
  // ------------------------------------------------------------------------------

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

    SmartDashboard.putNumber("RM6_Encoder.getPosition()", current_rel[0]); 
    SmartDashboard.putNumber("hopefully close to CANCODER 3", wrapEncoderValues(-current_rel[0] - delta_Motor[0]));

    SmartDashboard.putNumber("RM4_Encoder.getPosition()", current_rel[1]); 
    SmartDashboard.putNumber("hopefully close to CANCODER 2", wrapEncoderValues(-current_rel[1] - delta_Motor[1]));

    SmartDashboard.putNumber("RM6_Encoder.getPosition()", current_rel[2]); 
    SmartDashboard.putNumber("hopefully close to CANCODER 3", wrapEncoderValues(-current_rel[2] - delta_Motor[2]));

    SmartDashboard.putNumber("RM8_Encoder.getPosition()", current_rel[3]); 
    SmartDashboard.putNumber("hopefully close to CANCODER 3", wrapEncoderValues(-current_rel[3] - delta_Motor[3]));

    double joy = PS4joystick.getRightX();

    //double desired_body = 90;
    double desired_body = joy * 360.0;
    SmartDashboard.putNumber("desired_body", desired_body);

    double[] desired_abs = new double[]{
      wrapEncoderValues(desired_body - offset1[0]),
      wrapEncoderValues(desired_body - offset2[0]),
      wrapEncoderValues(desired_body - offset3[0]),
      wrapEncoderValues(desired_body - offset4[0])
    };

    double[] desired_rel = new double[]{
      -(desired_abs[0] + delta_Motor[0]),
      -(desired_abs[1] + delta_Motor[1]),
      -(desired_abs[2] + delta_Motor[2]),
      -(desired_abs[3] + delta_Motor[3])
    };


    double[] res1 = wrapWheelCommand(desired_rel[0], current_rel[0]);
    double[] res2 =wrapWheelCommand(desired_rel[1], current_rel[2]);
    double[] res3 =wrapWheelCommand(desired_rel[2], current_rel[2]);
    double[] res4 =wrapWheelCommand(desired_rel[3], current_rel[3]);

    double[] desired_rel1 = new double[]{
      res1[0],
      res2[0],
      res3[0],
      res4[0]
    };

    RM2_PidController.setReference(desired_rel1[0], CANSparkMax.ControlType.kSmartMotion);
    RM4_PidController.setReference(desired_rel1[0], CANSparkMax.ControlType.kSmartMotion);
    RM6_PidController.setReference(desired_rel1[0], CANSparkMax.ControlType.kSmartMotion);
    RM8_PidController.setReference(desired_rel1[0], CANSparkMax.ControlType.kSmartMotion);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}