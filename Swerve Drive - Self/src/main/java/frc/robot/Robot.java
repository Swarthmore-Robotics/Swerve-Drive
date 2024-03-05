// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// CAN + SPARKMAX
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
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

import javax.management.Descriptor;

import org.opencv.core.Core;
import java.util.List;
import java.text.BreakIterator;
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

  private List<SparkMaxPIDController> RM_PIDControllers = new ArrayList<SparkMaxPIDController>();
  private List<RelativeEncoder> RM_Encoders = new ArrayList<RelativeEncoder>();

  private List<SparkMaxPIDController> TM_PIDControllers = new ArrayList<SparkMaxPIDController>();
  private List<RelativeEncoder> TM_Encoders = new ArrayList<RelativeEncoder>();

  private final Timer m_Timer = new Timer();

  private double rotConvFactor = 5.62279270244;
  // private double transConvFactor = 1;
  private double transConvFactor = 0.1875; // RPM
  // private double transConvFactor = 0.0014961835; // m/s

  // joystick controller
  private final PS4Controller PS4joystick = new PS4Controller(0); // 0 is the USB Port to be used as indicated on the Driver Station

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
  public double Trans_kP = 2e-4;
  // public double Trans_kI = 1e-6;
  public double Trans_kI = 0;
  public double Trans_kD = 0;
  public double Trans_kIz = 0; 
  public double Trans_kFF = 0; 
  public double Trans_kMaxOutput = 1; 
  public double Trans_kMinOutput = -1;
  public double Trans_maxRPM = 5500;

  // encoder offset variables, should have two arrays of length 4: 
  // alpha to store abs offsets, delta to store rel offsets, alpha be final 
  public final double[] alpha_Motor = new double[]{14.589844, 51.064453, 117.949219, 124.365234};
  public double[] delta_Motor = new double[]{0.0, 0.0, 0.0, 0.0};

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
    // // initialize vision
    // initVision();

    // initialize PID
    initPID();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    TranslationMotors[WHEEL_FL].setIdleMode(IdleMode.kCoast);
    TranslationMotors[WHEEL_FR].setIdleMode(IdleMode.kCoast);
    TranslationMotors[WHEEL_BR].setIdleMode(IdleMode.kCoast);
    TranslationMotors[WHEEL_BL].setIdleMode(IdleMode.kCoast);
    m_Timer.reset();
    m_Timer.start();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    double[]current_abs = new double[]{
      coders[WHEEL_FL].getAbsolutePosition(),
      coders[WHEEL_FR].getAbsolutePosition(),
      coders[WHEEL_BR].getAbsolutePosition(),
      coders[WHEEL_BL].getAbsolutePosition()
    };
    double[]current_rel = new double[]{
      RM_Encoders.get(WHEEL_FL).getPosition(),
      RM_Encoders.get(WHEEL_FR).getPosition(),
      RM_Encoders.get(WHEEL_BR).getPosition(),
      RM_Encoders.get(WHEEL_BL).getPosition()
    };

    // Print absolute CANCoder values
    printDB("CANCODER", current_abs);

    // Print relative Encoder values
    printDB("RM_Encoder.getPosition()", current_rel);
    
    // Initialize empty arrays for RM and TM setPoint values to be used later by setReference 
    double[] desired_body = new double[]{0.0, 0.0, 0.0, 0.0};
    double[] desired_rel1 = new double[]{0.0, 0.0, 0.0, 0.0};
    double[] desired_translation = new double[]{0.0, 0.0, 0.0, 0.0};

    // in case 3, set each wheel angle to some 90 degree offset of each other
    double setAngle = 0.0;
    double[] DESIRED = new double[] {0, 0, 0, 0};

    //move 1m forward
    if (m_Timer.get() < 1.5709) {
      setAngle = 0.0;
      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);
      RM_PIDControllers.get(WHEEL_FL).setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_FR).setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BR).setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BL).setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);
  
      double setpoint = 1 * (Trans_maxRPM/6);
      TM_PIDControllers.get(WHEEL_FL).setReference(setpoint * desired_translation[WHEEL_FL], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_FR).setReference(setpoint * desired_translation[WHEEL_FR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BR).setReference(setpoint * desired_translation[WHEEL_BR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BL).setReference(setpoint * desired_translation[WHEEL_BL], CANSparkMax.ControlType.kVelocity);
      
      SmartDashboard.putNumber("Timer: ", m_Timer.get());
    }

    // //time discontinuity: 1 second
    // //move 1m backwards
    else if (m_Timer.get() > 2.5709 && m_Timer.get() < 4.1418) {
      setAngle = 0.0;
      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);

      SmartDashboard.putNumber("Timer 2: ", m_Timer.get());
      RM_PIDControllers.get(WHEEL_FL).setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_FR).setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BR).setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BL).setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);
  
      double setpoint = -1 * (Trans_maxRPM/6);
      TM_PIDControllers.get(WHEEL_FL).setReference(setpoint * desired_translation[WHEEL_FL], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_FR).setReference(setpoint * desired_translation[WHEEL_FR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BR).setReference(setpoint * desired_translation[WHEEL_BR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BL).setReference(setpoint * desired_translation[WHEEL_BL], CANSparkMax.ControlType.kVelocity);
    }

    //time discontinuity: 1 second
    //move 1m right
    else if (m_Timer.get() > 5.1418 && m_Timer.get() < 6.7127) {
      setAngle = 90;
      DESIRED[0] = setAngle;
      DESIRED[1] = setAngle;
      DESIRED[2] = setAngle;
      DESIRED[3] = setAngle;

      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);
      
      RM_PIDControllers.get(WHEEL_FL).setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_FR).setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BR).setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BL).setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);
  
      double setpoint = 1 * (Trans_maxRPM/6);
      TM_PIDControllers.get(WHEEL_FL).setReference(setpoint * desired_translation[WHEEL_FL], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_FR).setReference(setpoint * desired_translation[WHEEL_FR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BR).setReference(setpoint * desired_translation[WHEEL_BR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BL).setReference(setpoint * desired_translation[WHEEL_BL], CANSparkMax.ControlType.kVelocity);
      SmartDashboard.putNumber("Timer 3: ", m_Timer.get());
    }
     //time discontinuity: 1 second
    //move 1m left
    else if (m_Timer.get() > 7.7127 && m_Timer.get() < 8.2836) {
      setAngle = 90;
      DESIRED[0] = setAngle;
      DESIRED[1] = setAngle;
      DESIRED[2] = setAngle;
      DESIRED[3] = setAngle;

      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);

      RM_PIDControllers.get(WHEEL_FL).setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_FR).setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BR).setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BL).setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);
  
      double setpoint = -1 * (Trans_maxRPM/6);
      TM_PIDControllers.get(WHEEL_FL).setReference(setpoint * desired_translation[WHEEL_FL], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_FR).setReference(setpoint * desired_translation[WHEEL_FR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BR).setReference(setpoint * desired_translation[WHEEL_BR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BL).setReference(setpoint * desired_translation[WHEEL_BL], CANSparkMax.ControlType.kVelocity);
      SmartDashboard.putNumber("Timer 4: ", m_Timer.get());
    }

    else {
      // command everything to zero
      TranslationMotors[WHEEL_FL].setIdleMode(IdleMode.kBrake);
      TranslationMotors[WHEEL_FR].setIdleMode(IdleMode.kBrake);
      TranslationMotors[WHEEL_BR].setIdleMode(IdleMode.kBrake);
      TranslationMotors[WHEEL_BL].setIdleMode(IdleMode.kBrake);
      stopMotors();
    }

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

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
      // return Math.pow(rawInput, 3);
      return rawInput;
    }
  }

  private void stopMotors() {
    RotationMotors[WHEEL_FL].set(0);
    RotationMotors[WHEEL_FR].set(0);
    RotationMotors[WHEEL_BR].set(0);
    RotationMotors[WHEEL_BL].set(0);

    TranslationMotors[WHEEL_FL].set(0);
    TranslationMotors[WHEEL_FR].set(0);
    TranslationMotors[WHEEL_BR].set(0);
    TranslationMotors[WHEEL_BL].set(0);
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
  private void printDB(String name, double[] arr){ // print array of values
    String wheel_orientation = "";

    for (int i = 0; i < arr.length; i++) {
      
      switch(i) {
        case(0):
        wheel_orientation = "WHEEL_FL";
        break;
        case(1):
        wheel_orientation = "WHEEL_FR";
        break;
        case(2):
        wheel_orientation = "WHEEL_BR";
        break;
        case(3):
        wheel_orientation = "WHEEL_BL";
        break;
      }

      SmartDashboard.putNumber(name + "[" + wheel_orientation + "]", arr[i]);
    }
    // SmartDashboard.putString(name, s);
  }
  
  private void initPID(boolean verbose){

    // Rotation PID controllers
    SparkMaxPIDController RM2_PidController = RotationMotors[WHEEL_FL].getPIDController();
    RelativeEncoder RM2_Encoder = RotationMotors[WHEEL_FL].getEncoder();
    RM2_Encoder.setPositionConversionFactor(rotConvFactor);

    SparkMaxPIDController RM4_PidController = RotationMotors[WHEEL_FR].getPIDController();
    RelativeEncoder RM4_Encoder = RotationMotors[WHEEL_FR].getEncoder();
    RM4_Encoder.setPositionConversionFactor(rotConvFactor);

    SparkMaxPIDController RM6_PidController = RotationMotors[WHEEL_BR].getPIDController();
    RelativeEncoder RM6_Encoder = RotationMotors[WHEEL_BR].getEncoder();
    RM6_Encoder.setPositionConversionFactor(rotConvFactor);

    SparkMaxPIDController RM8_PidController = RotationMotors[WHEEL_BL].getPIDController();
    RelativeEncoder RM8_Encoder = RotationMotors[WHEEL_BL].getEncoder();
    RM8_Encoder.setPositionConversionFactor(rotConvFactor);

    RM_PIDControllers.add(RM2_PidController);
    RM_PIDControllers.add(RM4_PidController);
    RM_PIDControllers.add(RM6_PidController);
    RM_PIDControllers.add(RM8_PidController);

    RM_Encoders.add(RM2_Encoder);
    RM_Encoders.add(RM4_Encoder);
    RM_Encoders.add(RM6_Encoder);
    RM_Encoders.add(RM8_Encoder);

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

    // Translation PID controllers
    SparkMaxPIDController TM1_PidController = TranslationMotors[WHEEL_FL].getPIDController();
    RelativeEncoder TM1_Encoder = TranslationMotors[WHEEL_FL].getEncoder();
    TM1_Encoder.setVelocityConversionFactor(transConvFactor);

    SparkMaxPIDController TM3_PidController = TranslationMotors[WHEEL_FR].getPIDController();
    RelativeEncoder TM3_Encoder = TranslationMotors[WHEEL_FR].getEncoder();
    TM3_Encoder.setVelocityConversionFactor(transConvFactor);
    
    SparkMaxPIDController TM5_PidController = TranslationMotors[WHEEL_BR].getPIDController();
    RelativeEncoder TM5_Encoder = TranslationMotors[WHEEL_BR].getEncoder();
    TM5_Encoder.setVelocityConversionFactor(transConvFactor);
    
    SparkMaxPIDController TM7_PidController = TranslationMotors[WHEEL_BL].getPIDController();
    RelativeEncoder TM7_Encoder = TranslationMotors[WHEEL_BL].getEncoder();
    TM7_Encoder.setVelocityConversionFactor(transConvFactor);

    TM_PIDControllers.add(TM1_PidController);
    TM_PIDControllers.add(TM3_PidController);
    TM_PIDControllers.add(TM5_PidController);
    TM_PIDControllers.add(TM7_PidController);

    TM_Encoders.add(TM1_Encoder);
    TM_Encoders.add(TM3_Encoder);
    TM_Encoders.add(TM5_Encoder);
    TM_Encoders.add(TM7_Encoder);

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

    // define abs to relative encoder offset values
    delta_Motor[WHEEL_FL] = wrapEncoderValues((-1*coders[WHEEL_FL].getAbsolutePosition()) - RM_Encoders.get(WHEEL_FL).getPosition());
    delta_Motor[WHEEL_FR] = wrapEncoderValues((-1*coders[WHEEL_FR].getAbsolutePosition()) - RM_Encoders.get(WHEEL_FR).getPosition());
    delta_Motor[WHEEL_BR] = wrapEncoderValues((-1*coders[WHEEL_BR].getAbsolutePosition()) - RM_Encoders.get(WHEEL_BR).getPosition());
    delta_Motor[WHEEL_BL] = wrapEncoderValues((-1*coders[WHEEL_BL].getAbsolutePosition()) - RM_Encoders.get(WHEEL_BL).getPosition());
  }

  private void initPID(){
     // init PID without outputting to dashboard
    initPID(false);
  }

  // Computer Vision Methods

  /*
   * Calculate center of rectangle
   */
   private Point rectCenter(Rect r){
      return new Point(r.tl().x + (r.width/2.0), r.tl().y + (r.height/2.0));
   }

  /*
   * Initializes Computer Vision
   */
  private void initVision(){

    // colors
    Scalar redColor = new Scalar(0, 0, 255);
    Scalar yellowColor = new Scalar(0, 255, 255);
    Scalar greenColor = new Scalar(0, 255, 0);
    Scalar blueColor = new Scalar(255, 0, 0);
    Scalar pinkColor = new Scalar(255, 0, 255);

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

          sourceMat.setTo(yellowColor, yellowMask);
          sourceMat.setTo(redColor, redMask);

          if(!verbose){
            Imgproc.rectangle(sourceMat, biggestRed.tl(), biggestRed.br(), redColor, 1);
            Imgproc.rectangle(sourceMat, biggestYellow.tl(), biggestYellow.br(), yellowColor, 1);
            Imgproc.circle(sourceMat, rectCenter(biggestRed), 3, blueColor, -1);
            Imgproc.circle(sourceMat, rectCenter(biggestYellow), 3, blueColor, -1);
          }

          // output results
          // black.copyTo(sourceMat);

          outputStream.putFrame(sourceMat); // put processed image to smartdashboard
          long endTime = System.currentTimeMillis();

          System.out.println("Total execution time: " + (endTime - startTime));

          SmartDashboard.putString("Biggest Red Center", rectCenter(biggestRed).toString());
          SmartDashboard.putString("Biggest Yellow Center", rectCenter(biggestYellow).toString());
        }

      }
    
    });

    visionThread.setPriority(10); // highest priority
    visionThread.start(); // start vision thread
  }
  // ------------------------------------------------------------------------------

  /*
   * Sets the desired wheel angles and direction of translation for each wheel module, per case
   */
  private void set_desired(double[] desired_body, double[] DESIRED, double[] current_rel, double[] desired_rel1, double[] desired_translation, int CASE) {

    // No rotation, CASE 1
    if (CASE == 1) {
      // Set desired angles based on input parameter DESIRED storing desired wheel angles in robot frame
      desired_body[WHEEL_FL] = DESIRED[WHEEL_FL];
      desired_body[WHEEL_FR] = DESIRED[WHEEL_FR];
      desired_body[WHEEL_BR] = DESIRED[WHEEL_BR];
      desired_body[WHEEL_BL] = DESIRED[WHEEL_BL];

      // Subtract off absolute encoder offset to get desired wheel angles in terms of absolute encoder
      double[] desired_abs = new double[]{
        wrapEncoderValues(desired_body[WHEEL_FL] - alpha_Motor[WHEEL_FL]),
        wrapEncoderValues(desired_body[WHEEL_FR] - alpha_Motor[WHEEL_FR]),
        wrapEncoderValues(desired_body[WHEEL_BR] - alpha_Motor[WHEEL_BR]),
        wrapEncoderValues(desired_body[WHEEL_BL] - alpha_Motor[WHEEL_BL])
      };
      
      // Subtract off relative encoder offset to get desired wheel angles in terms of relative encoder
      double[] desired_rel = new double[]{
        -(desired_abs[WHEEL_FL] + delta_Motor[WHEEL_FL]),
        -(desired_abs[WHEEL_FR] + delta_Motor[WHEEL_FR]),
        -(desired_abs[WHEEL_BR] + delta_Motor[WHEEL_BR]),
        -(desired_abs[WHEEL_BL] + delta_Motor[WHEEL_BL])
      };
      
      // Given a desired angle and current angle both in terms of relative encoder, find optimal path
      // to the desired angle from current angle
      double[] res1 = wrapWheelCommand(desired_rel[WHEEL_FL], current_rel[WHEEL_FL]);
      double[] res2 = wrapWheelCommand(desired_rel[WHEEL_FR], current_rel[WHEEL_FR]);
      double[] res3 = wrapWheelCommand(desired_rel[WHEEL_BR], current_rel[WHEEL_BR]);
      double[] res4 = wrapWheelCommand(desired_rel[WHEEL_BL], current_rel[WHEEL_BL]);
      
      // Store results into appropriate arrays:
      // Rotation results
      desired_rel1[WHEEL_FL] = res1[0];
      desired_rel1[WHEEL_FR] = res2[0];
      desired_rel1[WHEEL_BR] = res3[0];
      desired_rel1[WHEEL_BL] = res4[0];

      // Translation results (whether to flip or not)
      desired_translation[WHEEL_FL] = -1 * res1[1];
      desired_translation[WHEEL_FR] = res2[1];
      desired_translation[WHEEL_BR] = -1 * res3[1];
      desired_translation[WHEEL_BL] = res4[1];
      }

    // General Case, CASE 2
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

      desired_translation[WHEEL_FL] = -1 * res1[1];
      desired_translation[WHEEL_FR] = res2[1];
      desired_translation[WHEEL_BR] = -1 * res3[1];
      desired_translation[WHEEL_BL] = res4[1];
    }

    // Rotate in place, CASE 3
    else if (CASE == 3) {

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
    
  }
  
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    
    double[]current_abs = new double[]{
      coders[WHEEL_FL].getAbsolutePosition(),
      coders[WHEEL_FR].getAbsolutePosition(),
      coders[WHEEL_BR].getAbsolutePosition(),
      coders[WHEEL_BL].getAbsolutePosition()
    };
    double[]current_rel = new double[]{
      RM_Encoders.get(WHEEL_FL).getPosition(),
      RM_Encoders.get(WHEEL_FR).getPosition(),
      RM_Encoders.get(WHEEL_BR).getPosition(),
      RM_Encoders.get(WHEEL_BL).getPosition()
    };

    // Print absolute CANCoder values
    printDB("CANCODER", current_abs);

    // Print relative Encoder values
    printDB("RM_Encoder.getPosition()", current_rel);

    // Obtain mapped and filtered joystick inputs 
    double Vx = (-1) * highpassFilter(PS4joystick.getLeftY());
    double Vy = (-1) * highpassFilter(PS4joystick.getLeftX());
    double omega = (-1) * highpassFilter(PS4joystick.getRightX());
    double Vr = mag(Vx, Vy);

    // Print joystick inputs
    printDB("Vx", Vx);
    printDB("Vy", Vy);
    printDB("w (omega)", omega);
    printDB("Vr", Vr);

    // Initialize empty arrays for RM and TM setPoint values to be used later by setReference 
    double[] desired_body = new double[]{0.0, 0.0, 0.0, 0.0};
    double[] desired_rel1 = new double[]{0.0, 0.0, 0.0, 0.0};
    double[] desired_translation = new double[]{0.0, 0.0, 0.0, 0.0};

    // Constants
    double x = 12.75; //distance from chassis center to module - x-component
    double y = 12.75; //distance from chassis center to module - y-component
    double r = mag(x,y); //needed to make x,y dimensionless

    double MAX_rad_s = 1.531707;
    double MAX_m_s = 0.635617;
    
    // No rotation, CASE 1
    if (omega == 0 & (Vx != 0 | Vy != 0)) {
      printDB("Case", 1);

      // define robot angle to be direction of joystick vectors, Vx and Vy
      // double setAngle = ((Math.atan2(Vy,Vx) * 180) / Math.PI);
      double setAngle = 0.0;

      // in case 1, set every wheel to same desired setAngle
      double[] DESIRED = new double[]{setAngle, setAngle, setAngle, setAngle};

      // call set_desired to populate empty arrays with correct values based on the current case
      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);

      RM_PIDControllers.get(WHEEL_FL).setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_FR).setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BR).setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BL).setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);
      
      double setpoint = Vr * (Trans_maxRPM/6);
      printDB("setpoint for case 1",setpoint);

      TM_PIDControllers.get(WHEEL_FL).setReference(setpoint * desired_translation[WHEEL_FL], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_FR).setReference(setpoint * desired_translation[WHEEL_FR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BR).setReference(setpoint * desired_translation[WHEEL_BR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BL).setReference(setpoint * desired_translation[WHEEL_BL], CANSparkMax.ControlType.kVelocity);
    }
    
    // General Case, CASE 2
    else if (omega != 0 & (Vx != 0 | Vy != 0)) {
      SmartDashboard.putNumber("Case", 2);

      // x component of each wheel is made up of x component of left joystick and x component of right joystick
      double v1x = -1*Vy + (-1*(omega * y/r));
      double v2x = -1*Vy + (-1*(omega * y/r));
      double v3x = -1*Vy + (omega * y/r);
      double v4x = -1*Vy + (omega * y/r);
      SmartDashboard.putNumber("v1x", v1x);
      SmartDashboard.putNumber("v2x", v2x);
      SmartDashboard.putNumber("v3x", v3x);
      SmartDashboard.putNumber("v4x", v4x);

      double v1y = Vx + (-1*(omega * x/r));
      double v2y = Vx + (omega * x/r);
      double v3y = Vx + (omega * x/r);
      double v4y = Vx + (-1*(omega * x/r));
      SmartDashboard.putNumber("v1y", v1y);
      SmartDashboard.putNumber("v2y", v2y);
      SmartDashboard.putNumber("v3y", v3y);
      SmartDashboard.putNumber("v4y", v4y);

      double q1 = mag(v1x, v1y);
      double q2 = mag(v2x, v2y);
      double q3 = mag(v3x, v3y);
      double q4 = mag(v4x, v4y);

      SmartDashboard.putNumber("q1", q1);
      SmartDashboard.putNumber("q2", q2);
      SmartDashboard.putNumber("q3", q3);
      SmartDashboard.putNumber("q4", q4);

      double Omega_wheel1 = -1*(Math.atan2(v1x, v1y) * 180) / Math.PI;
      double Omega_wheel2 = -1*(Math.atan2(v2x, v2y) * 180) / Math.PI;
      double Omega_wheel3 = -1*(Math.atan2(v3x, v3y) * 180) / Math.PI;
      double Omega_wheel4 = -1*(Math.atan2(v4x, v4y) * 180) / Math.PI;
      SmartDashboard.putNumber("Omega_wheel1", Omega_wheel1);
      SmartDashboard.putNumber("Omega_wheel2", Omega_wheel2);
      SmartDashboard.putNumber("Omega_wheel3", Omega_wheel3);
      SmartDashboard.putNumber("Omega_wheel4", Omega_wheel4);

      double[] DESIRED = new double[] {Omega_wheel1, Omega_wheel2, Omega_wheel3, Omega_wheel4};

      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 2);
      RM_PIDControllers.get(WHEEL_FL).setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_FR).setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BR).setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BL).setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);

      double[] setpoint = new double[]{q1 * Trans_maxRPM/6, q2 * Trans_maxRPM/6, q3 * Trans_maxRPM/6, q4 * Trans_maxRPM/6};

      TM_PIDControllers.get(WHEEL_FL).setReference(desired_translation[WHEEL_FL] * setpoint[WHEEL_FL], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_FR).setReference(desired_translation[WHEEL_FR] * setpoint[WHEEL_FR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BR).setReference(desired_translation[WHEEL_BR] * setpoint[WHEEL_BR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BL).setReference(desired_translation[WHEEL_BL] * setpoint[WHEEL_BL], CANSparkMax.ControlType.kVelocity);
      }

    // Rotate in place, CASE 3
    else if (Vx == 0.0 & Vy == 0.0 & omega != 0) {
      printDB("Case", 3);

      // in case 3, set each wheel angle to some 90 degree offset of each other
      double[] DESIRED = new double[] {-45, 45, 135, 225};

      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 3);
      RM_PIDControllers.get(WHEEL_FL).setReference(desired_rel1[WHEEL_FL], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_FR).setReference(desired_rel1[WHEEL_FR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BR).setReference(desired_rel1[WHEEL_BR], CANSparkMax.ControlType.kSmartMotion);
      RM_PIDControllers.get(WHEEL_BL).setReference(desired_rel1[WHEEL_BL], CANSparkMax.ControlType.kSmartMotion);

      double setpoint = omega * (Trans_maxRPM/6);
      TM_PIDControllers.get(WHEEL_FL).setReference(setpoint * desired_translation[WHEEL_FL], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_FR).setReference(setpoint * desired_translation[WHEEL_FR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BR).setReference(setpoint * desired_translation[WHEEL_BR], CANSparkMax.ControlType.kVelocity);
      TM_PIDControllers.get(WHEEL_BL).setReference(setpoint * desired_translation[WHEEL_BL], CANSparkMax.ControlType.kVelocity);
  } 
    
    // No input, CASE 4
    else {
      SmartDashboard.putNumber("Case", 4);

      // command everything to zero
      stopMotors();
    }
  
}

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}