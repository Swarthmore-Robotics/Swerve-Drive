// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// CAN + SPARKMAX
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.CANCoder;

// WPILIB, Servo
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.networktables.DoubleSubscriber;
// Raspbarry Pi Vision
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import java.io.IOException;

import edu.wpi.first.wpilibj.I2C;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
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
      new CANCoder(1),
      new CANCoder(2),
      new CANCoder(3),
      new CANCoder(4),
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

  private final PS4Controller PS4joystick = new PS4Controller(0); 

  private static I2C arduino;

  // offset value array for going from abs to relative, changes on startup
  public double[] delta_Motor = new double[] { 0.0, 0.0, 0.0, 0.0 };

  final double cool_omega = -0.7; // rad / s
  final double cool_speed = 0.4; // m / s
  private double Vx_auto;
  private double Vy_auto;
  private double omega_auto;
  private double grip_flag = 0;

  // Vision Variables
  private static Constants C = new Constants();
  DoubleSubscriber redASub;
  DoubleSubscriber redXSub;
  DoubleSubscriber redYSub;

  enum autoStates {
    findBlock,
    move,
    stopped,
    cool
  }

  private autoStates currState;

  enum Colors {
    red,
    yellow,
    green
  }

  private Colors colorOfInterest;
  private double area;


  /* ------------------------------------------------------------------------- */
  /* ----------------------------- Swerve Methods ---------------------------- */
  /* ------------------------------------------------------------------------- */

  /*
   * Returns a filtered value of some raw input by setting values in a range
   * from -0.1 to 0.1 to 0.
   */
  private double highpassFilter(double rawInput) {
    if (Math.abs(rawInput) < 0.1) {
      return 0.0;
    } else {
      return rawInput;
    }
  }

  /*
   * Returns the magnitude of elements x and y
   */
  private double mag(double x, double y) {
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

    else if ((EncPos < -180.0)) {
      return 180.0 - (((-1 * EncPos) + 180.0) % 360);
    }

    else if ((EncPos > 180.0)) {
      return -180.0 + ((EncPos - 180.0) % 360);
    }

    return EncPos;
  }

  /*
   * Returns modified desired to be within 180 degrees of current.
   * desired_rel is a relative encoder destination that could be modified.
   * current_rel is the current encoder position in relative degrees.
   * Both of the above can be arbitrary since encoders count up or down
   * continuously.
   */
  private double[] wrapWheelCommand(double desired_rel, double current_rel) {

    // diff is always in [-180, 180]
    double diff = wrapEncoderValues(desired_rel - current_rel);

    double[] res = new double[] { 0.0, 0.0 };
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
   * Setter method for setting PID constants
   */
  private void setMotorPID(List<SparkMaxPIDController> pidControllers, String R_T_Flag) {

    int smartMotionSlot = 0;

    for (int i = 0; i < pidControllers.size(); i++) {

      if (R_T_Flag == "R") {
        pidControllers.get(i).setP(C.Rot_kP);
        pidControllers.get(i).setI(C.Rot_kI);
        pidControllers.get(i).setD(C.Rot_kD);
        pidControllers.get(i).setIZone(C.Rot_kIz);
        pidControllers.get(i).setFF(C.Rot_kFF);
        pidControllers.get(i).setOutputRange(C.Rot_kMinOutput, C.Rot_kMaxOutput);

        pidControllers.get(i).setSmartMotionMaxVelocity(C.Rot_maxVel, smartMotionSlot);
        pidControllers.get(i).setSmartMotionMinOutputVelocity(C.Rot_minVel, smartMotionSlot);
        pidControllers.get(i).setSmartMotionMaxAccel(C.Rot_maxAcc, smartMotionSlot);
        pidControllers.get(i).setSmartMotionAllowedClosedLoopError(C.Rot_allowedErr, smartMotionSlot);
      } else if (R_T_Flag == "T") {
        pidControllers.get(i).setP(C.Trans_kP);
        pidControllers.get(i).setI(C.Trans_kI);
        pidControllers.get(i).setD(C.Trans_kD);
        pidControllers.get(i).setIZone(C.Trans_kIz);
        pidControllers.get(i).setFF(C.Trans_kFF);
        pidControllers.get(i).setOutputRange(C.Trans_kMinOutput, C.Trans_kMaxOutput);
      }

    }
  }

  /*
   * Initializes Motor PID controllers, set PID constants, and calculate relative
   * offset
   * values on robot startup
   */
  private void initPID() {

    // Rotation PID controllers
    SparkMaxPIDController RM2_PidController = RotationMotors[WHEEL_FL].getPIDController();
    RelativeEncoder RM2_Encoder = RotationMotors[WHEEL_FL].getEncoder();
    RM2_Encoder.setPositionConversionFactor(C.rotConvFactor);

    SparkMaxPIDController RM4_PidController = RotationMotors[WHEEL_FR].getPIDController();
    RelativeEncoder RM4_Encoder = RotationMotors[WHEEL_FR].getEncoder();
    RM4_Encoder.setPositionConversionFactor(C.rotConvFactor);

    SparkMaxPIDController RM6_PidController = RotationMotors[WHEEL_BR].getPIDController();
    RelativeEncoder RM6_Encoder = RotationMotors[WHEEL_BR].getEncoder();
    RM6_Encoder.setPositionConversionFactor(C.rotConvFactor);

    SparkMaxPIDController RM8_PidController = RotationMotors[WHEEL_BL].getPIDController();
    RelativeEncoder RM8_Encoder = RotationMotors[WHEEL_BL].getEncoder();
    RM8_Encoder.setPositionConversionFactor(C.rotConvFactor);

    RM_PIDControllers.add(RM2_PidController);
    RM_PIDControllers.add(RM4_PidController);
    RM_PIDControllers.add(RM6_PidController);
    RM_PIDControllers.add(RM8_PidController);

    RM_Encoders.add(RM2_Encoder);
    RM_Encoders.add(RM4_Encoder);
    RM_Encoders.add(RM6_Encoder);
    RM_Encoders.add(RM8_Encoder);

    setMotorPID(RM_PIDControllers, "R");

    // Translation PID controllers
    SparkMaxPIDController TM1_PidController = TranslationMotors[WHEEL_FL].getPIDController();
    RelativeEncoder TM1_Encoder = TranslationMotors[WHEEL_FL].getEncoder();
    TM1_Encoder.setVelocityConversionFactor(C.transConvFactor);

    SparkMaxPIDController TM3_PidController = TranslationMotors[WHEEL_FR].getPIDController();
    RelativeEncoder TM3_Encoder = TranslationMotors[WHEEL_FR].getEncoder();
    TM3_Encoder.setVelocityConversionFactor(C.transConvFactor);

    SparkMaxPIDController TM5_PidController = TranslationMotors[WHEEL_BR].getPIDController();
    RelativeEncoder TM5_Encoder = TranslationMotors[WHEEL_BR].getEncoder();
    TM5_Encoder.setVelocityConversionFactor(C.transConvFactor);

    SparkMaxPIDController TM7_PidController = TranslationMotors[WHEEL_BL].getPIDController();
    RelativeEncoder TM7_Encoder = TranslationMotors[WHEEL_BL].getEncoder();
    TM7_Encoder.setVelocityConversionFactor(C.transConvFactor);

    TM_PIDControllers.add(TM1_PidController);
    TM_PIDControllers.add(TM3_PidController);
    TM_PIDControllers.add(TM5_PidController);
    TM_PIDControllers.add(TM7_PidController);

    TM_Encoders.add(TM1_Encoder);
    TM_Encoders.add(TM3_Encoder);
    TM_Encoders.add(TM5_Encoder);
    TM_Encoders.add(TM7_Encoder);

    setMotorPID(TM_PIDControllers, "T");

    // Set appropriate current limits for each motor type, though already done in
    // hardware
    TranslationMotors[WHEEL_FL].setSmartCurrentLimit(40);
    TranslationMotors[WHEEL_FR].setSmartCurrentLimit(40);
    TranslationMotors[WHEEL_BR].setSmartCurrentLimit(40);
    TranslationMotors[WHEEL_BL].setSmartCurrentLimit(40);

    RotationMotors[WHEEL_FL].setSmartCurrentLimit(20);
    RotationMotors[WHEEL_FR].setSmartCurrentLimit(20);
    RotationMotors[WHEEL_BR].setSmartCurrentLimit(20);
    RotationMotors[WHEEL_BL].setSmartCurrentLimit(20);

    // set delta_Motor array values to store offset values to go from abs to relative
    for (int i = 0; i < delta_Motor.length; i++) {
      delta_Motor[i] = wrapEncoderValues((-1 * coders[i].getAbsolutePosition()) - RM_Encoders.get(i).getPosition());
    }
  }

  /*
   * Close and open gripper based on a button press
   */
  private void closeGrip(boolean pressed, boolean released) {
    if (pressed) {
      // Arm.Gripper.set(Arm.GRIPPER_MAX);
      Arm.Gripper.setAngle(180);
    }
    else if (released) {
      // Arm.Gripper.set(Arm.GRIPPER_MIN);
      Arm.Gripper.setAngle(0);
    }
  }

  /*
   * Command motors to a given wheel angle or translation speed via setReference
   */
  private void setWheelState(List<SparkMaxPIDController> pidControllers, double[] desired_motion, boolean RM_flag, double[] setpoint) {
    
    if (RM_flag) {
      // Rotation Motors
      for (int i = 0; i <= 3; i ++) {
        RM_PIDControllers.get(i).setReference(desired_motion[i], CANSparkMax.ControlType.kSmartMotion);
      }

    }
    else {
      // Translation Motors
      for (int i = 0; i <= 3; i ++) {
        TM_PIDControllers.get(i).setReference(setpoint[i] * desired_motion[i], CANSparkMax.ControlType.kVelocity);
      }

    }

  }

  /*
   * Applies pre-defined wheel offset values and sets the desired wheel angles and direction of translation
   * for each wheel module
   */
  private void apply_offsets(double[] desired_body, double[] DESIRED, double[] current_rel, double[] desired_rel1,
      double[] desired_translation) {

    // Set desired angles based on input parameter DESIRED storing desired wheel
    // angles in robot frame
    desired_body[WHEEL_FL] = DESIRED[WHEEL_FL];
    desired_body[WHEEL_FR] = DESIRED[WHEEL_FR];
    desired_body[WHEEL_BR] = DESIRED[WHEEL_BR];
    desired_body[WHEEL_BL] = DESIRED[WHEEL_BL];

    // Subtract off absolute encoder offset to get desired wheel angles in terms of
    // absolute encoder
    double[] desired_abs = new double[] {
        wrapEncoderValues(desired_body[WHEEL_FL] - C.alpha_Motor[WHEEL_FL]),
        wrapEncoderValues(desired_body[WHEEL_FR] - C.alpha_Motor[WHEEL_FR]),
        wrapEncoderValues(desired_body[WHEEL_BR] - C.alpha_Motor[WHEEL_BR]),
        wrapEncoderValues(desired_body[WHEEL_BL] - C.alpha_Motor[WHEEL_BL])
    };

    // Subtract off relative encoder offset to get desired wheel angles in terms of
    // relative encoder
    double[] desired_rel = new double[] {
        -(desired_abs[WHEEL_FL] + delta_Motor[WHEEL_FL]),
        -(desired_abs[WHEEL_FR] + delta_Motor[WHEEL_FR]),
        -(desired_abs[WHEEL_BR] + delta_Motor[WHEEL_BR]),
        -(desired_abs[WHEEL_BL] + delta_Motor[WHEEL_BL])
    };

    // Given a desired angle and current angle both in terms of relative encoder,
    // find optimal path
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

    desired_translation[WHEEL_FL] = -1 * res1[1];
    desired_translation[WHEEL_FR] = res2[1];
    desired_translation[WHEEL_BR] = -1 * res3[1];
    desired_translation[WHEEL_BL] = res4[1];

  }

  /*
   * Calculates vector sum of linear and angular velocity vectors for General Case 2 
   */
  private double[] set_vectors(double Vx, double Vy, double omega, double x, double y, double r) {
    
    double v1x = -1 * Vy + (-1 * (omega * y / r));
    double v2x = -1 * Vy + (-1 * (omega * y / r));
    double v3x = -1 * Vy + (omega * y / r);
    double v4x = -1 * Vy + (omega * y / r);

    double v1y = Vx + (-1 * (omega * x / r));
    double v2y = Vx + (omega * x / r);
    double v3y = Vx + (omega * x / r);
    double v4y = Vx + (-1 * (omega * x / r));

    double q1 = mag(v1x, v1y) * C.Trans_maxRPM / 6;
    double q2 = mag(v2x, v2y) * C.Trans_maxRPM / 6;
    double q3 = mag(v3x, v3y) * C.Trans_maxRPM / 6;
    double q4 = mag(v4x, v4y) * C.Trans_maxRPM / 6;

    double Omega_wheel1 = -1 * (Math.atan2(v1x, v1y) * 180) / Math.PI;
    double Omega_wheel2 = -1 * (Math.atan2(v2x, v2y) * 180) / Math.PI;
    double Omega_wheel3 = -1 * (Math.atan2(v3x, v3y) * 180) / Math.PI;
    double Omega_wheel4 = -1 * (Math.atan2(v4x, v4y) * 180) / Math.PI;

    double[] state = new double[]{ Omega_wheel1, Omega_wheel2, Omega_wheel3, Omega_wheel4, q1, q2, q3, q4 };

    return state;
  }

  /*
   * Sets all motor speeds to 0
   */
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

  /* ------------------------------------------------------------------------- */
  /* ------------------------- Vision + Auto Methods ------------------------- */
  /* ------------------------------------------------------------------------- */

  /*
   * Initializes Computer Vision
   */
  private void initVision() {
    
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.setServer("6593");
    
    NetworkTable table = inst.getTable("Vision");
    redASub = table.getDoubleTopic("redA").subscribe(-1.0);
    redXSub = table.getDoubleTopic("redX").subscribe(-1.0);
    redYSub = table.getDoubleTopic("redY").subscribe(-1.0);

  }

  /*
   * Spin in place
   */
  private void spin(double[] DESIRED, double[] desired_body, double[] desired_rel1, double[] desired_translation, double[] current_rel) {
    DESIRED[0] = -45;
    DESIRED[1] = 45;
    DESIRED[2] = 135;
    DESIRED[3] = 225;

    double setpoint = 0.5 * (C.Trans_maxRPM / 7);
    double[] setpointArr = new double[] {-1 * setpoint, setpoint, -1 * setpoint, setpoint};

    apply_offsets(desired_body, DESIRED, current_rel, desired_rel1, desired_translation);

    setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);

    setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);
  }

  /* ------------------------------------------------------------------------- */
  /* ----------------------------- Debug Methods ----------------------------- */
  /* ------------------------------------------------------------------------- */

  /*
   * Wrapper functions to more easily output values to SmartDashboard
   * cleaner code
   */
  private void printDB(String name, double val) { // print 1 value
    SmartDashboard.putNumber(name, val);
  }

  private void printDB(String name, boolean bool) { // print 1 value
    SmartDashboard.putBoolean(name, bool);
  }

  private void printDB(String name, String s) { // print 1 value
    SmartDashboard.putString(name, s);
  }

  private void printDB(String name, double[] arr) { // print array of values
    String wheel_orientation = "";

    for (int i = 0; i < arr.length; i++) {

      switch (i) {
        case (0):
          wheel_orientation = "WHEEL_FL";
          break;
        case (1):
          wheel_orientation = "WHEEL_FR";
          break;
        case (2):
          wheel_orientation = "WHEEL_BR";
          break;
        case (3):
          wheel_orientation = "WHEEL_BL";
          break;
      }

      SmartDashboard.putNumber(name + "[" + wheel_orientation + "]", arr[i]);

    }
  }

  /* ------------------------------------------------------------------------- */
  /* --------------------------- FRC Robot Methods --------------------------- */
  /* ------------------------------------------------------------------------- */

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // initialize vision
    initVision();

    // initialize PID
    initPID();

    // I2C port
    arduino = new I2C(I2C.Port.kOnboard, 8);

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_Timer.reset();
    currState = autoStates.stopped;
    colorOfInterest = Colors.red;
    // Arm.Gripper.set(Arm.GRIPPER_MIN);
  }

  /** This function is called periodically during autonomous. */
  // @Override
  public void autonomousPeriodic() {

    // Network Tables - ere
    double redArea = redASub.get();
    double redX = redXSub.get();
    double redY = redYSub.get();

    System.out.println(redArea);

  }

  //   double[] current_rel = new double[] {
  //       RM_Encoders.get(WHEEL_FL).getPosition(),
  //       RM_Encoders.get(WHEEL_FR).getPosition(),
  //       RM_Encoders.get(WHEEL_BR).getPosition(),
  //       RM_Encoders.get(WHEEL_BL).getPosition()
  //   };

  //   // Initialize empty arrays for RM and TM setPoint values to be used later by
  //   // setReference
  //   double[] desired_body = new double[] { 0.0, 0.0, 0.0, 0.0 };
  //   double[] desired_rel1 = new double[] { 0.0, 0.0, 0.0, 0.0 };
  //   double[] desired_translation = new double[] { 0.0, 0.0, 0.0, 0.0 };
  //   double[] DESIRED = new double[] { 0, 0, 0, 0 };
  //   double setpoint = 0;
  //   double[] setpointArr = new double[] {setpoint, setpoint, setpoint, setpoint};

  //   // double GripperPosition = Arm.Gripper.get();
  //   // printDB("Gripper Position", GripperPosition);

  //   double centerx = 160;
  //   double x = centerx;
  //   double x_diff = 0;

  //   String tempState = "";
  //   switch (currState) {

  //     case findBlock:

  //       tempState = "findBlock";

  //       switch(colorOfInterest) {
  //         case red:
  //           x = redX;
  //           area = redArea;
  //           break;
          
  //         case green:
  //           // x = greenX;
  //           // area = greenArea;
  //           break;

  //         case yellow:
  //           // x = yellowX;
  //           // area = yellowArea;
  //           break;

  //         default:
  //           break;

  //       }
        
  //       x_diff = Math.abs(centerx - x);

  //       if (area <= C.minSpinThresh) {
  //         spin(DESIRED, desired_body, desired_rel1, desired_translation, current_rel);              
  //       }
  //       else {
  //         currState = autoStates.stopped;
  //       }

  //       break;

  //     case move:

  //       tempState = "move";

  //       System.out.printf("colorOfInterest = %s ------------------------\n", colorOfInterest);
  //       System.out.printf("area = %f ------------------------\n", area);
        
  //       // if not centered
  //       if (x_diff >= C.centerThresh) {
  //         System.out.println("INSIDE NOT CENTERED ----------------------\n");
          
  //         // Centering a red block if it enters into frame
  //         DESIRED[0] = -45;
  //         DESIRED[1] = 45;
  //         DESIRED[2] = 135;
  //         DESIRED[3] = 225;

  //         setpoint = C.vision_kP * (centerx - x) * ((C.Trans_maxRPM / 6) / C.MAX_rad_s);
  //         for (int i = 0; i <=3; i++) {
  //           if (i % 2 == 0) {
  //             setpointArr[i] = -1 * setpoint;
  //           }
  //           else {
  //             setpointArr[i] = setpoint;
  //           }    
  //         }
  //       }

  //       // if centered, move closer or farther away
  //       else {

  //         // if too far away, go closer
  //         if (area <= C.minDistThresh) {
  //           System.out.println("INSIDE TOO FAR ----------------------\n");

  //           DESIRED[0] = 0;
  //           DESIRED[1] = 0;
  //           DESIRED[2] = 0;
  //           DESIRED[3] = 0;

  //           setpoint = 0.25 * (C.Trans_maxRPM / 6);
  //           for (int i = 0; i <=3; i++) {
  //             setpointArr[i] = setpoint;
  //           }
            
  //         }

  //         // if too close, go back
  //         else if ((area > C.maxDistThresh) && x_diff < C.centerThresh) {
  //           System.out.println("INSIDE TOO CLOSE ----------------------\n");

  //           DESIRED[0] = 0;
  //           DESIRED[1] = 0;
  //           DESIRED[2] = 0;
  //           DESIRED[3] = 0;

  //           setpoint = -0.25 * (C.Trans_maxRPM / 6);
  //           for (int i = 0; i <=3; i++) {
  //             setpointArr[i] = setpoint;
  //           }

  //         } 

  //         // otherwise, ideal spot achieved
  //         else {
  //           System.out.println("IDEAL SPOT ACHIEVED ----------------------");

  //           // red-following demo
  //           // stopMotors();
  //           // currState = autoStates.findRed;

  //           // red-green loop demo
  //           // currState = autoStates.findGreen;

  //           // pick-up and transport demo
  //           if (grip_flag == 0) {
  //             // Arm.Gripper.set(Arm.GRIPPER_MAX);
  //             grip_flag = 1;
  //           }
  //           else {
  //             // Arm.Gripper.set(Arm.GRIPPER_MIN);
  //             grip_flag = 0;
  //           }

  //           currState = autoStates.stopped;
  //         }
          
  //       }

  //       apply_offsets(desired_body, DESIRED, current_rel, desired_rel1, desired_translation);

  //       setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);
  //       setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);

  //     break;

  //     case stopped:
  //       tempState = "stopped";
      
  //       stopMotors();

  //     break;
      
  //     case cool:

  //       tempState = "cool";

  //       double time = m_Timer.get();

  //       // Constants
  //       double dist_x = 12.75;
  //       double dist_y = 12.75;
  //       double r = mag(dist_x, dist_y);
        
  //       // assume omega = 0.1 rad/s
  //       // at t=0, theta=0, robot should be moving in the [1, 0] direction
  //       // at t=10*pi/2 seconds, theta=pi/2, robot should be moving in the [0, -1] direction
  //       // at t=10*pi seconds, theta=pi, robot should be moving in the [-1, 0] direction 
  //       double theta = time * cool_omega;

  //       omega_auto = cool_omega;

  //       Vx_auto = Math.cos(theta) * cool_speed;
  //       Vy_auto = -Math.sin(theta) * cool_speed;

  //       // calculate the vector sum of joystick inputs to define each respective wheel module's 
  //       // linear and angular velocities
  //       double[] state = set_vectors(Vx_auto, Vy_auto, omega_auto, dist_x, dist_y, r);

  //       // in case 2, each wheel is set to a distinct angle and speed
  //       for (int j = 0; j <= 3; j++) {
  //         DESIRED[j] = state[j];
  //         setpointArr[j] = state[j + 4];
  //       }

  //       apply_offsets(desired_body, DESIRED, current_rel, desired_rel1, desired_translation);

  //       setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);
  //       setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);

  //     break;

  //     default:
  //       printDB("STATE", tempState);
  //       break;

  //   }


  // }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    double[] current_rel = new double[] {
        RM_Encoders.get(WHEEL_FL).getPosition(),
        RM_Encoders.get(WHEEL_FR).getPosition(),
        RM_Encoders.get(WHEEL_BR).getPosition(),
        RM_Encoders.get(WHEEL_BL).getPosition()
    };

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

    // Initialize empty arrays for RM and TM setPoint values to be used later by
    // setReference
    double[] desired_body = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] desired_rel1 = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] desired_translation = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] DESIRED = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] setpointArr = new double[] { 0.0, 0.0, 0.0, 0.0 };

    // Constants
    double x = 12.75; // distance from chassis center to module (in) - x-component
    double y = 12.75; // distance from chassis center to module (in) - y-component
    double r = mag(x, y); // needed to make x,y dimensionless

    boolean SqPressed = PS4joystick.getSquareButtonPressed();
    boolean SqReleased = PS4joystick.getSquareButtonReleased();
    closeGrip(SqPressed, SqReleased);

    double GripperPosition = Arm.Gripper.get();

    double GripperAngle = Arm.Gripper.getAngle();

    // System.out.println("----------------------------------------");
    // System.out.printf("GripperPosition = %f \n", GripperPosition);
    System.out.println("----------------------------------------");
    System.out.printf("GripperAngle = %f \n", GripperAngle);
    
    // If no input, command everything to zero
    if (Vx == 0 & Vy == 0 & omega == 0) {
      stopMotors();
    }
    else {
      // calculate the vector sum of joystick inputs to define each respective wheel module's 
      // linear and angular velocities
      double[] state = set_vectors(Vx, Vy, omega, x, y, r);

      // store calculated wheel angles and speeds
      for (int i = 0; i <= 3; i++) {
        DESIRED[i] = state[i];
        setpointArr[i] = state[i + 4];
      }

      // apply pre-defined wheel offset values to get correct values
      apply_offsets(desired_body, DESIRED, current_rel, desired_rel1, desired_translation);

      // command wheel modules to the desired wheel angles and speeds
      setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);
      setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);
      
    }

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    // m_Timer.reset();
    // Arm.Gripper.setAngle(180);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    // double GripperAngle = Arm.Gripper.getAngle();

    // System.out.println("----------------------------------------");
    // System.out.printf("GripperAngle = %f \n", GripperAngle);

    // // boolean SqPressed = PS4joystick.getSquareButtonPressed();
    // // boolean SqReleased = PS4joystick.getSquareButtonReleased();
    // // closeGrip(SqPressed, SqReleased);

    


    

  }
}