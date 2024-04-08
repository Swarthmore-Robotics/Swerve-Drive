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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Point;
import java.util.List;
import java.util.ArrayList;

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

  // offset value array for going from abs to relative, changes on startup
  public double[] delta_Motor = new double[] { 0.0, 0.0, 0.0, 0.0 };

  // Vision Variables
  private static Vision cv;
  private static Constants C = new Constants();

  enum autoStates {
    findRed, 
    findGreen,
    move,
    stopped,
    cool
  }

  private autoStates currState;

  enum Colors {
    red,
    yellow,
    green,
    none
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
   * Sets the desired wheel angles and direction of translation for each wheel
   * module, per case
   */
  private void set_desired(double[] desired_body, double[] DESIRED, double[] current_rel, double[] desired_rel1,
      double[] desired_translation, int CASE) {

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

    desired_translation[WHEEL_FL] = res1[1];
    desired_translation[WHEEL_FR] = res2[1];
    desired_translation[WHEEL_BR] = res3[1];
    desired_translation[WHEEL_BL] = res4[1];

    // No rotation, CASE 1
    if (CASE == 1 || CASE == 2) {

      // Translation results (whether to flip or not)
      desired_translation[WHEEL_FL] = -1 * res1[1];
      desired_translation[WHEEL_FR] = res2[1];
      desired_translation[WHEEL_BR] = -1 * res3[1];
      desired_translation[WHEEL_BL] = res4[1];
    }

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
    cv = cv.getInstance();
    cv.startVision();
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
    double[] setpointArr = new double[] {setpoint, setpoint, setpoint, setpoint};

    set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 3);

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
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_Timer.reset();
    // m_Timer.start();
    currState = autoStates.findRed;
    colorOfInterest = Colors.none;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    double[] current_abs = new double[] {
        coders[WHEEL_FL].getAbsolutePosition(),
        coders[WHEEL_FR].getAbsolutePosition(),
        coders[WHEEL_BR].getAbsolutePosition(),
        coders[WHEEL_BL].getAbsolutePosition()
    };
    double[] current_rel = new double[] {
        RM_Encoders.get(WHEEL_FL).getPosition(),
        RM_Encoders.get(WHEEL_FR).getPosition(),
        RM_Encoders.get(WHEEL_BR).getPosition(),
        RM_Encoders.get(WHEEL_BL).getPosition()
    };

    // Print absolute CANCoder values
    printDB("CANCODER", current_abs);

    // Print relative Encoder values
    printDB("RM_Encoder.getPosition()", current_rel);

    // Initialize empty arrays for RM and TM setPoint values to be used later by
    // setReference
    double[] desired_body = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] desired_rel1 = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] desired_translation = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] DESIRED = new double[] { 0, 0, 0, 0 };

    Point center = cv.rectCenter(cv.biggestRed);
    double x = center.x;
    double centerx = (double) cv.imgWidth / 2;
    double x_diff = Math.abs(centerx - x);
    
    switch (currState) {

      case findRed:
        System.out.println("currState = findRed ------------------------");
      
        colorOfInterest = Colors.red;

        // if nothing of (RED) color detected, keep spinning around until detected
        if (cv.biggestRed.area() <= C.minSpinThresh) {
          spin(DESIRED, desired_body, desired_rel1, desired_translation, current_rel);
        }
        else {
          currState = autoStates.move;
        }
        break;

      case findGreen:
        System.out.println("currState = findGreen ------------------------");
        
        colorOfInterest = Colors.green;

        // if nothing of (GREEN) color detected, keep spinning around until detected
        if (cv.biggestGreen.area() <= C.minSpinThresh) {
          spin(DESIRED, desired_body, desired_rel1, desired_translation, current_rel);
        }
        else {
          currState = autoStates.move;
        }
        break;

      case move:

        System.out.println("currState = move ------------------------");
        System.out.printf("colorOfInterest = %s ------------------------\n", colorOfInterest);
        System.out.printf("area = %f ------------------------\n", area);
      
          
        if (colorOfInterest == Colors.red) {
          area = cv.biggestRed.area();
        }
        else if (colorOfInterest == Colors.green) {
          area = cv.biggestGreen.area();
        }
        
        // if centered, and too far away, go closer
        if ((area <= C.minDistThresh) && x_diff < C.centerThresh) {
          System.out.println("INSIDE TOO FAR ----------------------\n");

          DESIRED[0] = 0;
          DESIRED[1] = 0;
          DESIRED[2] = 0;
          DESIRED[3] = 0;

          double tempSetPoint = 0.25 * (C.Trans_maxRPM / 6);
          double[] setpointArr = new double[] {tempSetPoint, tempSetPoint, tempSetPoint, tempSetPoint};

          set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);

          setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);

          setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);
          
        }

        // else, if centered and too close go back
        else if ((area > C.maxDistThresh) && x_diff < C.centerThresh) {
          System.out.println("INSIDE TOO CLOSE ----------------------\n");

          DESIRED[0] = 0;
          DESIRED[1] = 0;
          DESIRED[2] = 0;
          DESIRED[3] = 0;

          double tempSetPoint = -0.25 * (C.Trans_maxRPM / 6);
          double[] setpointArr = new double[] {tempSetPoint, tempSetPoint, tempSetPoint, tempSetPoint};

          set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);

          setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);

          setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);

        } 

        else if (x_diff >= C.centerThresh) {
          System.out.println("INSIDE NOT CENTERED ----------------------\n");
          
          // Centering a red block if it enters into frame
          DESIRED[0] = -45;
          DESIRED[1] = 45;
          DESIRED[2] = 135;
          DESIRED[3] = 225;

          double setpoint = C.vision_kP * (centerx - x) * ((C.Trans_maxRPM / 6) / C.MAX_rad_s);
          double[] setpointArr = new double[] {setpoint, setpoint, setpoint, setpoint};

          set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 3);

          setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);

          setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);

        } 

        else {
          System.out.println("IDEAL SPOT ACHIEVED ----------------------");

          currState = autoStates.findGreen;
        }

        break;

      case stopped:

        System.out.println("currState = stopped ------------------------");
      
        stopMotors();
        break;
      
      case cool:
        spin(DESIRED, desired_body, desired_rel1, desired_translation, current_rel);

        m_Timer.start();

        if (m_Timer.get() > 1) {
          // Rotate 180deg + Translate 6ft in 2.877s
          
        }
      break;

      default:
        break;

    }

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    double[] current_abs = new double[] {
        coders[WHEEL_FL].getAbsolutePosition(),
        coders[WHEEL_FR].getAbsolutePosition(),
        coders[WHEEL_BR].getAbsolutePosition(),
        coders[WHEEL_BL].getAbsolutePosition()
    };
    double[] current_rel = new double[] {
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

    // Initialize empty arrays for RM and TM setPoint values to be used later by
    // setReference
    double[] desired_body = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] desired_rel1 = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] desired_translation = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] DESIRED = new double[] { 0.0, 0.0, 0.0, 0.0 };
    double[] setpointArr = new double[] { 0.0, 0.0, 0.0, 0.0 };

    // Constants
    double x = 12.75; // distance from chassis center to module - x-component
    double y = 12.75; // distance from chassis center to module - y-component
    double r = mag(x, y); // needed to make x,y dimensionless

    // No rotation, CASE 1
    if (omega == 0 & (Vx != 0 | Vy != 0)) {
      // define wheel angle based on direction of joystick vectors, Vx and Vy
      double setAngle = ((Math.atan2(Vy, Vx) * 180) / Math.PI);
      
      // define wheel speeds to be magnitude of joystick input * arbitrary 1/6th of max wheel RPM
      double setpoint = Vr * (C.Trans_maxRPM / 6);

      // in case 1, set every wheel to the same desired setAngle and setpoint
      for (int i = 0; i <= 3; i++) {
        DESIRED[i] = setAngle;
        setpointArr[i] = setpoint;
      }

      // apply pre-defined wheel offset values to get correct values
      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 1);

      // command wheel modules to the desired wheel angles and speeds
      setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);
      setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);

    }

    // General Case, CASE 2
    else if (omega != 0 & (Vx != 0 | Vy != 0)) {

      // calculate the vector sum of joystick inputs to define each respective wheel module's 
      // linear and angular velocities
      double[] state = set_vectors(Vx, Vy, omega, x, y, r);

      // in case 2, each wheel is set to a distinct angle and speed
      for (int i = 0; i <= 3; i++) {
        DESIRED[i] = state[i];
        setpointArr[i] = state[i + 4];
      }

      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 2);

      setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);
      setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);

    }

    // Rotate in place, CASE 3
    else if (Vx == 0.0 & Vy == 0.0 & omega != 0) {

      double setpoint = omega * (C.Trans_maxRPM / 6);

      // in case 3, set each wheel angle to some 90 degree offset of each other
      for (int i = 0; i <= 3; i++) {
        DESIRED[i] = -45 + (90 * i);
        setpointArr[i] = setpoint;
      }

      set_desired(desired_body, DESIRED, current_rel, desired_rel1, desired_translation, 3);

      setWheelState(RM_PIDControllers, desired_rel1, true, setpointArr);
      setWheelState(TM_PIDControllers, desired_translation, false, setpointArr);

    }

    // No input, CASE 4
    else {
      // command everything to zero
      stopMotors();
    }

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}