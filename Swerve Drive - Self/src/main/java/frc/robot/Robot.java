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
// import edu.wpi.first.math.geometry.Rotation2d;

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

  private Thread visionThread;
  private final int imgWidth = 640;
  private final int imgHeight = 480;
  
  public final double[] offset1 = new double[]{14.589844, -165.585938};
  public final double[] offset2 = new double[]{51.064453, -130.253906};
  public final double[] offset3 = new double[]{117.949219, -60.292969};
  // public final double[] offset4 = new double[]{117.949219, -60.292969};

  public final double[] pinkoffset1 = new double[]{0,0};
  public final double[] pinkoffset2 = new double[]{0,0};
  public final double[] pinkoffset3 = new double[]{-90.500122, 89.499878};
  public final double[] pinkoffset4 = new double[]{0,0};

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
    visionThread = new Thread(() -> {
      // add USB camera, create server for SmartDashboard
      UsbCamera usbCamera = CameraServer.startAutomaticCapture("Main Camera", 0);
      // UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
      usbCamera.setResolution(imgWidth, imgHeight);

      CvSink cvSink = CameraServer.getVideo(); // grab images from camera
      CvSource outputStream = CameraServer.putVideo("Processed Image", imgWidth, imgHeight);

      //cvSink.setSource(usbCamera);
      Point upleft = new Point(0, 0);
      Point downright = new Point(200, 200);
      Scalar color = new Scalar(255, 255, 255);
      Mat sourceMat = new Mat();
      
      while(true){
        if (cvSink.grabFrame(sourceMat) == 0) {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          // skip the rest of the current iteration
          continue;
        }
        Imgproc.cvtColor(sourceMat, sourceMat, Imgproc.COLOR_BGR2GRAY);
        Imgproc.rectangle(sourceMat, upleft, downright, color, -1, 8, 0);
        outputStream.putFrame(sourceMat);
        SmartDashboard.putNumber("Image Mat Height", sourceMat.height());
        SmartDashboard.putNumber("Image Mat Width", sourceMat.width());
        SmartDashboard.putNumber("Image Mat Dims", sourceMat.dims());
      }

    });

    visionThread.setDaemon(true);
    visionThread.start();

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

    // double cancodervalue3 = coder3.getAbsolutePosition();
    // double newval3 = wrapEncoderValues(cancodervalue3 + offset3[0]);
    // if (cancodervalue3 >= 0) {
    //   pinkoffset3[0] = -1 * (cancodervalue3 - (-1* (offset3[1])));
    //   pinkoffset3[1] = 180 + pinkoffset3[0];
    //   // pinkoffset3[1] = cancodervalue3 - offset3[0];
    // }
    // else {
    //   pinkoffset3[0] = cancodervalue3 - (offset3[0]);
    //   pinkoffset3[1] = -180 - pinkoffset3[0];
    // }

    // SmartDashboard.putNumber("pinkoffset3[0]", pinkoffset3[0]);
    // SmartDashboard.putNumber("pinkoffset3[1]", pinkoffset3[1]);


    // PID coefficients, RM2
    kP = 5e-5;
    // kP = 0; 
    kI = 1e-8;
    // kI = 0;
    // kD = 1e-7; 
    kD = 0;
    // kIz = 1e-4;
    kIz = 0; 
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
    
    // // display PID coefficients on SmartDashboard
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

    // // button to toggle between velocity and smart motion modes
    // SmartDashboard.putBoolean("Mode", true);

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

  // ------------------------------------------------------------------------------

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // Obtain and print absolute and relative encoder values

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
    double x = 12.125;
    double y = 12.125;
    double d;

    // double cancodervalue1 = coder1.getAbsolutePosition();
    // SmartDashboard.putNumber("CANCODER 1", cancodervalue1);
    // double newval1 = wrapEncoderValues(cancodervalue1 + offset1[0]);
    // SmartDashboard.putNumber("newOFFSET", newval1);
    // SmartDashboard.putNumber("RM2_Encoder Value WRAPPED", RM2_Encoder.getPosition());
    // double setValue1 = 0;
    // // double newsetValue1 = wrapEncoderValues(360 - cancodervalue1 - offset1[0]);
    // // RM2_PidController.setReference(2*wrapEncoderValues(setValue1 - newsetValue1), CANSparkMax.ControlType.kSmartMotion);

    // double cancodervalue2 = coder2.getAbsolutePosition();
    // SmartDashboard.putNumber("CANCODER 2", cancodervalue2);
    // double newval2 = wrapEncoderValues(cancodervalue2 + offset2[0]);
    // SmartDashboard.putNumber("newOFFSET", newval2);
    // SmartDashboard.putNumber("RM4_Encoder Value WRAPPED", RM4_Encoder.getPosition());
    // double setValue2 = 0;
    // // double newsetValue2 = wrapEncoderValues(360 - cancodervalue2 - offset2[0]);
    // // RM4_PidController.setReference(2*wrapEncoderValues(setValue2 - newsetValue2), CANSparkMax.ControlType.kSmartMotion);

    double cancodervalue3 = coder3.getAbsolutePosition();
    SmartDashboard.putNumber("CANCODER 3", cancodervalue3);
    double newval3 = wrapEncoderValues(cancodervalue3 + offset3[0]);

    // RM6_Encoder.setPosition(newval3);
    SmartDashboard.putNumber("newOFFSET", newval3);
    
    //  AARON's METHOD
    // double setValue3 = 0;
    // double theta = wrapEncoderValues(360 - cancodervalue3 - offset3[0]);
    
    // double alpha = RM6_Encoder.getPosition() - cancodervalue3;

    // double beta = setValue3 + alpha;
    // SmartDashboard.putNumber("beta", beta);

    // SmartDashboard.putNumber("SetReferenceVal", wrapEncoderValues(setValue3 + theta));
    // RM6_PidController.setReference(wrapEncoderValues(setValue3 + theta), CANSparkMax.ControlType.kSmartMotion);

    // ERE's METHOD
    // double setValue3 = 0;
    // double newPink = -1 * wrapEncoderValues(RM6_Encoder.getPosition());
    // double newPink = -1 * RM6_Encoder.getPosition();
    // SmartDashboard.putNumber("RM6_Encoder Value", newPink);

    // NEW OFFSETS:
    // 151.962891 = PINK ZERO (in CC3)
    // 90.500122 
    // 89.499878


    // double alpha = cancodervalue3 + RM6_Encoder.getPosition();
    // SmartDashboard.putNumber("alpha", alpha);

    // RM6_PidController.setReference((-1*setValue3) - pinkoffset3[0], CANSparkMax.ControlType.kSmartMotion);

    // double alpha = cancodervalue3 - RM6_Encoder.getPosition();
    // double setValue3 = 0;
    // RM6_PidController.setReference(wrapEncoderValues(alpha - offset3[0]), CANSparkMax.ControlType.kSmartMotion);

    // double newsetValue3;

    // if (cancodervalue3 < 0) {
    //   newsetValue3 = wrapEncoderValues(cancodervalue3 - offset3[0]);
    // } 
    // else {
    //   newsetValue3 = wrapEncoderValues(180 - cancodervalue3 + 60.292969);
    // }
    // SmartDashboard.putNumber("newsetValue3", newsetValue3);
    // SmartDashboard.putNumber("setReference3", wrapEncoderValues(setValue3 - newsetValue3));
    // RM6_PidController.setReference(wrapEncoderValues(setValue3 - newsetValue3), CANSparkMax.ControlType.kSmartMotion);
    if (omega == 0 & Vx != 0 & Vy != 0) {
      // No rotation, CASE 1
      SmartDashboard.putNumber("Case", 1);
      double DESIRED = ((Math.atan2(Vy,Vx) * 180) / Math.PI);
      SmartDashboard.putNumber("DESIRED_BEFORE", DESIRED);
      
      // double AngleAtan2 = Math.atan2(Vy,Vx);
      // SmartDashboard.putNumber("AngleAtan2", AngleAtan2);

      SmartDashboard.putNumber("CURRENT_BEFORE", -1 * RM6_Encoder.getPosition() + pinkoffset3[0]);

      // double res1[] = angleDiff(DESIRED, RM2_EncPos);
      // double res2[] = angleDiff(DESIRED, RM4_EncPos);
      double res3[] = angleDiff(DESIRED, (-1 * RM6_Encoder.getPosition()) + pinkoffset3[0]);
      // double res4[] = angleDiff(DESIRED, RM8_EncPos);

      // SmartDashboard.putNumber("Vwheel1", Vr * res1[1]);
      // SmartDashboard.putNumber("SetAngle1", res1[0]);
      // SmartDashboard.putNumber("Vwheel3", Vr * res3[1]);
      SmartDashboard.putNumber("SetAngle3", res3[0]);
      SmartDashboard.putNumber("NEWSetAngle3", res3[0]);

      // Set Rotation Motor Position based on encoders
      // RM2_PidController.setReference(-1 * res1[0], CANSparkMax.ControlType.kSmartMotion);
      // RM2_PidController.setReference(res1[0], CANSparkMax.ControlType.kSmartMotion);
      // RM4_PidController.setReference(res2[0], CANSparkMax.ControlType.kSmartMotion);
      RM6_PidController.setReference(res3[0], CANSparkMax.ControlType.kSmartMotion);
      // RM8_PidController.setReference(res4[0], CANSparkMax.ControlType.kSmartMotion);


      // Then set translation motor speeds

      // translateMotor1.set(Vr * res1[1]);
      // translateMotor3.set(Vr * res2[1]);
      // translateMotor5.set(Vr * res3[1]);
      // translateMotor7.set(Vr * res4[1]); 
    }

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
