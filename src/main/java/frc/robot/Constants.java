package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Rect;

public class Constants {


    // --------------------------------- Swerve Constants --------------------------------- \\

    public double rotConvFactor = 5.62279270244;
    public double transConvFactor = 0.1875; // RPM
    public double MAX_rad_s = 1.531707;
    public double MAX_m_s = 0.635617;

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
    public double Trans_kI = 0;
    public double Trans_kD = 0;
    public double Trans_kIz = 0;
    public double Trans_kFF = 0;
    public double Trans_kMaxOutput = 1;
    public double Trans_kMinOutput = -1;
    public double Trans_maxRPM = 5500;

    // encoder offset variables, should have two arrays of length 4:
    // alpha to store abs offsets, delta to store rel offsets, alpha be final
    public final double[] alpha_Motor = new double[] { 14.589844, 51.064453, 117.949219, 124.365234 };

    // --------------------------------- Vision Constants --------------------------------- \\
    public Size gb = new Size(7, 7); // gaussian blur
    public final Rect EMPTY_RECT = new Rect(0, 0, 0, 0);

    public final int MAX_OBJECT_DETECT = 5;
    public final boolean VERBOSE = false;
    // colors
    public final Scalar RED_COLOR = new Scalar(0, 0, 255);
    public final Scalar YELLOW_COLOR = new Scalar(0, 255, 255);
    public final Scalar GREEN_COLOR = new Scalar(0, 255, 0);
    public final Scalar BLUE_COLOR = new Scalar(255, 0, 0);
    public final Scalar PINK_COLOR = new Scalar(255, 0, 255);
    public final Scalar WHITE_COLOR = new Scalar(255, 255, 255);

    // BGR thresholding values
    public Scalar redLower = new Scalar(0, 0, 55);
    public Scalar redUpper = new Scalar(50, 35, 255);

    public Scalar yellowLower = new Scalar(0, 160, 175);
    public Scalar yellowUpper = new Scalar(40, 210, 255);
    // bad: 44, 152, 216
    // good: 29, 196, 255

    public Scalar greenLower = new Scalar(0, 100, 0);
    public Scalar greenUpper = new Scalar(150, 255, 50);

    
    // --------------------------------- Autonomous Constants --------------------------------- \\

    public double minSpinThresh = 1000;
    public double centerThresh = 20;
    public double minDistThresh = 70500;
    public double maxDistThresh = 72000;
    public double minyellowDistThresh = 22000;
    public double maxyellowDistThresh = 23500;

    public double vision_kP = 0.0045;

}
