package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class Arm {

    // Servos that move the arm
    // private final Servo Lifter = new Servo(0);
    public static final Servo Gripper = new Servo(0);

    // Constants for arm range of motion
    public static final double GRIPPER_MIN = 0.15;
    public static final double GRIPPER_MAX = 1;
    public static final double GRIPPER_INIT = 0.30;

    // Current positions for each servo
    // private double gripperPosition = GRIPPER_INIT;
    // private double lifterAngle;

    public void setGripper(double position) {
        // gripperPosition = MathUtil.clamp(position, GRIPPER_MIN, GRIPPER_MAX);
        Gripper.set(position);
    }

    // public void setLifter(double angle) {
    //     lifterAngle = MathUtil.clamp(angle, GRIPPER_MIN, GRIPPER_MAX);
    //     Lifter.set(angle);
    // }


}
