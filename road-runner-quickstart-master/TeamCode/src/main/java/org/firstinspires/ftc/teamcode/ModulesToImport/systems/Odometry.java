package org.firstinspires.ftc.teamcode.ModulesToImport.systems;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry {
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder horizontalEncoder;

    private final double ticksPerInch;
    private final double wheelBaseWidth;
    private final double horizontalOffset;

    private double x = 0.0;
    private double y = 0.0;
    private double heading = 0.0;

    private int leftEncoderStart = 0;
    private int rightEncoderStart = 0;
    private int horizontalEncoderStart = 0;

    public Odometry(HardwareMap hardwareMap, double ticksPerInch, double wheelBaseWidth, double horizontalOffset) {
        leftEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "left_encoder")));
        rightEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "right_encoder")));
        horizontalEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "horizontal_encoder")));

        rightEncoder.setDirection(DcMotorEx.Direction.REVERSE);

        this.ticksPerInch = ticksPerInch;
        this.wheelBaseWidth = wheelBaseWidth;
        this.horizontalOffset = horizontalOffset;
    }

    public void update() {
        // Integration for displacement calculation
        double deltaX = evaluateCos(0.001, 0, 1, 0); // x displacement
        double deltaY = evaluateSin(0.001, 0, 1, 0); // y displacement

        // Update global position
        x += deltaX;
        y += deltaY;
    }

    private double fcos(double t) {
        // Calculate velocity * cos(heading) at time t
        double velocity = calculateVelocity(t);
        double heading = calculateHeading(t);
        return velocity * Math.cos(heading);
    }

    private double fsin(double t) {
        // Calculate velocity * sin(heading) at time t
        double velocity = calculateVelocity(t);
        double heading = calculateHeading(t);
        return velocity * Math.sin(heading);
    }

    private double evaluateCos(double eps, double t1, double t2, int level) {
        // Adaptive quadrature for velocity * cos(heading)
        double s1 = (t2 - t1) * (fcos(t1) + 4.0 * fcos((t1 + t2) / 2.0) + fcos(t2)) / 6.0;
        double s2 = (t2 - t1) * (fcos(t1) + 4.0 * fcos(0.75 * t1 + 0.25 * t2) + 2.0 * fcos(0.5 * t1 + 0.5 * t2)
                + 4.0 * fcos(0.25 * t1 + 0.75 * t2) + fcos(t2)) / 12.0;
        if (Math.abs(s2 - s1) <= eps || level > 10) {
            return s2;
        }
        return evaluateCos(eps / 2.0, t1, (t1 + t2) / 2.0, level + 1) +
                evaluateCos(eps / 2.0, (t1 + t2) / 2.0, t2, level + 1);
    }

    private double evaluateSin(double eps, double t1, double t2, int level) {
        // Adaptive quadrature for velocity * sin(heading)
        double s1 = (t2 - t1) * (fsin(t1) + 4.0 * fsin((t1 + t2) / 2.0) + fsin(t2)) / 6.0;
        double s2 = (t2 - t1) * (fsin(t1) + 4.0 * fsin(0.75 * t1 + 0.25 * t2) + 2.0 * fsin(0.5 * t1 + 0.5 * t2)
                + 4.0 * fsin(0.25 * t1 + 0.75 * t2) + fsin(t2)) / 12.0;
        if (Math.abs(s2 - s1) <= eps || level > 10) {
            return s2;
        }
        return evaluateSin(eps / 2.0, t1, (t1 + t2) / 2.0, level + 1) +
                evaluateSin(eps / 2.0, (t1 + t2) / 2.0, t2, level + 1);
    }

    private double calculateVelocity(double t) {
        // Compute velocity from encoders
        int leftVelocity = leftEncoder.getPositionAndVelocity().velocity;
        int rightVelocity = rightEncoder.getPositionAndVelocity().velocity;
        return (leftVelocity + rightVelocity) / 2.0 / ticksPerInch;
    }

    private double calculateHeading(double t) {
        // Compute heading from encoders
        int leftPosition = leftEncoder.getPositionAndVelocity().position;
        int rightPosition = rightEncoder.getPositionAndVelocity().position;
        return (rightPosition - leftPosition) / wheelBaseWidth / ticksPerInch;
    }

    public void resetPose(double startX, double startY, double startHeading) {
        // Set starting positions
        leftEncoderStart = leftEncoder.getPositionAndVelocity().position;
        rightEncoderStart = rightEncoder.getPositionAndVelocity().position;
        horizontalEncoderStart = horizontalEncoder.getPositionAndVelocity().position;

        // Set global pose
        x = startX;
        y = startY;
        heading = startHeading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
