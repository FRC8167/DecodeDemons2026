package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

/**
 * MecanumDrive Subsystem
 * Handles low-level motor control and odometry updates.
 * Operator input handled in a separate DriveCommand.
 */
public class MecanumDrive extends SubsystemBase {

    private final MotorEx frontLeft, backLeft, frontRight, backRight;

    private double controlAuthority;
    private final double DEGRADE_AUTHORITY = 0.35;
    private final double MAX_AUTHORITY = 0.85;


    /**
     *
     * @param leftFront
     * @param leftRear
     * @param rightFront
     * @param rightRear
     */
    public MecanumDrive(MotorEx leftFront, MotorEx leftRear, MotorEx rightFront, MotorEx rightRear) {

        this.frontLeft = leftFront;
        this.backLeft = leftRear;
        this.frontRight = rightFront;
        this.backRight = rightRear;

        // Set motor directions
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        frontRight.setInverted(false);
        backRight.setInverted(false);
        ;

        frontRight.setRunMode(Motor.RunMode.RawPower);
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        controlAuthority = MAX_AUTHORITY;

        // Initialize motors
        setMotorPower(0, 0, 0, 0);
    }


    /**
     * Apply mecanum powers
     *
     * @param driveCmd  forward/back
     * @param strafeCmd left/right
     * @param turnCmd   rotation
     */
    public void drive(double driveCmd, double strafeCmd, double turnCmd) {

        double drive = driveCmd * controlAuthority;
        double strafe = strafeCmd * controlAuthority;
        double turn = turnCmd * controlAuthority;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);

        double frontLeftPower = (drive + strafe + turn) / denominator;
        double backLeftPower = (drive - strafe + turn) / denominator;
        double frontRightPower = (drive - strafe - turn) / denominator;
        double backRightPower = (drive + strafe - turn) / denominator;

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    /**
     * Drive with a constant heading
     *
     * @param driveCmd
     * @param strafeCmd
     * @param turnCmd
     * @param currentHeading
     * @param headingDeg
     */
    public void driveWithHeading(double driveCmd, double strafeCmd, double turnCmd, double currentHeading, double headingDeg) {

        double error, gain, newTurnCmd;
        double headingCourseGain = 0.1;
        double headingFineGain = 0.05;

        error = headingDeg - currentHeading;
        /* Need Angle Wrap calculation to ensure turning the shortest distance */
        if (error > 10) {
            newTurnCmd = Range.clip(headingCourseGain * error, -1.0, 1.0);
        } else {
            newTurnCmd = Range.clip(headingFineGain * error, -1.0, 1.0);
        }

        drive(driveCmd, strafeCmd, newTurnCmd);
    }


    /**
     * his routine drives the robot field relative
     *
     * @param forward
     * @param right
     * @param rotate
     * @param currentHeading
     */
    public void driveFieldRelative(double forward, double right, double rotate, double currentHeading) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - currentHeading);
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }


    /**
     * Low-level method: set motor power directly
     */
    public void setMotorPower(double lf, double rf, double lr, double rr) {
        frontLeft.set(lf);
        frontRight.set(rf);
        backLeft.set(lr);
        backRight.set(rr);
    }


    @Override
    public void periodic() {

    }


    public void enableSnailDrive() {
        controlAuthority = DEGRADE_AUTHORITY;
    }

    public void disableSnailDrive() {
        controlAuthority = MAX_AUTHORITY;
    }

    public double getControlAuthority() {
        return controlAuthority;
    }

}
