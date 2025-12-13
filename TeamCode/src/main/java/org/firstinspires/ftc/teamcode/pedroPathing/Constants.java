package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    //mass of 15 is a placeholder
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-35.198)
            .lateralZeroPowerAcceleration(-61.011)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.4, 0, .03, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0.0, 0.0, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.006, 0.0, .0006, 0.6, 0.01))
            .centripetalScaling(.0005)
            .mass(8.0);  //TODO weigh the robot


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);




    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(63.487)
            .yVelocity(53.791)
            .rightFrontMotorName("RightFront")  //TODO check these names                                             c
            .rightRearMotorName("RightRear")
            .leftRearMotorName("LeftRear")
            .leftFrontMotorName("LeftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.0)  //TODO
            .strafePodX(-6.5)  //TODO
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")  //change name?
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)  //TODO verify or REVERSE
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);  //TODO verify or REVERSE
}
