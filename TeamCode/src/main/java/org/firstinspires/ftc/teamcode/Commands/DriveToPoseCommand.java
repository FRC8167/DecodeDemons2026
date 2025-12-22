package org.firstinspires.ftc.teamcode.Commands;


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.Robot;


public class DriveToPoseCommand extends CommandBase {

    private final Pose targetPose; // target pose including heading
    private final Robot robot;
    private final GamepadEx driver;


    public DriveToPoseCommand(Pose targetPose, GamepadEx driver)
    {
        robot = Robot.getInstance();
        this.targetPose = targetPose;
        this.driver = driver;
//        addRequirements(robot.mecanumDrive);

    }

    @Override
    public void initialize() {

        Pose currentPose = robot.follower.getPose();

        PathChain pathToShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(),
                        targetPose.getHeading()
                )
                .build();

        robot.follower.followPath(pathToShoot);

//        Pose currentPose = robot.follower.getPose();
//
//         PathChain pathToShoot = robot.follower.pathBuilder()
//                .addPath(new BezierLine(currentPose, targetPose))
//                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
//                .build();
//
//        robot.follower.followPath(pathToShoot);

    }

    @Override
    public void execute() {

    }

    @Override

    public boolean isFinished() {

        boolean driverOverride =
                Math.abs(driver.getLeftY()) > 0.1 ||
                        Math.abs(driver.getLeftX()) > 0.1 ||
                        Math.abs(driver.getRightX()) > 0.1;

        return !robot.follower.isBusy() || driverOverride;
    }
//            Pose current = robot.follower.getPose();
//            double distanceXError = Math.abs(current.getX() - targetPose.getX());
//            double distanceYError = Math.abs(current.getY() - targetPose.getY());
//            double headingError = Math.abs(current.getHeading() - targetPose.getHeading());
//
//            // Tolerances: 1 inch, 3 degrees
//            // Driver override: joystick magnitude > 0.1
//        boolean driverOverride = Math.abs(driver.getLeftY()) > 0.1  ||
//                Math.abs(driver.getLeftX()) > 0.1  ||
//                Math.abs(driver.getRightX()) > 0.1;
//
//        return (distanceXError < 2.0 && distanceYError < 2.0 && headingError < Math.toRadians(3))
//                || driverOverride;        }

    @Override
    public void end(boolean interrupted) {
        robot.follower.breakFollowing();
    }





}
