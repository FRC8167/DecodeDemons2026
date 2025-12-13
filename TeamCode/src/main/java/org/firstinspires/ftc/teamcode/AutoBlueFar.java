package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSmartSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Feeder;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//@Disabled
@Autonomous(name="AutoBlueFar") //, preselectTeleOp = "TeleOpMode", group="Name of Group")
public class AutoBlueFar extends CommandOpMode {
    Robot robot = Robot.getInstance();

    private ElapsedTime timer;
    private final Pose startPose = new Pose(56, 8, Math.toRadians(-70));
    private final Pose artifactsGPPPose = new Pose(56, 36, Math.toRadians(180));
    private final Pose collectGPPPose = new Pose(14, 36, Math.toRadians(180));
    private final Pose shootFarPose = new Pose(56, 8, Math.toRadians(-70));


    private PathChain path1, path2, path3;

    public void buildPaths() {
        robot.follower.setStartingPose(startPose);


        path1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, artifactsGPPPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), artifactsGPPPose.getHeading())
                .build();

        path2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactsGPPPose, collectGPPPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        path3= robot.follower.pathBuilder()
                .addPath(new BezierLine(collectGPPPose, shootFarPose))
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .build();
    }

    public void initialize() {
        Robot.OP_MODE_TYPE = Robot.OpModeType.AUTO;

        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        try {
            robot.init(hardwareMap);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        buildPaths();

//        schedule(new DetectArtifactCommand(robot.rgbLight, robot.colorMatch, robot.shooter));
//        schedule(new VisionCommand(robot.vision));
//        schedule(
//                new SequentialCommandGroup(
//
//                        new ShooterSpinUpCommand(robot.shooter, 3900)
//                        new InstantCommand(()-> robot.feederF.feed(Feeder.FeederState.FORWARD)),
//                        new WaitCommand(1000),
//                        new InstantCommand(()-> robot.feederR.feed(Feeder.FeederState.FORWARD)),
//                        new WaitCommand(750),
//                        new InstantCommand(robot.intake::forward)
//                        ),
//                        new ParallelCommandGroup(
//                                new ShooterSpinUpCommand(robot.shooter,0.0),
//                                new InstantCommand(robot.feederR::off),
//                                new InstantCommand(robot.feederF::off),
//                                new InstantCommand(robot.intake::off)
//                        ),
//                        new FollowPathCommand(robot.follower, path1, true),
//                        new ParallelCommandGroup(
//                                new InstantCommand(()-> robot.feederF.feed(Feeder.FeederState.FORWARD)),
//                                new InstantCommand(robot.intake::forward),
//                                new FollowPathCommand( robot.follower, path2, true)
//                        ),
//                        new ParallelCommandGroup(
//                                new InstantCommand(robot.feederR::off),
//                                new InstantCommand(robot.feederF::off),
//                                new InstantCommand(robot.intake::off)
//
//                        ),
//                        new FollowPathCommand(robot.follower, path3, true),
//                        new ShooterSmartSpinUpCommand(robot.shooter, robot.vision),
//                        new InstantCommand(()-> robot.feederF.feed(Feeder.FeederState.FORWARD)),
//                        new WaitCommand(1000),
//                        new InstantCommand(()-> robot.feederR.feed(Feeder.FeederState.FORWARD)),
//                        new WaitCommand(750),
//                        new InstantCommand(robot.intake::forward)
//                        );
//    }

schedule(
        new SequentialCommandGroup(
                new ShooterSpinUpCommand(robot.shooter, 3900),
                new FollowPathCommand(robot.follower, path1, true)),
                new InstantCommand(robot.intake::forward),
                new ShooterSpinUpCommand(robot.shooter, 0)
        );
    }

    @Override
    public void run() {
        super.run();
        AprilTagDetection tag = robot.vision.getFirstTargetTag();
        robot.follower.update();
        robot.follower.getPose();
        telemetry.addData("X:  ", robot.follower.getPose().getX());
        telemetry.addData("Y:  ", robot.follower.getPose().getY());
        telemetry.addData("Theta:  ", robot.follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }


}

