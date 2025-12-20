package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.FeederCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSmartSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Feeder;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//@Disabled
@Autonomous(name="AutoBlueClose" ,preselectTeleOp = "MainTeleOp", group="Autonomous")
public class AutoBlueClose extends CommandOpMode {
    Robot robot = Robot.getInstance();

    private ElapsedTime timer;
    private final Pose startPose = new Pose(18, 115.5, Math.toRadians(0));
    private final Pose artifactsPPGPose = new Pose(56, 86, Math.toRadians(180));
    private final Pose collectPPGPose = new Pose(20, 86, Math.toRadians(180));
    private final Pose shootClosePose = new Pose(60, 78, Math.toRadians(-45));
    private final Pose artifactPGPPose = new Pose(56, 60, Math.toRadians(180));
    private final Pose collectPGPPose = new Pose(20, 60, Math.toRadians(180));



    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;

    public void buildPaths() {
        robot.follower.setStartingPose(startPose);


        path1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootClosePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootClosePose.getHeading())
                .build();

        path2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, artifactsPPGPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        path3= robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactsPPGPose, collectPPGPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        path4 = robot.follower.pathBuilder()
                .addPath(new BezierLine(collectPPGPose, shootClosePose))
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .build();
        path5 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, collectPPGPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        path6 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, artifactPGPPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        path7 = robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactPGPPose, collectPGPPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        path8 = robot.follower.pathBuilder()
                .addPath(new BezierLine(collectPGPPose ,shootClosePose))
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

        schedule(
                new ParallelCommandGroup(
                        new DetectArtifactCommand(robot.rgbLight, robot.colorMatch), // robot.shooter),
                        new VisionCommand(robot.vision),

                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, path1, true),
                                        new ShooterSpinUpCommand(robot.shooter, 3400)
                                ),

                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 4000),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 4000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  4000)
                                ),
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, path2, true),
                                        new ShooterSpinUpCommand(robot.shooter,0.0),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 250),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,250)
                                ),
                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000),
                                        new FollowPathCommand( robot.follower, path3, true)
                                ),
                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,  250),
                                        new FollowPathCommand(robot.follower, path4, true),
                                        new ShooterSpinUpCommand(robot.shooter, 3400) //TODO:  Set this

                                ),

                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 3000),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 3000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  3000)
                                ),
//                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
//                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,  250),
//                                        new FollowPathCommand(robot.follower, path3, true)
//                                ),
//                                new ShooterSpinUpCommand(robot.shooter, 4200),  //TODO:  Set this
//                                //                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 750),
//                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 2500),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2500),
//                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2500)
//                                ),
                                new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                new IntakeCommand(robot.intake, Intake.MotorState.STOP,  250),
                                new ShooterSpinUpCommand(robot.shooter, 0.0),
                                new FollowPathCommand(robot.follower, path6, true),
                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000),
                                        new FollowPathCommand( robot.follower, path7, true)
                                ),

                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,  250),
                                        new FollowPathCommand(robot.follower, path8 , true),
                                        new ShooterSpinUpCommand(robot.shooter, 3400) //TODO:  Set this

                                ),

                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 3000),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 3000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  3000)
                                ),
                                new FollowPathCommand(robot.follower, path5, true)
                        )
                )
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
        if (tag != null) {
            telemetry.addLine("Target Tag Detected!");
            telemetry.addData("ID", tag.id);
            telemetry.addData("Center", "(%.0f, %.0f)", tag.center.x, tag.center.y);
            telemetry.addData("Range (in)", "%.1f", tag.ftcPose.range);
        } else {
            telemetry.addLine("No target tags (20–24) detected.");
        }
        telemetry.update();
    }

    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }


}

