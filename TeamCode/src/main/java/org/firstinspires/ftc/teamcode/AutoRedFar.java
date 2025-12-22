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

import org.firstinspires.ftc.teamcode.Cogintilities.MirrorUtility;
import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.FeederCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSmartSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Feeder;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//@Disabled
@Autonomous(name="AutoRedFar", preselectTeleOp = "MainTeleOp", group="Autonomous")
public class AutoRedFar extends CommandOpMode {
    Robot robot = Robot.getInstance();

    private ElapsedTime timer;
    private final Pose startPose = MirrorUtility.mirror(new Pose(64+2, 9,Math.toRadians(-90)));
    private final Pose rotatedPose = MirrorUtility.mirror(new Pose(56, 12, Math.toRadians(-72)));
    private final Pose artifactsGPPPose = MirrorUtility.mirror(new Pose(56, 36.5, Math.toRadians(180)));
    private final Pose collectGPPPose = MirrorUtility.mirror(new Pose(20, 36.5, Math.toRadians(180)));
    private final Pose shootFarPose = MirrorUtility.mirror(new Pose(56, 12, Math.toRadians(-72)));
    private final Pose artifactPGPPose = MirrorUtility.mirror(new Pose(56, 60, Math.toRadians(180)));
    private final Pose collectPGPPose = MirrorUtility.mirror(new Pose(20, 60, Math.toRadians(180)));


    private PathChain rotateToShootPath, shootToGPPSpikePath, eatGPPPath, endGPPToShootPath, shootToPGPSpikePath,
            eatPGPPath, endPGPToShootPath;

    public void buildPaths() {
        robot.follower.setStartingPose(startPose);




        rotateToShootPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, rotatedPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), rotatedPose.getHeading())
                .build();

        shootToGPPSpikePath = robot.follower.pathBuilder()
                .addPath(new BezierLine(rotatedPose, artifactsGPPPose))
                .setLinearHeadingInterpolation(rotatedPose.getHeading(), artifactsGPPPose.getHeading())
                .build();

        eatGPPPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactsGPPPose, collectGPPPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        endGPPToShootPath= robot.follower.pathBuilder()
                .addPath(new BezierLine(collectGPPPose, shootFarPose))
                .setConstantHeadingInterpolation(Math.toRadians(180-(-72)))
                .build();

        shootToPGPSpikePath  =  robot.follower.pathBuilder()
                .addPath(new BezierLine(shootFarPose, artifactPGPPose))
                .setLinearHeadingInterpolation(shootFarPose.getHeading(), artifactPGPPose.getHeading())
                .build();

        eatPGPPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactPGPPose, collectPGPPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        endPGPToShootPath= robot.follower.pathBuilder()
                .addPath(new BezierLine(collectPGPPose, shootFarPose))
                .setConstantHeadingInterpolation(Math.toRadians(180-(-72)))
                .build();

    }

    public void initialize() {
        Robot.OP_MODE_TYPE = Robot.OpModeType.AUTO;
        robot.setAlliance(Robot.Alliance.RED);

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
                        new DetectArtifactCommand(robot.rgbLight, robot.colorMatch),// robot.shooter),
                        new VisionCommand(robot.vision),

                        new SequentialCommandGroup(

                                //rotate and get ready to shoot
                                new ParallelCommandGroup(
                                        new ShooterSpinUpCommand(robot.shooter, 3850),
                                        new FollowPathCommand(robot.follower, rotateToShootPath, true)
                                ),
                                //shoot first ball
                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 2000),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000)
                                ),
                                new WaitCommand(250),
                                //shoot second ball
                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 2000),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000)
                                ),
                                //move to first spike while shutting off intake, feeders, and shooter
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, shootToGPPSpikePath, true),
                                        new ShooterSpinUpCommand(robot.shooter,0.0),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 250),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,250)
                                ),
                                //gobble up the balls on spike1
                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 3000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  3000),
                                        new FollowPathCommand( robot.follower, eatGPPPath, true)
                                ),
                                new ShooterSpinUpCommand(robot.shooter, -1000),
                                new WaitCommand(250),
                                new ShooterSpinUpCommand(robot.shooter, 0.0),
                                //move to shoot
                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,  250),
                                        new FollowPathCommand(robot.follower, endGPPToShootPath, true)
                                ),
                                //get ready to shoot
                                new ShooterSpinUpCommand(robot.shooter, 3850),  //TODO:  Set this
                                //                      //shoot both balls and fix this later
                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 2000),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000)
                                ),

                                //move to spike2 and shut off systems
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, shootToPGPSpikePath, true),
                                        new ShooterSpinUpCommand(robot.shooter,0.0),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 250),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,250)
                                ),

                                //gobble up spike2 balls
                                new ParallelCommandGroup(
                                        new FollowPathCommand( robot.follower, eatPGPPath, true),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 3000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  3000)
                                ),
                                new ShooterSpinUpCommand(robot.shooter, -1000),
                                new WaitCommand(250),
                                new ShooterSpinUpCommand(robot.shooter, 0.0),

                                //prepare and move to shoot position
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, endPGPToShootPath, true),
                                        new ShooterSpinUpCommand(robot.shooter,3850),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 250),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,250)
                                ),
                                //shoot and fix this later
                                new ParallelCommandGroup(
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 2000),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,2000)
                                ),
                                //park outside of launch zone and power down subsystems
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, shootToGPPSpikePath, true),
                                        new ShooterSpinUpCommand(robot.shooter,0.0),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 250),
                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,250)
                                )

                        )
                )
        );

        //park somewhere
//                                new ParallelCommandGroup(
//
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 2000),
//                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000),
//                                        new FollowPathCommand( robot.follower, path6, true)
//                                ),
//
//
//                                new ParallelCommandGroup(
//                                        new ShooterSpinUpCommand(robot.shooter,4200),
//
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 3000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 3000),
//                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  3000)


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

