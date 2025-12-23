package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveToPoseCommand;
import org.firstinspires.ftc.teamcode.Commands.FeederCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSmartSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Feeder;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
//@Disabled
@TeleOp(name="MainTeleOp", group="Competition")
public class MainTeleOp extends CommandOpMode {

    public GamepadEx driver;
    public GamepadEx operator;
    public ElapsedTime timer;
    private final Robot robot = Robot.getInstance();
    static TelemetryManager telemetryM;


    private final Pose startPose = new Pose(24, 24, Math.toRadians(0)); // Test
    private Pose autoEndPose = new Pose(0, 0, 0);
    private final Pose shootFarPose = new Pose(56, 12, Math.toRadians(-66));


    @Override
    public void initialize() {

        // Must have for all opModes
        Robot.OP_MODE_TYPE = Robot.OpModeType.TELEOP;
        // Resets the command scheduler
        super.reset();
        //Initialize the robot
        try {
            robot.init(hardwareMap);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        //added 12-22
        robot.mecanumDrive.setDefaultCommand(new DriveCommand(robot.mecanumDrive, gamepad1));
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //end pose held in robot
        //if auto ran, last pose is used or else default of 24,24,0
        Pose startPose = robot.autoEndPose != null ? robot.autoEndPose : new Pose(24, 24, 0);
        robot.follower.setStartingPose(startPose);
        robot.follower.update();
        //only schedule perpetually running commands
        schedule(new DriveCommand(robot.mecanumDrive, gamepad1));
        schedule(new VisionCommand(robot.vision));
        schedule(new DetectArtifactCommand(robot.rgbLight, robot.colorMatch)); //, robot.shooter));

        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);



        //******OPERATOR CONTROLS*****
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whileHeld(new ShooterSmartSpinUpCommand(robot.shooter, robot.vision))
            .whenReleased(new ShooterSpinUpCommand(robot.shooter, 0.0));


        operator.getGamepadButton(GamepadKeys.Button.BACK)
            .whileHeld(new ShooterSpinUpCommand(robot.shooter, -1000.0))
            .whenReleased(new ShooterSpinUpCommand(robot.shooter, 0.0));


        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new ParallelCommandGroup(
                        new InstantCommand(robot.intake::forward),
                        new InstantCommand(robot.feederF::forwardTogal)
                        )

                );

//        operator.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new InstantCommand( robot.feederF::forwardTogal)
//                        );


        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SequentialCommandGroup(
                        new InstantCommand(robot.intake::reverse),
                        new InstantCommand(robot.feederF::reverseTogal
                        )));



        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(()->robot.feederR.feed(Feeder.FeederState.FORWARD)))
                .whenReleased(new InstantCommand(()->robot.feederR.feed(Feeder.FeederState.STOP)));



        operator.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(()->robot.feederR.feed(Feeder.FeederState.REVERSE)))
                .whenReleased(new InstantCommand(()->robot.feederR.feed(Feeder.FeederState.STOP)));



        /* ******************************************************************************* */
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.mecanumDrive::enableSnailDrive))
                .whenReleased(new InstantCommand(robot.mecanumDrive::disableSnailDrive));


        driver.getGamepadButton(GamepadKeys.Button.X).
                whenPressed(new InstantCommand(robot.gate::open, robot.gate));

        driver.getGamepadButton(GamepadKeys.Button.Y).
                whenPressed(new InstantCommand(robot.gate::close, robot.gate));



        driver.getGamepadButton(GamepadKeys.Button.B).
                whenPressed(
                        new SequentialCommandGroup(
                                new ShooterSpinUpCommand(robot.shooter, 3850),
                                new DriveToPoseCommand(shootFarPose, driver),
                                new ParallelCommandGroup(
                                        new InstantCommand(robot.gate::open, robot.gate),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 2000),
                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000)
                                ),
                                new WaitCommand(250),
                                new InstantCommand(robot.gate::close, robot.gate),
                                new ShooterSpinUpCommand(robot.shooter, 0)
                        )
        );









//        driver.getGamepadButton(GamepadKeys.Button.Y). shoot far

    }

    @Override
    public void run() {
        super.run();
        robot.follower.update();
        autoEndPose = robot.follower.getPose();
        AprilTagDetection tag = robot.vision.getFirstTargetTag();



        if (tag != null) {
            telemetry.addLine("Target Tag Detected!");
            telemetry.addData("ID", tag.id);
            telemetry.addData("Center", "(%.0f, %.0f)", tag.center.x, tag.center.y);
            telemetry.addData("Range (in)", "%.1f", tag.ftcPose.range);
        } else {
            telemetry.addLine("No target tags (20–24) detected.");
        }


//       if (gamepad2.right_trigger > 0.3) {
//           robot.feederR.feed(Feeder.FeederState.FORWARD);
//           robot.feederF.feed(Feeder.FeederState.FORWARD);
//
//       }else{
//           robot.feederR.feed(Feeder.FeederState.STOP);
//           robot.feederF.feed(Feeder.FeederState.STOP);
//       }



        telemetry.addData("autoEndPose", autoEndPose.toString());
        telemetry.addData("FollowerX", Math.round(robot.follower.getPose().getX()*100)/100.0);
        telemetry.addData("FollowerY", Math.round(robot.follower.getPose().getY()*100)/100.0);
        telemetry.addData("FollowerH", Math.round(Math.toDegrees(robot.follower.getPose().getHeading())*100)/100.0);
        telemetry.addData("Distance to Goal", robot.vision.getDistanceToGoal());

        telemetry.addData("Shooter Velocity (RPM)", robot.shooter.getRPM());
        telemetry.addData("Shooter Ready?", robot.shooter.atTargetVelocity());

        telemetry.addData("FeederState?", robot.feederF.getState());
        telemetryM.addData("Shooter Ready?", robot.shooter.atTargetVelocity());

        telemetryM.update(telemetry);

    }


    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }

    public Pose getAutoEndPose() {
        return autoEndPose;
        }


    }







