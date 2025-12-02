package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShooterSpinUpATCommand extends CommandBase {

    private final Shooter shooter;
    private AprilTagDetection tag;

    public ShooterSpinUpATCommand(Shooter shooterSubsystem, AprilTagDetection tag) {
        this.shooter = shooterSubsystem;
        this.tag = tag;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        shooter.smartVelocity(tag.ftcPose.range);
    }

    @Override
    public boolean isFinished() {
        return shooter.atTargetVelocity();
    }

    @Override
    public void end(boolean interrupted) {
            shooter.stop();

    }
}
