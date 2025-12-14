package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Feeder;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class SetIntakeCommand extends CommandBase {
    private final Intake intake;
    private final ElapsedTime timer = new ElapsedTime();
    private final double duration;

    public SetIntakeCommand(Intake intake, double duration){
        this.intake = intake;
        this.duration = duration;
        addRequirements(intake);
    }


    @Override
    public void initialize() {
        timer.reset();
        intake.forward();
    }

    @Override
    public void execute() {
        intake.forward();
    }

    @Override
    public boolean isFinished() {
        return duration > 0 && timer.milliseconds() > duration;
    }


    @Override
    public void end(boolean interrupted) {
        intake.off();
    }

}

