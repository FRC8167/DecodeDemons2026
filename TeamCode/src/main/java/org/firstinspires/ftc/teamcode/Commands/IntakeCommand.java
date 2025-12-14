package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final Intake.MotorState motorState;
    private final ElapsedTime timer = new ElapsedTime();
    private final double duration;

    public IntakeCommand(Intake intake, Intake.MotorState motorState, double duration){
        this.intake = intake;
        this.motorState = motorState;
        this.duration = duration;
        addRequirements(intake);
    }


    @Override
    public void initialize() {
        timer.reset();
        intake.setMotorState(motorState);
    }

    @Override
    public void execute() {
        intake.setIntakeState();
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

