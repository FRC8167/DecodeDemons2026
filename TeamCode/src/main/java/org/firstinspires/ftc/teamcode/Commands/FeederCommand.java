package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Feeder;
//This is a command for controlling the feeder subsystem
//A command is a reusable action the robot can perform such as run feeder FWD

public class FeederCommand extends CommandBase {


    private final Feeder feeder;
    private final Feeder.FeederState feederState;
    private final ElapsedTime timer = new ElapsedTime();
    private final double duration;

    public FeederCommand(Feeder.FeederState feederState, Feeder feeder, double duration){
        this.feeder = feeder;
        this.feederState = feederState;
        this.duration = duration;
        addRequirements(feeder);
    }


    @Override
    public void initialize() {
        timer.reset();
        feeder.feed(feederState);
    }

    @Override
    public void execute() {
        feeder.feed(feederState);
    }

    @Override
    public boolean isFinished() {
        return duration > 0 && timer.milliseconds() > duration;
    }


    @Override
    public void end(boolean interrupted) {
        feeder.feed(Feeder.FeederState.STOP);
    }
    
}
