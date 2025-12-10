package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Feeder;
//This is a command for controlling the feeder subsystem
//A command is a reusable action the robot can perform such as run feeder FWD

public class FeederCommand extends CommandBase {

    //TODO:  Reference the subsystem this command controls
    //private final Subsystem subsystem

    //TODO:  Create a constructor with parameters
    //TODO:  include addRequirements(subsystem) which prevents commands from controlling more than one system at once
    @Override
    public void initialize() {
        //Runs ONCE at the start of the command

    }

    @Override
    public void execute() {

    }
    //Runs repeatedly while the command is active


    @Override
    public boolean isFinished() {
        return true;
    }
    //Determines wehn teh command ends
    //You can use a timer

    @Override
    public void end(boolean interrupted) {

    }
    //Runs when the command finishes or is interrupted
}
