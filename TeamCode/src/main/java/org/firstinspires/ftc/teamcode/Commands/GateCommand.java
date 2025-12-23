package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Gate;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class GateCommand extends CommandBase {
    private final Gate gate;
    private final Gate.GateState gateState;


    public GateCommand(Gate gate, Gate.GateState gateState){
        this.gate = gate;
        this.gateState = gateState;
        addRequirements(gate);
    }


    @Override
    public void initialize() {
        gate.setGateState(gateState);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }


    @Override
    public void end(boolean interrupted) {

    }

}

