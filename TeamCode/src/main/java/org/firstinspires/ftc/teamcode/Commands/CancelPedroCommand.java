package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;

public class CancelPedroCommand  extends CommandBase {
    private final Robot robot;

    public CancelPedroCommand() {
        robot = Robot.getInstance();
    }

    @Override
    public void initialize(){
        robot.follower.breakFollowing();
        robot.mecanumDrive.drive(0,0,0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
