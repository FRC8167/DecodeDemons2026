package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake extends SubsystemBase{

    private MotorEx motor;

    private boolean isRunning;

    private Double fSpeed = 0.7;
    private Double rSpeed = -0.4;


    public Intake (MotorEx intakeMotor){
        motor = intakeMotor;
        off();
    }


    public void off(){
        motor.set(0.0);
        isRunning = false;
    }


    public void forward() {
        if (isRunning){
            off();
        } else{
            motor.set(fSpeed);
            isRunning = true;}
    }


    public void reverse(){
        if (isRunning){
            off();
        }else{
        motor.set(rSpeed);
        isRunning = true;}
    }


    public boolean getIntakeState(){
        return isRunning;
    }
}






