package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Robot;

public class Feeder extends SubsystemBase {

    private CRServo servo;

    public enum FeederState{
        REVERSE,
        FORWARD,
        STOP
    }
    private FeederState state;

    /**
     * class to define feeder object
     * @param feed servo to use as feeder
     */
    public Feeder (CRServo feed){
        servo = feed;
        state = FeederState.STOP;
        servo.set(0.0);
    }


    public void feed (FeederState state){
        this.state = state;
        switch(state){
            case FORWARD:
                servo.set(-1.0);
                break;

            case REVERSE:
                servo.set(1.0);
                break;

            case STOP:
                servo.set(0.0);
                break;

        }
    }

    public FeederState getState() {
        return state;
    }
}