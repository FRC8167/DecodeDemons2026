package org.firstinspires.ftc.teamcode.SubSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;


@Configurable
public class Shooter_Alternate extends SubsystemBase {

//    private final Robot robot = Robot.getInstance();

    private final MotorEx shooterMotor;
    private final double ENCODER_TICKS = 28;
    private final PIDFController shooterPID;
    private double ticksPerSec;

    /* Public Static for Panels Access */
    public static double targetRPM = 0.0;
    public static double kV = 0.2;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double pidToleranceRPM = 100.0;


    public Shooter_Alternate(MotorEx motor) {
        shooterMotor = motor;
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setInverted(true);
        shooterPID = new PIDFController(kP, kI, kD, kV);
        shooterPID.setTolerance(rpmToTicksPerSec(50));

        setVelocity(0);
    }


    @Override
    public void periodic() {

        /* *** These two lines needed for dynamic updating from panels *** */
        shooterPID.setPIDF(kP, kI, kD, kV);
        shooterPID.setTolerance(rpmToTicksPerSec(pidToleranceRPM));

        double currentVelocity = shooterMotor.getVelocity();
        double output = shooterPID.calculate(currentVelocity, ticksPerSec);

        shooterMotor.set(output);
    }


    public void setVelocity(double targetRevPerMin) {
        ticksPerSec = rpmToTicksPerSec(targetRevPerMin);
        shooterPID.setSetPoint(ticksPerSec);
    }


    private double rpmToTicksPerSec(double rpm) {
        return rpm * ENCODER_TICKS / 60.0;
    }


    private double ticksPerSecToRPM(double ticksPerSec) {
        return ticksPerSec / ENCODER_TICKS * 60.0;
    }


    public boolean atTargetVelocity() {
        return shooterPID.atSetPoint();
    }


    //Helper methods or as Dave says:  getters???

    public double getRPM() {
        return ticksPerSecToRPM(shooterMotor.getVelocity());
    }


    public double getTicsPerSec() {
        return shooterMotor.getVelocity();
    }


    public double getTargetRPM() {
        return targetRPM;
    }


}

