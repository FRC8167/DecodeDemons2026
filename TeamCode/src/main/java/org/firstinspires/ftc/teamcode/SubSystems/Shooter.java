package org.firstinspires.ftc.teamcode.SubSystems;

import com.bylazar.configurables.annotations.Configurable;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Robot;

@Configurable
public class Shooter extends SubsystemBase {
//        private final Robot robot = Robot.getInstance();

    public MotorEx shooterMotor;
        private final PIDFController shooterPID;
        private double ticksPerSec;  //PID uses ticksPerSecond

        //Panels Configurables
        public static double targetRPM = 0.0;
        public static double kv = 0.0004;
        public static double kp = 0.002;
        public static double ki = 0.02;
        public static double kd = 0.00003;
        public static double tolerance= 200.0;  //RPMs

    public double RPM1;
    public double RPM2;
    public double RPM3;
    public double RPM4;
    public double RPM5;
    public double RPM6;
    public double RPM7;
    public double RPM8;
    public double RPM9;
    public double RPM10;



    public static final InterpLUT distanceToRPM;
        static{
            distanceToRPM = new InterpLUT();
            distanceToRPM.add(0.0, 0.0);
            distanceToRPM.add(12.0, 0.0);
            distanceToRPM.add(24.0, 0.0);
            distanceToRPM.add(36.0, 0.0);
            distanceToRPM.add(48.0, 0.0);
            distanceToRPM.add(60.0, 0.0);
            distanceToRPM.add(72.0, 0.0);
            distanceToRPM.add(84.0, 0.0);
            distanceToRPM.add(96.0, 0.0);
            distanceToRPM.add(108.0, 5800);  //feet and RPM
            distanceToRPM.createLUT();
        }


        public Shooter(MotorEx motor) {
            shooterMotor = motor;
            shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
            shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setInverted(true);
            shooterPID = new PIDFController(kp, ki, kd, kv);
            shooterPID.setTolerance(convertRPMToTicksPerSec(tolerance));  // PID needs ticksPerSec
            setVelocity(0.0);
        }

        public void setTargetRPM(double rpm) {
            targetRPM = rpm;
            shooterPID.setSetPoint(rpm);
        }

        public void stop() {
            targetRPM = 0;
        }

        @Override
        public void periodic() {
            shooterPID.setPIDF(kp, ki, kd, kv);
            shooterPID.setTolerance(convertRPMToTicksPerSec(tolerance));

            double currentVelocity = shooterMotor.getVelocity();
            double output = shooterPID.calculate(currentVelocity, ticksPerSec);

            shooterMotor.set(output);
        }

        public double convertRPMToTicksPerSec(double rpm) {
            return rpm * 28.0 / 60.0;
        }

        public double convertTicksPerSecToRPM(double ticksPerSec) {
            return ticksPerSec / 28.0 * 60.0;
        }

         public void setVelocity(double targetRPM) {
            ticksPerSec = convertRPMToTicksPerSec(targetRPM);
            shooterPID.setSetPoint(ticksPerSec);
        }

        public boolean atTargetVelocity() {
            return shooterPID.atSetPoint();
        }

        public double getRPM() {
            return convertTicksPerSecToRPM(shooterMotor.getVelocity());
        }

        public void smartVelocity(double ATdistance) {
            double targetRPM = distanceToRPM.get(ATdistance);
            ticksPerSec = convertRPMToTicksPerSec(targetRPM);
            shooterPID.setSetPoint(ticksPerSec);
        }





    }

