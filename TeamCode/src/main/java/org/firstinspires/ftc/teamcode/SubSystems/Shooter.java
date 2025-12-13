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
        public static double kv = 0.00045;
        public static double kp = 0.00045;
        public static double ki = 0.05;
        public static double kd = 0.00008;
        public static double tolerance= 100.0;  //RPMs




    public static final InterpLUT distanceToRPM;
        static{
            distanceToRPM = new InterpLUT();
            distanceToRPM.add(20.0, 1743.0);
            distanceToRPM.add(66.7, 3000.0);
            distanceToRPM.add(87.0, 3200.0);
            distanceToRPM.add(116.0, 4000.0);
            distanceToRPM.add(144.0, 4800.0);
            //in and RPM
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
            double targetRPM = 0.0;
            if (ATdistance > 20 && ATdistance <=144){
                targetRPM = distanceToRPM.get(ATdistance);

            }
            else
            {
                targetRPM = 3400;
            }
            ticksPerSec = convertRPMToTicksPerSec(targetRPM);
            shooterPID.setSetPoint(ticksPerSec);
        }





    }

