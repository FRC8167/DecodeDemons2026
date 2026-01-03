package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.SensorColor;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Cogintilities.MirrorUtility;
import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Feeder;
import org.firstinspires.ftc.teamcode.SubSystems.Gate;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.RGBLight;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;


import java.util.List;


public class Robot extends com.seattlesolvers.solverslib.command.Robot {

    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public enum Alliance {
        RED,
        BLUE,
        UNSPECIFIED
    }

    //field poses
    public static final Pose BLUE_SHOOT_FAR_POSE = MirrorUtility.mirror(new Pose(56, 12, Math.toRadians(-72)));
    private static final Pose RED_SHOOT_FAR_POSE = MirrorUtility.mirror(new Pose(56, 12, Math.toRadians(-72)));

    private Alliance alliance = Alliance.UNSPECIFIED;

    public Telemetry telemetry;


    public Pose autoEndPose = null;
    public static OpModeType OP_MODE_TYPE;
    static List<LynxModule> ctrlHubs;

    public MotorEx driveMotorRF;
    public MotorEx driveMotorLF;
    public MotorEx driveMotorRR;
    public MotorEx driveMotorLR;

    public MotorEx intakeMotor;
    public MotorEx shooterMotor;

    public SensorColor sensorColor;
    public Follower follower;

    public WebcamName webCam1;
//    public Limelight3A limelight;

    public GoBildaPinpointDriver pinpoint;


    public MecanumDrive mecanumDrive;


    public Intake intake;
    public Shooter shooter;
    public SensorColor colorSensor;
    public Vision vision;
    public Feeder feederF, feederR;
    public Gate gate;
    public RGBLight rgbLight;
    public ColorMatch colorMatch;


    public void init(HardwareMap hardwareMap) throws InterruptedException {


        // Hardware
        driveMotorRF = new MotorEx(hardwareMap, "RightFront").setCachingTolerance(0.01);
        driveMotorLF = new MotorEx(hardwareMap, "LeftFront").setCachingTolerance(0.01);
        driveMotorLR = new MotorEx(hardwareMap, "LeftRear").setCachingTolerance(0.01);
        driveMotorRR = new MotorEx(hardwareMap, "RightRear").setCachingTolerance(0.01);

        follower = Constants.createFollower(hardwareMap);

        intakeMotor = new MotorEx(hardwareMap, "Intake").setCachingTolerance(0.01);
        shooterMotor = new MotorEx(hardwareMap, "Shooter").setCachingTolerance(0.01);

        CRServo feederServoF = new CRServo(hardwareMap, "feederServoF");
        CRServo feederServoR = new CRServo(hardwareMap, "feederServoR");

        ServoEx gateServo = new ServoEx(hardwareMap, "gateServo");

        ServoEx rgbServo = new ServoEx(hardwareMap, "rgbServo");

        webCam1 = hardwareMap.get(WebcamName.class, "Webcam1");
        //limelight = hwMap.get(Limelight3A.class, "limelight");  //dreaming
        sensorColor = new SensorColor(hardwareMap, "sensorColor");



        ctrlHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : ctrlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        //Instantiate Subsystems
        mecanumDrive = new MecanumDrive(driveMotorLF, driveMotorLR, driveMotorRF, driveMotorRR);
        intake  = new Intake(intakeMotor);
        feederF  = new Feeder(feederServoF);
        feederR = new Feeder(feederServoR);
        shooter = new Shooter(shooterMotor);
        vision  = new Vision(webCam1);
        rgbLight = new RGBLight(rgbServo);
        colorMatch = new ColorMatch(sensorColor);
        gate = new Gate(gateServo);



        //Set default command for RGBLight using the registered colorMatch
        DetectArtifactCommand detectArtifactCommand =
                new DetectArtifactCommand(rgbLight, colorMatch); //, shooter);
        rgbLight.setDefaultCommand(detectArtifactCommand);

        //Register Subsystems
        register(mecanumDrive, intake, shooter, feederF, feederR, vision, rgbLight, colorMatch, gate);

        if (OP_MODE_TYPE.equals(OpModeType.AUTO)) {
            initHasMovement();
        }
    }

    public void initHasMovement() {
        //TODO what goes here??
    }

    public void setAlliance(Alliance ally) {
        alliance = ally;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public Pose getShootPose(){
        if (alliance == Alliance.BLUE) return BLUE_SHOOT_FAR_POSE;
        else if (alliance == Alliance.RED) return RED_SHOOT_FAR_POSE;
        else return BLUE_SHOOT_FAR_POSE;  //default if UNSPECIFIED
    }


}





