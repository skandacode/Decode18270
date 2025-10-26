package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Artifact;

import solverslib.controller.PIDFController;
import solverslib.controller.feedforwards.SimpleMotorFeedforward;
import solverslib.hardware.AbsoluteAnalogEncoder;
import solverslib.hardware.motors.Motor;
import solverslib.util.MathUtils;

@Configurable
public class Spindexer implements Subsystem{
    private Motor spindexerMotor;
    private AnalogInput spindexerEncoder;

    private Artifact[] artifactPositions;
    private int currentIndex = 0;
    private double encoderPos = 0;
    private double error;

    public static double ABS_OFFSET = 100;
    public static double errorTolerance = 10;


    public static double kP = 0.008;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double kS = 0.09;
    public static double kV = 1;

    private PIDFController spindexerController;
    private SimpleMotorFeedforward feedforward;


    public enum SpindexerPositions{
        INTAKE1(30),
        INTAKE2(145),
        INTAKE3(270),
        SHOOT1(-146),
        SHOOT2(-25),
        SHOOT3(93);

        public final double pos;

        SpindexerPositions(double pos) {
            this.pos = pos;
        }
    }

    private double curr_pos;


    public Spindexer(HardwareMap hardwareMap, Artifact[] startingPositions) {
        spindexerController = new PIDFController(kP, kI, kD, kF);
        feedforward = new SimpleMotorFeedforward(kS, kV, 0);
        spindexerMotor = new Motor(hardwareMap, "spindexer_motor");
        spindexerEncoder = hardwareMap.analogInput.get("spindexer_encoder");
        artifactPositions = startingPositions;
        curr_pos = SpindexerPositions.INTAKE1.pos;
    }
    public Spindexer(HardwareMap hardwareMap) {
        this(hardwareMap, new Artifact[]{Artifact.NONE, Artifact.NONE, Artifact.NONE});
    }

    public Artifact[] getArtifactPositions() {
        return artifactPositions;
    }

    public void setPosition(SpindexerPositions position) {
        curr_pos = position.pos;
    }

    public void shootPos(int index) {
        if (index < 0 || index >= 3) return;
        currentIndex = index;
        if (index == 0) {
            setPosition(SpindexerPositions.SHOOT1);
        } else if (index == 1) {
            setPosition(SpindexerPositions.SHOOT2);
        } else {
            setPosition(SpindexerPositions.SHOOT3);
        }
    }
    public void afterShoot(){
        artifactPositions[currentIndex] = Artifact.NONE;
    }

    public void intakePos(int index) {
        if (index < 0 || index >= 3) return;
        currentIndex = index;
        if (index == 0) {
            setPosition(SpindexerPositions.INTAKE1);
        } else if (index == 1) {
            setPosition(SpindexerPositions.INTAKE2);
        } else {
            setPosition(SpindexerPositions.INTAKE3);
        }
    }
    public void afterIntake(Artifact artifact) {
        artifactPositions[currentIndex] = artifact;
    }

    public boolean atTarget(){
        return Math.abs(error) < errorTolerance;
    }

    public double getEncoderPosition(){
        return encoderPos;
    }

    public int getIndex(Artifact artifact){
        for (int index = 0; index < artifactPositions.length; index++) {
            if (artifactPositions[index] == artifact) {
                return index;
            }
        }
        return -1;
    }

    @Override
    public void update() {
        error = MathUtils.normalizeAngle(curr_pos - getEncoderPosition(), false, AngleUnit.DEGREES)-360;
        double power = feedforward.calculate(spindexerController.calculate(error, 0));
        if (Math.abs(error)<1){
            power=0;
        }
        System.out.println(error+"     "+power);
        spindexerMotor.set(-power);
        spindexerMotor.update();
        encoderPos = getEncoderDegrees();
    }
    private double getEncoderDegrees(){
        return AngleUnit.normalizeDegrees((spindexerEncoder.getVoltage())/3.227*360 + ABS_OFFSET);
    }
    private double getEncoderVoltage(){
        return (spindexerEncoder.getVoltage());
    }
}
