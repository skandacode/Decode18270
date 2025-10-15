package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Artifact;

import solverslib.controller.PIDFController;
import solverslib.hardware.AbsoluteAnalogEncoder;
import solverslib.hardware.motors.Motor;
import solverslib.util.MathUtils;

@Configurable
public class Spindexer implements Subsystem{
    private Motor spindexerMotor;
    private AbsoluteAnalogEncoder spindexerEncoder;

    private Artifact[] artifactPositions;
    private int currentIndex = 0;
    private double encoderPos = 0;

    public static double ABS_OFFSET = 0.0;
    public static double ABS_VOLTS_PER_ROT = 3.3;


    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    private PIDFController spindexerController;


    public enum SpindexerPositions{
        INTAKE1(0),
        INTAKE2(120),
        INTAKE3(240),
        SHOOT1(180),
        SHOOT2(300),
        SHOOT3(60);

        public final double pos;

        SpindexerPositions(double pos) {
            this.pos = pos/360.0;
        }
    }

    private double curr_pos;


    public Spindexer(HardwareMap hardwareMap, Artifact[] startingPositions) {
        spindexerController = new PIDFController(kP, kI, kD, kF);
        spindexerMotor = new Motor(hardwareMap, "spindexer_motor");
        spindexerEncoder = new AbsoluteAnalogEncoder(hardwareMap, "spindexer_encoder", ABS_VOLTS_PER_ROT, AngleUnit.DEGREES);
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

    public double getEncoderPosition(){
        return encoderPos;
    }

    @Override
    public void update() {
        double error = MathUtils.normalizeAngle(curr_pos - getEncoderPosition(), false, AngleUnit.DEGREES);
        double power = spindexerController.calculate(error, 0);
        spindexerMotor.set(power);
        spindexerMotor.update();
        encoderPos = spindexerEncoder.getCurrentPosition()/360.0 + ABS_OFFSET;
    }
}
