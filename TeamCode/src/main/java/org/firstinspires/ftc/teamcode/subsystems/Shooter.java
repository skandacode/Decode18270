package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

import dev.nextftc.core.units.Angle;
import solverslib.controller.PIDFController;
import solverslib.controller.feedforwards.SimpleMotorFeedforward;
import solverslib.hardware.motors.Motor;

@Configurable
public class Shooter implements Subsystem {

    private Motor shooterMotor1, shooterMotor2;
    private Servo kicker1, kicker2;
    private Servo turret;

    private Limelight3A limelight;

    private AnalogInput turretEncoder;

    private PIDFController pidf;
    private SimpleMotorFeedforward feedforward;

    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;

    // --- Flywheel PIDF coefficients ---
    public static double kP = 0.0009;
    public static double kI = 0.0001;
    public static double kD = 0.0002;

    public static double kS = 0.08; // Static feedforward
    public static double kV = 1.0/2900; // Velocity feedforward

    public static boolean enablePIDF = true;

    // --- Turret bounds ---
    public static double upperBound = 0.8;
    public static double lowerBound = 0.22;

    // --- Kicker positions ---
    public static double KICKER_UP = 0.32;
    public static double KICKER_DOWN = 0.478;

    // --- Low-pass filter coefficient (for smoothing) ---
    public static double ALPHA = 0.3;
    private double smoothedVelocity = 0.0;

    private Pipeline currPipeline = Pipeline.MOTIF;

    public enum Pipeline{
        BLUETRACK (0), REDTRACK (1), MOTIF (2);

        public final int pipelineIndex;

        Pipeline(int pipelineIndex){
            this.pipelineIndex = pipelineIndex;
        }
    }

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = new Motor(hardwareMap, "outtakemotor1");
        shooterMotor2 = new Motor(hardwareMap, "outtakemotor2");

        turretEncoder = hardwareMap.analogInput.get("turret_encoder");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        kicker1 = hardwareMap.get(Servo.class, "kicker1");
        kicker2 = hardwareMap.get(Servo.class, "kicker2");
        turret = hardwareMap.get(Servo.class, "turret");

        pidf = new PIDFController(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        setCurrPipeline(Pipeline.MOTIF);
    }

    public void setCurrPipeline(Pipeline pipeline){
        this.currPipeline = pipeline;
        limelight.pipelineSwitch(this.currPipeline.pipelineIndex);
    }

    public double getTurretVoltage(){
        return turretEncoder.getVoltage();
    }

    public int getMotif(){
        if (this.currPipeline != Pipeline.MOTIF){
            setCurrPipeline(Pipeline.MOTIF);
        }
        LLResult result = limelight.getLatestResult();
        int pattern = 0;
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId()>20&&fr.getFiducialId()<24){
                    pattern=fr.getFiducialId()-20;
                }
            }
        }
        return pattern;
    }

    public double convertAreatoDist(double area){
        double percentage = area*100;
        return (84.87951 / (Math.pow(percentage, 0.5) + 0.075744)) - 9.92407;   //https://www.desmos.com/calculator/mosqvk2gyd
    }

    public double convertDistToVelo(double dist){
        return 2000;
    }

    public double[] getAngleDistance(Pipeline color){
        if (color == Pipeline.MOTIF){
            throw new IllegalArgumentException("Bull is a bum and made the color a motif. Pipeline.MOTIF is not a color");
        }
        if (this.currPipeline != color){
            setCurrPipeline(color);
        }
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                return new double[]{fr.getTargetXDegrees(), convertAreatoDist(fr.getTargetArea())};
            }
        }
        return null;
    }

    public void setTurretPos(double pos){
        turret.setPosition(pos);
    }

    public double convertVoltageToDegrees(double volts){
        return volts*109.95723-182.45569;
    }
    public double convertDegreestoServoPos(double deg){
        return deg*0.00322222+0.51;
    }

    public void aimAtTarget(Pipeline color){
        double turretPos;
        double shooterVelo;
        double currAngle = convertVoltageToDegrees(getTurretVoltage());
        System.out.println("Currangle "+currAngle);
        double[] angleAndDistance = getAngleDistance(color);
        System.out.println(Arrays.toString(angleAndDistance));
        if (angleAndDistance == null){
            System.out.println("Cannot See");
        }else {
            double dAngle = angleAndDistance[0];
            double distanceInches = angleAndDistance[1];

            double targetAngle = currAngle + dAngle;

            System.out.println(currAngle+"  "+dAngle+"   "+targetAngle);

            turretPos = convertDegreestoServoPos(targetAngle);
            turretPos = Range.clip(turretPos, lowerBound, upperBound);

            shooterVelo = convertDistToVelo(distanceInches);

            setTargetVelocity(shooterVelo);
            setTurretPos(turretPos);
        }
    }

    public void setTargetVelocity(double target) {
        targetVelocity = target;
        pidf.reset();
    }

    public double getCurrentVelocity() {
        return smoothedVelocity;
    }

    public void setDirectPower(double power) {
        shooterMotor1.set(-power);
        shooterMotor2.set(power);
    }

    public void kickerUp() {
        kicker1.setPosition(KICKER_UP);
        kicker2.setPosition(1 - KICKER_UP);
    }

    public void kickerDown() {
        kicker1.setPosition(KICKER_DOWN);
        kicker2.setPosition(1 - KICKER_DOWN);
    }

    @Override
    public void update() {
        // Measure velocity
        currentVelocity = Math.abs(shooterMotor2.getVelocity());
        smoothedVelocity = ALPHA * currentVelocity + (1 - ALPHA) * smoothedVelocity;

        double outputPower;

        if (targetVelocity <= 0) {
            outputPower = 0;
            smoothedVelocity = 0;
        } else {
            outputPower = feedforward.calculate(targetVelocity);
            if (enablePIDF){
                outputPower += pidf.calculate(smoothedVelocity, targetVelocity);
            }
        }

        // Clamp to [-1, 1] range for safety
        outputPower = Math.max(-1, Math.min(1, outputPower));

        // Apply power
        setDirectPower(outputPower);
        shooterMotor1.update();
        shooterMotor2.update();
    }

    public double getTargetVelo() {
        return targetVelocity;
    }
}
