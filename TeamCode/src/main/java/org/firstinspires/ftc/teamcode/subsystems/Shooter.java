package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import solverslib.controller.PIDFController;
import solverslib.controller.feedforwards.SimpleMotorFeedforward;

@Configurable
public class Shooter implements Subsystem {

    private DcMotorEx shooterMotor1, shooterMotor2;
    private Servo kicker1, kicker2;

    private PIDFController pidf;
    private SimpleMotorFeedforward feedforward;
    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;

    // --- PIDF coefficients ---
    public static double kP = 0.0009;
    public static double kI = 0.0001;
    public static double kD = 0.0002;

    public static double kS = 0.08; // Static feedforward
    public static double kV = 1.0/2900; // Velocity feedforward

    public static boolean enablePIDF = true;

    // --- Kicker positions ---
    public static double KICKER_UP = 0.25;
    public static double KICKER_DOWN = 0.42;

    // --- Low-pass filter coefficient (for smoothing) ---
    public static double ALPHA = 0.3;
    private double smoothedVelocity = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "outtakemotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "outtakemotor2");

        kicker1 = hardwareMap.get(Servo.class, "kicker1");
        kicker2 = hardwareMap.get(Servo.class, "kicker2");

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new PIDFController(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);
    }

    public void setTargetVelocity(double target) {
        targetVelocity = target;
        pidf.reset();
    }

    public double getCurrentVelocity() {
        return smoothedVelocity;
    }

    public void setDirectPower(double power) {
        shooterMotor1.setPower(-power);
        shooterMotor2.setPower(power);
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
    }

    public double getTargetVelo() {
        return targetVelocity;
    }
}
