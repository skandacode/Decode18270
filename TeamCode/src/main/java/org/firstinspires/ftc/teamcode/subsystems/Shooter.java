package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import solverslib.controller.PIDFController;

public class Shooter implements Subsystem {

    private DcMotorEx shooterMotor1, shooterMotor2;
    private Servo kicker1, kicker2;

    private PIDFController pidf;
    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;

    // --- PIDF coefficients ---
    private double kP = 0.0009;
    private double kI = 0.0001;
    private double kD = 0.0002;
    private double kF = 0.00015;

    // --- Kicker positions ---
    private static final double KICKER_UP = 0.25;
    private static final double KICKER_DOWN = 0.42;

    // --- Low-pass filter coefficient (for smoothing) ---
    private static final double ALPHA = 0.3;
    private double smoothedVelocity = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "outtakemotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "outtakemotor2");

        kicker1 = hardwareMap.get(Servo.class, "kicker1");
        kicker2 = hardwareMap.get(Servo.class, "kicker2");

        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new PIDFController(kP, kI, kD, kF);
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
        // Measure velocity (assuming shooterMotor2 has encoder plugged in)
        currentVelocity = Math.abs(shooterMotor2.getVelocity());
        smoothedVelocity = ALPHA * currentVelocity + (1 - ALPHA) * smoothedVelocity;

        double outputPower;

        if (targetVelocity <= 0) {
            outputPower = 0;
            smoothedVelocity = 0;
        } else {
            outputPower = pidf.calculate(smoothedVelocity, targetVelocity);
        }

        // Clamp to [-1, 1] range for safety
        outputPower = Math.max(-1, Math.min(1, outputPower));

        // Apply power (reverse one side)
        setDirectPower(outputPower);
    }

    public double getTargetVelo() {
        return targetVelocity;
    }
}
