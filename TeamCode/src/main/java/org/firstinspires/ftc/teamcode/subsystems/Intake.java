package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import solverslib.hardware.motors.Motor;

public class Intake implements Subsystem {
    private Motor intakeMotor;
    private double power = 0.0;
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = new Motor(hardwareMap, "intake_motor");
    }

    public void setPower(double power) {
        this.power = power;
    }
    @Override
    public void update() {
        intakeMotor.set(power);
        intakeMotor.update();
    }
}
