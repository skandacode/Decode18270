package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import solverslib.controller.feedforwards.SimpleMotorFeedforward;
import solverslib.drivebase.MecanumDrive;
import solverslib.hardware.motors.Motor;

public class Drivetrain implements Subsystem{
    private MecanumDrive drive;
    private SimpleMotorFeedforward forwardFF;
    private SimpleMotorFeedforward strafeFF;
    private SimpleMotorFeedforward turnFF;

    private double forwardPower = 0.0;
    private double strafePower = 0.0;
    private double turnPower = 0.0;
    public Drivetrain(HardwareMap hardwareMap) {
        Motor frontLeft = new Motor(hardwareMap, "frontleft");
        Motor frontRight = new Motor(hardwareMap, "frontright");
        Motor backLeft = new Motor(hardwareMap, "backleft");
        Motor backRight = new Motor(hardwareMap, "backright");
        frontLeft.setInverted(false);
        backLeft.setInverted(false);
        frontRight.setInverted(false);
        backRight.setInverted(false);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        forwardFF = new SimpleMotorFeedforward(0.0, 1.0, 0.0);
        strafeFF = new SimpleMotorFeedforward(0.0, 1.0, 0.0);
        turnFF = new SimpleMotorFeedforward(0.0, 1.0, 0.0);
    }

    public void driveRobotCentric(double forward, double strafe, double turn) {
        forwardPower = forwardFF.calculate(forward);
        strafePower = strafeFF.calculate(strafe);
        turnPower = turnFF.calculate(turn);
    }

    @Override
    public void update(){
        drive.driveRobotCentric(strafePower, forwardPower, turnPower);
        drive.update();
    }
}
