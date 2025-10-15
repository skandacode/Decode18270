package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import solverslib.hardware.ServoEx;
import solverslib.hardware.motors.Motor;
import solverslib.hardware.motors.MotorGroup;

@Configurable
public class Shooter implements Subsystem {
    public static double SERVO_TO_DEGREES = 300.0;
    public static double TURRET_OFFSET = 0.5;

    private Motor shooterMotors;
    //private final ServoEx turret1;
    //private final ServoEx turret2;
    private ServoEx kicker1;
    private ServoEx kicker2;

    private double currentVelo = 0.0;
    private double targetVelo = 0.0;

    public static double kickerUpPos = 0.3;
    public static double kickerDownPos = 0.52;

    public Shooter(HardwareMap hardwareMap) {
        Motor shooterMotor1 = new Motor(hardwareMap, "outtakemotor1");
        Motor shooterMotor2 = new Motor(hardwareMap, "outtakemotor2");
        shooterMotors = new MotorGroup(shooterMotor1, shooterMotor2);

        //turret1 = new ServoEx(hardwareMap, "turret1");
        //turret2 = new ServoEx(hardwareMap, "turret2");
        kicker1 = new ServoEx(hardwareMap, "kicker1");
        kicker2 = new ServoEx(hardwareMap, "kicker2");
    }

    public void setTargetVelo(double targetVelo) {
        this.targetVelo = targetVelo;
    }

    public void setTurretAngle(double angle) {
        double position = angle / SERVO_TO_DEGREES + TURRET_OFFSET;
        //turret1.setPosition(position);
        //turret2.setPosition(position);
    }

    public double getCurrentVelo() {
        return currentVelo;
    }
    public void kickerUp() {
        kicker1.setPosition(kickerUpPos);
        kicker2.setPosition(1-kickerUpPos);
    }
    public void kickerDown() {
        kicker1.setPosition(kickerDownPos);
        kicker2.setPosition(1-kickerDownPos);
    }

    @Override
    public void update() {
        currentVelo = shooterMotors.getVelocity();
        shooterMotors.set(targetVelo < currentVelo ? 0.0 : 1.0);
        kicker1.update();
        kicker2.update();
        //turret1.update();
        //turret2.update();
    }
}