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

    private Motor shooterMotor1, shooterMotor2;
    //private final ServoEx turret1;
    //private final ServoEx turret2;
    private ServoEx kicker1;
    private ServoEx kicker2;

    private double currentVelo = 0.0;
    public double targetVelo = 0.0;

    public static double kickerUpPos = 0.25;
    public static double kickerDownPos = 0.42;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = new Motor(hardwareMap, "outtakemotor1");
        shooterMotor2 = new Motor(hardwareMap, "outtakemotor2");

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
        if (targetVelo == 0){
            shooterMotor1.set(0);
            shooterMotor2.set(0);
        }else {
            currentVelo = Math.abs(shooterMotor2.getCorrectedVelocity());

            if (targetVelo <= currentVelo) {
                shooterMotor1.set(0);
                shooterMotor2.set(0);
            } else {
                shooterMotor1.set(-1);
                shooterMotor2.set(1);
            }
        }
        kicker1.update();
        kicker2.update();
        //turret1.update();
        //turret2.update();
        shooterMotor1.update();
        shooterMotor2.update();
        System.out.println("Update is running");
    }
}