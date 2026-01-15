package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

import solverslib.hardware.ServoEx;
import solverslib.hardware.motors.Motor;

@TeleOp
@Configurable
public class v2test extends LinearOpMode {
    public static double shooterPower = 0;
    public static double kick = 0.5;
    public static double  intakePowergood = 0;
    public static double  intakePowerbad = 0;
    public static double gatePos1 = 0.5;
    public static double gatePos2 = 0.5;
    public static double spindexerPos = 0.5;

    private Motor shooterMotor1, shooterMotor2, intakeMotor, intakeMotorbad;
    private ServoEx kicker;
    private ServoEx gate1, gate2;
    private ServoEx spindexer1, spindexer2;



    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        long t1 = System.nanoTime();
        while (opModeIsActive()) {
            shooterMotor1 = new Motor(hardwareMap, "outtakemotor1");
            shooterMotor2 = new Motor(hardwareMap, "outtakemotor2");
            intakeMotor = new Motor(hardwareMap, "goodintake");
            intakeMotorbad = new Motor(hardwareMap, "badintake");

            kicker = new ServoEx(hardwareMap, "kicker");
            gate1 = new ServoEx(hardwareMap, "gate1");
            gate2 = new ServoEx(hardwareMap, "gate2");
            spindexer1 = new ServoEx(hardwareMap, "spindexer1");
            spindexer2 = new ServoEx(hardwareMap, "spindexer2");

            shooterMotor1.set(shooterPower);
            shooterMotor2.set(-shooterPower);
            intakeMotor.set(intakePowergood);
            intakeMotorbad.set(intakePowerbad);

            kicker.setPosition(kick);
            gate1.setPosition(gatePos1);
            gate2.setPosition(gatePos2);
            spindexer1.setPosition(spindexerPos);
            spindexer1.update();
            spindexer2.setPosition(spindexerPos);
            spindexer2.update();
            gate1.update();
            gate2.update();
            kicker.update();
            intakeMotor.update();
            intakeMotorbad.update();

            shooterMotor1.update();
            shooterMotor2.update();
            telemetry.update();
        }
    }
}