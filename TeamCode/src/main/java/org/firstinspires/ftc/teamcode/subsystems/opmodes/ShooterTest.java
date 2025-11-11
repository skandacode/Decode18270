package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
@TeleOp
@Configurable
public class ShooterTest extends LinearOpMode {
    Shooter shooter;
    public static double shooterPower = 0;
    public static boolean kick = false;
    public static double turretangle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = new Shooter(hardwareMap);
        waitForStart();
        long t1 = System.nanoTime();
        while (opModeIsActive()) {
            shooter.setTargetVelocity(shooterPower);
            shooter.setTurretAngle(turretangle);

            if (kick){
                shooter.kickerUp();
            }
            else{
                shooter.kickerDown();
            }
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            shooter.update();
            long t2 = System.nanoTime();
            long dt = t2-t1;
            telemetry.addData("Loops ms", dt/1000000.0);
            t1=t2;
            telemetry.update();
        }
    }
}