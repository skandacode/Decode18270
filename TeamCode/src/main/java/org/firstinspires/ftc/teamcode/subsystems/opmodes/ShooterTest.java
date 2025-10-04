package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
@TeleOp
@Configurable
public class ShooterTest extends LinearOpMode {
    Shooter shooter;
    public static double shooterPower = 0.0;
    public static boolean kick = false;
    public static double turretangle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            shooter.setTargetVelo(shooterPower);
            if (kick){
                shooter.kickerUp();
            }
            else{
                shooter.kickerDown();
            }
            shooter.setTurretAngle(turretangle);
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelo());
            telemetry.update();
            shooter.update();
        }
    }
}