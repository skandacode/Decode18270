package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Configurable
public class ShooterSetPower extends LinearOpMode {
    Shooter shooter;
    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = new Shooter(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            shooter.setDirectPower(power);
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.update();
        }
    }
}
