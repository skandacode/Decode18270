package org.firstinspires.ftc.teamcode;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp
public class LibTesting extends LinearOpMode {

    private TelemetryManager panelsTelemetry;

    public static double testVar = 0.0;


    @Override
    public void runOpMode() {
        panelsTelemetry = Panels.getTelemetry();
        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);
        waitForStart();

        while (opModeIsActive()) {
            panelsTelemetry.debug("This is a line!");
            panelsTelemetry.debug("Here is a variable: " + Math.PI);
            panelsTelemetry.debug("Test variable: " + testVar);
            panelsTelemetry.update(telemetry);
        }
    }
}
