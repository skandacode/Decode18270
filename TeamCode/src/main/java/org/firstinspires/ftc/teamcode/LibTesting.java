package org.firstinspires.ftc.teamcode;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import solverslib.hardware.Cached2mDistance;
import solverslib.hardware.CachedColorSensorV3;

@Configurable
@TeleOp
public class LibTesting extends LinearOpMode {

    private TelemetryManager panelsTelemetry;

    public static double testVar = 0.0;

    CachedColorSensorV3 color;
    Cached2mDistance distance;

    @Override
    public void runOpMode() {
        panelsTelemetry = Panels.getTelemetry();
        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

        color = (CachedColorSensorV3) hardwareMap.get(RevColorSensorV3.class, "color");
        distance = (Cached2mDistance) hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        waitForStart();

        while (opModeIsActive()) {
            panelsTelemetry.debug("This is a line!");
            panelsTelemetry.debug("Here is a variable: " + Math.PI);
            panelsTelemetry.debug("Test variable: " + testVar);
            panelsTelemetry.debug();
            panelsTelemetry.debug("Color Value: "+color.getDistance(DistanceUnit.INCH));
            panelsTelemetry.debug("Color last refresh: "+color.getLastRefresh());
            panelsTelemetry.debug();
            panelsTelemetry.debug("Distance Value: "+distance.getDistance(DistanceUnit.INCH));
            panelsTelemetry.debug("Distance last refresh: "+distance.getLastRefresh());
            panelsTelemetry.debug();
            panelsTelemetry.update(telemetry);
        }
    }
}
