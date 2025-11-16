package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
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
        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

        color = (CachedColorSensorV3) hardwareMap.get(RevColorSensorV3.class, "color");
        distance = (Cached2mDistance) hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("This is a test var", testVar);
            telemetry.addData("Red", color.red());
            telemetry.addData("Color distance", color.getDistance(DistanceUnit.INCH));
            telemetry.addData("Color last updated", color.getLastRefresh());
            telemetry.addData("2m distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("2m last updated", distance.getLastRefresh());
            telemetry.update();
        }
    }
}
