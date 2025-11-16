package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;

@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class SensorLimelight3A extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();
        boolean prevAPressed = false;
        boolean prevBPressed = false;
        limelight.pipelineSwitch(0);
        int pipelineIndex = 0;
        while (opModeIsActive()) {
            boolean currAPressed = gamepad1.a;
            boolean currBPressed = gamepad1.b;

            if (currAPressed && !prevAPressed){
                limelight.pipelineSwitch(0); //blue
            }
            if (currBPressed && !prevBPressed){
                limelight.pipelineSwitch(1); //red
            }

            prevAPressed = currAPressed;
            prevBPressed = currBPressed;

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            telemetry.addData("Pipeline index", pipelineIndex);
            telemetry.update();
        }
        limelight.stop();
    }
}