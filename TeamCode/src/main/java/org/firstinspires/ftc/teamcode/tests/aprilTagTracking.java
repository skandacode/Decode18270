package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import solverslib.controller.PIDFController;


@TeleOp
@Configurable
public class aprilTagTracking extends LinearOpMode{
    Drivetrain drivetrain;
    Limelight3A limelight;
    HardwareMap hardwareMap;

    PIDFController turnController = new PIDFController(0.005, 0, 0,0);

    public void runOpMode() throws InterruptedException{
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);

        limelight.start();

        waitForStart();

        turnController.setSetPoint(0);

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double forward = 0;
                double strafe = 0;
                double turn = turnController.calculate(tx);
                drivetrain.driveRobotCentric(forward, strafe, turn);

                telemetry.addData("tx value", result.getTx());
            }

            telemetry.update();
            drivetrain.update();
        }

    }

}
