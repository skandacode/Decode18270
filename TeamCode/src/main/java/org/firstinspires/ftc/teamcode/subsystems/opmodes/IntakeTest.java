package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Arrays;

@TeleOp
@Configurable
public class IntakeTest extends LinearOpMode {
    Intake intake;
    public static double intakePower = 0.0;
    
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            intake.setPower(intakePower);
            intake.update();
            telemetry.addData("color dist", intake.getIntakeDistance());
            telemetry.addData("color", Arrays.toString(intake.getColor()));
            telemetry.addData("Artifact", (intake.getArtifact()));
            telemetry.update();
        }
    }
}
