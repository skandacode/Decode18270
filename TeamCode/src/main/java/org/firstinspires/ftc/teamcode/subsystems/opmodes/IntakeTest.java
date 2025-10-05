package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
        }
    }
}
