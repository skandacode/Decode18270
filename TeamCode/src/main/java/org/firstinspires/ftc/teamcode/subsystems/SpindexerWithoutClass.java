package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Spindexer.ABS_OFFSET;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import solverslib.hardware.motors.Motor;

@TeleOp
@Configurable
public class SpindexerWithoutClass extends LinearOpMode {
    public static double power = 0.0;
    public static int counter = -180;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        Motor spindexer = new Motor(hardwareMap, "spindexer_motor");
        AnalogInput spindexerEncoder = hardwareMap.analogInput.get("spindexer_encoder");
        waitForStart();
        while (opModeIsActive()){
            spindexer.set(power);
            spindexer.update();
            telemetry.addData("EncoderPos", AngleUnit.normalizeDegrees((spindexerEncoder.getVoltage())/3.227*360 + ABS_OFFSET));
            telemetry.addData("EncoderVoltage", (spindexerEncoder.getVoltage()));

            counter++;
            if (counter>180){
                counter = -180;
            }
            telemetry.addData("Counter", counter);
            telemetry.update();
        }
    }
}
