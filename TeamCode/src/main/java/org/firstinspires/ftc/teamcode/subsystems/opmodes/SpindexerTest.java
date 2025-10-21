package org.firstinspires.ftc.teamcode.subsystems.opmodes;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
@TeleOp
@Configurable
public class SpindexerTest extends LinearOpMode {
    Spindexer spindexer;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        spindexer = new Spindexer(hardwareMap);
        Spindexer.SpindexerPositions currPos = Spindexer.SpindexerPositions.INTAKE1;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                currPos = Spindexer.SpindexerPositions.INTAKE1;
            } else if (gamepad1.b) {
                currPos = Spindexer.SpindexerPositions.INTAKE2;
            } else if (gamepad1.x) {
                currPos = Spindexer.SpindexerPositions.INTAKE3;
            }else if (gamepad1.dpad_down){
                currPos = Spindexer.SpindexerPositions.SHOOT1;
            }
            spindexer.setPosition(currPos);
            telemetry.addData("Artifact Positions", java.util.Arrays.toString(spindexer.getArtifactPositions()));
            telemetry.addData("Spindexer Position", spindexer.getEncoderPosition());
            telemetry.update();
            spindexer.update();
        }
    }
}