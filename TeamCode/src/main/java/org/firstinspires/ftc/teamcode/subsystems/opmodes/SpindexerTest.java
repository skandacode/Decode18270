package org.firstinspires.ftc.teamcode.subsystems.opmodes;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
@TeleOp
@Configurable
public class SpindexerTest extends LinearOpMode {
    Spindexer spindexer;
    @Override
    public void runOpMode() throws InterruptedException {
        spindexer = new Spindexer(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            spindexer.setPositions(Spindexer.SpindexerPositions.INTAKE1);
            telemetry.addData("Artifact Positions", java.util.Arrays.toString(spindexer.getArtifactPositions()));
            telemetry.update();
            spindexer.update();
        }
    }
}