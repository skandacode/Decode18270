package org.firstinspires.ftc.teamcode.oldautos;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;


import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;


import java.util.Arrays;
import java.util.List;


@Autonomous(name = "RedAutoFarIndex", group = "Auto")
@Disabled
public class RedAutoFarIndex extends LinearOpMode {
    private Follower follower;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static int[] shootorder = {0, 1, 2};
    public boolean shooterButtonAll = false;
    Limelight3A limelight;
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    Spindexer spindexer;
    private StateMachine stateMachine;
    public int pattern = 1;
    public static double timeforkicker = 0.2;
    public static double timeforspin = 0.5;
    public static double timeForIntake = 0.23;
    private final Pose opengate = new Pose(105, -28.6, Math.toRadians(-90));
    private final Pose opengateback = new Pose(105, -7, Math.toRadians(-90));
    private final Pose startPose = new Pose(43, 0, Math.toRadians(-180));
    private final Pose shootPose = new Pose(130, 2.6, Math.toRadians(132.5));
    private final Pose intake1Pose = new Pose(115, 7, Math.toRadians(-90));
    private final Pose intake2Pose = new Pose(93, 7, Math.toRadians(-90));
    private final Pose intake3Pose = new Pose(67,7, Math.toRadians(-90));
    private final Pose intake1donePose = new Pose(115, -30, Math.toRadians(-90));
    private final Pose intake2donePose = new Pose(93, -38, Math.toRadians(-90));
    private final Pose intake3donePose = new Pose(67, -38, Math.toRadians(-90));
    private final Pose leave = new Pose(140, 2.6, Math.toRadians(135));




    public enum AutoStates {
        MOVETOSHOOT1, wait1, SHOOT1,
        MOVETOINTAKE1, INTAKE1, BACK, GATE, waitgate,
        MOVETOSHOOT2, wait2, SHOOT2,
        MOVETOINTAKE2, INTAKE2, INTAKE2BACK,
        MOVETOSHOOT3, wait3,SHOOT3,
        MOVETOINTAKE3, INTAKE3,
        MOVETOSHOOT4, wait4,SHOOT4,
        LEAVE
    }
    private enum RobotState {
        Intake1, wait1,
        Intake2, wait2,
        Intake3, wait3, reverseIntake,
        WaitForShoot,
        PreShoot1,
        Shoot1,
        PreShoot2,
        Shoot2,
        PreShoot3,
        Shoot3
    }


    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        follower = createFollower(hardwareMap);
        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();


        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .build();


        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake2Pose.getHeading())
                .build();

        PathChain toIntake1back = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, opengateback))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), opengateback.getHeading())
                .build();
        PathChain intakeToGate= follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, opengate))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), opengate.getHeading())
                .build();

        PathChain toIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake3Pose.getHeading())
                .build();
        PathChain toIntake1fin = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, intake1donePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), intake1donePose.getHeading())
                .build();


        PathChain toIntake2fin = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, intake2donePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake2donePose.getHeading())
                .build();
        PathChain toIntake2back= follower.pathBuilder()
                .addPath(new BezierLine(intake2donePose, intake2Pose))
                .setLinearHeadingInterpolation(intake2donePose.getHeading(), intake2Pose.getHeading())
                .build();


        PathChain toIntake3fin = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, intake3donePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), intake3donePose.getHeading())
                .build();


        PathChain toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose, shootPose))
                .setLinearHeadingInterpolation(intake1donePose.getHeading(), shootPose.getHeading())
                .build();


        PathChain toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose.getHeading())
                .build();


        PathChain toScore3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3donePose, shootPose))
                .setLinearHeadingInterpolation(intake3donePose.getHeading(), shootPose.getHeading())
                .build();
        PathChain park = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leave))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leave.getHeading())
                .build();


        follower.setStartingPose(startPose);


        StateMachine stateMachine = new StateMachineBuilder()
                .state(RobotState.Intake1)
                .onEnter(() -> {
                    spindexer.intakePos(0);
                })
                .transition(() -> intake.isIntaked())


                .state(RobotState.wait1)
                .onEnter(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(1);
                })
                .transitionTimed(timeForIntake)


                .state(RobotState.Intake2)
                .onEnter(() -> {
                    spindexer.intakePos(1);
                })
                .transition(() -> intake.isIntaked())


                .state(RobotState.wait2)
                .onEnter(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.intakePos(2);
                })
                .transitionTimed(timeForIntake)


                .state(RobotState.Intake3)
                .onEnter(() -> {
                    spindexer.intakePos(2);
                })
                .transition(() -> intake.isIntaked())


                .state(RobotState.wait3)
                .onEnter(() -> {
                    spindexer.afterIntake(intake.getArtifact());
                    spindexer.shootPos(0);
                })
                .transition(() -> spindexer.atTarget())
                .transitionTimed(0.3)
                .state(RobotState.reverseIntake)
                .onEnter(() -> {
                    intake.setPower(-0.4);
                })
                .transitionTimed(0.15)


                .state(RobotState.WaitForShoot)
                .onEnter(() -> {
                    intake.setPower(1);
                })
                .transition(() -> shooterButtonAll)
                .state(RobotState.PreShoot1)
                .onEnter(() -> {
                    shooterButtonAll=false;
                    spindexer.shootPos(shootorder[0]);
                })
                .transition(() -> spindexer.atTarget())
                .state(RobotState.Shoot1)
                .onEnter(() -> {
                    shooter.kickerUp();
                })
                .transitionTimed(timeforkicker)


                .onExit(() -> {
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })
                .state(RobotState.PreShoot2)
                .onEnter(() -> {
                    spindexer.shootPos(shootorder[1]);
                })
                .transitionTimed(timeforspin)


                .state(RobotState.Shoot2)
                .onEnter(() -> {
                    shooter.kickerUp();
                })
                .transitionTimed(timeforkicker)
                .onExit(() -> {
                    shooter.kickerDown();
                    spindexer.afterShoot();
                })
                .state(RobotState.PreShoot3)
                .onEnter(() -> {
                    spindexer.shootPos(shootorder[2]);
                })
                .transitionTimed(timeforspin)


                .state(RobotState.Shoot3)
                .onEnter(() -> {
                    shooter.kickerUp();
                })
                .transitionTimed(timeforkicker, RobotState.Intake1)
                .onExit(() -> {
                    shooter.kickerDown();
                    spindexer.afterShoot();
                    spindexer.intakePos(0);
                })
                .build();


        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    follower.followPath(toShoot, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .state(AutoStates.wait1)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                    }else if (pattern==2){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT3);
                    }else{
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT2);

                    }
                })
                .transitionTimed(0.5)
                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(1.8)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)

                .state(AutoStates.INTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1fin, 0.5, true);
                })
                .transitionTimed(2)
                .state(AutoStates.BACK)
                .onEnter(()->{
                    follower.followPath(toIntake1back, 0.9, true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.GATE)
                .onEnter(()->{
                    follower.followPath(intakeToGate, 0.5, true);
                    intake.setPower(0);
                })
                .transitionTimed(0.3)
                .state(AutoStates.waitgate)
                .onEnter(()->{
                })
                .transitionTimed(2)
                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    intake.setPower(1);
                    follower.followPath(toScore1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.wait2)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT3);
                    }else if (pattern==2){
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT2);
                    }else{
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                    }
                })
                .transitionTimed(0.5)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)

                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2fin, 0.5, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(3)

                .state(AutoStates.INTAKE2BACK)
                .onEnter(()->{
                    follower.followPath(toIntake2back, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)

                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    follower.followPath(toScore2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)

                .state(AutoStates.wait3)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT2);
                    }else if (pattern==2){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                    }else{
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT3);
                    }
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2.3)
                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3fin, 0.5, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(3)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    follower.followPath(toScore3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(3)
                .state(AutoStates.wait4)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT1);
                    }else if (pattern==2){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT3);
                    }else{
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPositions.SHOOT2);

                    }
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2)
                .state(AutoStates.LEAVE)
                .onEnter(()->{
                    follower.followPath(park, true);
                })
                .build();

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    panelsTelemetry.debug("Fiducial", "ID: %d" , fr.getFiducialId());
                    if (fr.getFiducialId()>20&&fr.getFiducialId()<24){
                        pattern=fr.getFiducialId()-20;
                    }
                }
            } else {
                panelsTelemetry.debug("Limelight", "No data available");
            }
            panelsTelemetry.debug("Pattern", pattern);
            panelsTelemetry.debug("Init Pose: " + follower.getPose());
            panelsTelemetry.update(telemetry);
        }

        telemetry.update();
        waitForStart();
        stateMachine.start();
        stateMachine.setState(RobotState.WaitForShoot);
        spindexer.setArtifactPositions(new String[] {"GREEN", "PURPLE", "PURPLE"});
        autoMachine.start();
        intake.setPower(1);
        shooter.setTargetVelocity(2100);
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            stateMachine.update();
            autoMachine.update();
            follower.update();
            intake.update();
            shooter.update();
            spindexer.update();
            panelsTelemetry.debug("Artifact colors", Arrays.toString(spindexer.getArtifactPositions()));
            panelsTelemetry.debug("State: " + stateMachine.getState());
            panelsTelemetry.debug("State auto: " + autoMachine.getState());
            panelsTelemetry.debug("Pose: " + follower.getPose());
            panelsTelemetry.update(telemetry);


            telemetry.update();
        }
    }
}

