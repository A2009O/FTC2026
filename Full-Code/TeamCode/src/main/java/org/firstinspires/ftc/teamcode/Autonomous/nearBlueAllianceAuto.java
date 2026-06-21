package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotMemory;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(preselectTeleOp = "BlueAllianceTeleOp")
public class nearBlueAllianceAuto extends OpMode {

    private DcMotor turret;

    double TICKS_PER_DEGREE = 4.75;
    double MIN_DEG = -100;
    double MAX_DEG = 150;

    // AIMING POWER SETTINGS
    double kP = 0.004;
    double MAX_POWER = 1;

    double turretDeg = 0.0;

    int targetTicks = 0;

    double TARGET_X = 0;
    double TARGET_Y = 134;

    DcMotorEx Shooter;
    public double targetVelocity = 1200;
    double F = 14.23;
    double P = 500;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private boolean shotStarted = false;

    private final NearMechanisms shooter = new NearMechanisms();

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD1,
        DRIVE_SHOOTPOS_INTAKE1POS,
        DRIVE_INTAKE1POS_ENDPOS,
        SHOOT_PRELOAD2,
        DRIVE_SHOOTPOS_INTAKE2POS,
        DRIVE_INTAKE2POS_SHOOTPOS,
        SHOOT_PRELOAD3,
        DRIVE_SHOOTPOS_GATEINTAKE1,
        DRIVE_GATEINTAKE_SHOOTPOS1,
        SHOOT_PRELOAD4,
        DRIVE_SHOOTPOS_GATEINTAKE2,
        DRIVE_GATEINTAKE_SHOOTPOS2,
        SHOOT_PRELOAD5,
        DRIVE_SHOOTPOS_GATEINTAKE3,
        DRIVE_GATEINTAKE_SHOOTPOS3,
        SHOOT_PRELOAD6
    }

    PathState pathState;

    private final Pose startPose  = new Pose(15.291588785046721, 115.65700934579438, Math.toRadians(141));
    private final Pose shootPose  = new Pose(48.853426586218724,  79.65944430374081, Math.toRadians(180));
    private final Pose intake1Pose= new Pose(14.177570093457943,   81.7476635514019,  Math.toRadians(180));
    private final Pose intake2Curve = new Pose(65.49532710280374,   53.831775700934585);
    private final Pose intake2Pose= new Pose(0,  53.6355140186916, Math.toRadians(180));
    private final Pose gateIntake = new Pose(5.02429906542056,   55.13551401869161, Math.toRadians(159));
    private final Pose gateCurve = new Pose(48.59345794392524, 66.02336448598129);
    private final Pose endPose   = new Pose(56.10280373831776,   111.97196261682244, Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosIntake1Pos, driveIntake1PosEndPos, driveShootPosIntake2Pos,
    driveIntake2PosShootPos, driveShootPosGateIntake1, driveGateIntakeShootPos1, driveShootPosGateIntake2,
            driveGateIntakeShootPos2, driveShootPosGateIntake3, driveGateIntakeShootPos3;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosIntake2Pos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intake2Curve, intake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake2Pose.getHeading())
                .build();

        driveIntake2PosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(intake2Pose, intake2Curve, shootPose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosGateIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, gateCurve, gateIntake))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gateIntake.getHeading())
                .build();

        driveGateIntakeShootPos1 = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntake, gateCurve, shootPose))
                .setConstantHeadingInterpolation(gateIntake.getHeading())
                .build();

        driveShootPosGateIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, gateCurve, gateIntake))
                .setConstantHeadingInterpolation(gateIntake.getHeading())
                .build();

        driveGateIntakeShootPos2 = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntake, gateCurve, shootPose))
                .setConstantHeadingInterpolation(gateIntake.getHeading())
                .build();

        driveShootPosGateIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, gateCurve, gateIntake))
                .setConstantHeadingInterpolation(gateIntake.getHeading())
                .build();

        driveGateIntakeShootPos3 = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntake, gateCurve, shootPose))
                .setConstantHeadingInterpolation(gateIntake.getHeading())
                .build();

        driveShootPosIntake1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .build();

        driveIntake1PosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, endPose))
                .setConstantHeadingInterpolation(intake1Pose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveStartPosShootPos, true);
                }
                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 2) {

                    setPathState(PathState.SHOOT_PRELOAD1);
                }
                break;

            case SHOOT_PRELOAD1:
                if (!follower.isBusy() && !shotStarted) {
                    shooter.setLaunch();
                    shotStarted = true;
                }
                if (shotStarted && !shooter.isBusy()) {
                    shotStarted = false;
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKE2POS);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKE2POS:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveShootPosIntake2Pos, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 0.5) {

                    setPathState(PathState.DRIVE_INTAKE2POS_SHOOTPOS);
                }

                break;

            case DRIVE_INTAKE2POS_SHOOTPOS:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveIntake2PosShootPos, true);
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    shooter.setPause();
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 2) {

                    setPathState(PathState.SHOOT_PRELOAD2);
                }

                break;

            case SHOOT_PRELOAD2:
                if (!follower.isBusy() && !shotStarted) {
                    shooter.setLaunch();
                    shotStarted = true;
                }
                if (shotStarted && !shooter.isBusy()) {
                    shotStarted = false;
                    setPathState(PathState.DRIVE_SHOOTPOS_GATEINTAKE1);
                }
                break;

            case DRIVE_SHOOTPOS_GATEINTAKE1:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveShootPosGateIntake1, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 3) {

                    setPathState(PathState.DRIVE_GATEINTAKE_SHOOTPOS1);
                }

                break;

            case DRIVE_GATEINTAKE_SHOOTPOS1:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveGateIntakeShootPos1, true);
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    shooter.setPause();
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 0.5) {

                    setPathState(PathState.SHOOT_PRELOAD3);
                }

                break;

            case SHOOT_PRELOAD3:
                if (!follower.isBusy() && !shotStarted) {
                    shooter.setLaunch();
                    shotStarted = true;
                }
                if (shotStarted && !shooter.isBusy()) {
                    shotStarted = false;
                    setPathState(PathState.DRIVE_SHOOTPOS_GATEINTAKE2);
                }
                break;

            case DRIVE_SHOOTPOS_GATEINTAKE2:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveShootPosGateIntake2, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 3.6) {

                    setPathState(PathState.DRIVE_GATEINTAKE_SHOOTPOS2);
                }

                break;

            case DRIVE_GATEINTAKE_SHOOTPOS2:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveGateIntakeShootPos2, true);
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    shooter.setPause();
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 0.5) {

                    setPathState(PathState.SHOOT_PRELOAD4);
                }

                break;

            case SHOOT_PRELOAD4:
                if (!follower.isBusy() && !shotStarted) {
                    shooter.setLaunch();
                    shotStarted = true;
                }
                if (shotStarted && !shooter.isBusy()) {
                    shotStarted = false;
                    setPathState(PathState.DRIVE_SHOOTPOS_GATEINTAKE3);
                }
                break;

            case DRIVE_SHOOTPOS_GATEINTAKE3:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveShootPosGateIntake3, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 3.9) {

                    setPathState(PathState.DRIVE_GATEINTAKE_SHOOTPOS3);
                }

                break;

            case DRIVE_GATEINTAKE_SHOOTPOS3:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveGateIntakeShootPos3, true);
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    shooter.setPause();
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 0.5) {

                    setPathState(PathState.SHOOT_PRELOAD5);
                }

                break;

            case SHOOT_PRELOAD5:
                if (!follower.isBusy() && !shotStarted) {
                    shooter.setLaunch();
                    shotStarted = true;
                }
                if (shotStarted && !shooter.isBusy()) {
                    shotStarted = false;
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKE1POS);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKE1POS:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveShootPosIntake1Pos, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 0.5) {

                    setPathState(PathState.DRIVE_INTAKE1POS_ENDPOS);
                }

                break;

            case DRIVE_INTAKE1POS_ENDPOS:

                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveIntake1PosEndPos, true);
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3){
                    shooter.setPause();
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 0.5) {

                    setPathState(PathState.SHOOT_PRELOAD6);
                }

                break;

            case SHOOT_PRELOAD6:
                if (!follower.isBusy() && !shotStarted) {
                    shooter.setLaunch();
                    shotStarted = true;
                }
                if (shotStarted && !shooter.isBusy()) {
                    shotStarted = false;
                    shooter.setPause();
                    Shooter.setVelocity(0);
                    telemetry.addLine("Autonomous Done");
                }
                break;
                
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        // ===== TURRET =====
        turret = hardwareMap.get(DcMotor.class, "Turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // ===== INIT =====
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        telemetry.addLine("Done INIT");
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        Shooter.setVelocity(targetVelocity);
    }

    @Override
    public void loop() {

        follower.update();
        shooter.update();
        statePathUpdate();

        Pose pose = follower.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double headingDeg = Math.toDegrees(pose.getHeading());

        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        double distance = Math.hypot(dx, dy);

        targetVelocity = 741.7823 +
                (23.71982*distance) -
                (0.4833158*distance*distance) +
                (0.004071122*distance*distance*distance) -
                (0.00001110041*distance*distance*distance*distance);

        Shooter.setVelocity(targetVelocity);

        turretTrack(dx, dy, headingDeg);

        telemetry.addData("path state: ", pathState.toString());
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("path time: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Turret Deg", turretDeg);
        telemetry.addData("Turret Ticks", targetTicks);
    }

    @Override
    public void stop() {
        RobotMemory.autoEndPose = new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()
        );
    }

    private void turretTrack(double dx, double dy, double headingDeg) {

        double angleToGoal =
                Math.toDegrees(
                        Math.atan2(dy, dx)
                );

        // =================================================
        // CONVERT FIELD ANGLE TO ROBOT RELATIVE
        // =================================================

        turretDeg =
                angleWrap(
                        angleToGoal
                                - headingDeg
                );

        turretDeg = Range.clip(
                turretDeg,
                MIN_DEG,
                MAX_DEG
        );

        targetTicks =
                (int)(-turretDeg * TICKS_PER_DEGREE);

        turret.setTargetPosition(targetTicks);

        double error =
                targetTicks
                        - turret.getCurrentPosition();

        double power =
                Range.clip(
                        Math.abs(error * kP),
                        0,
                        MAX_POWER
                );

        if (Math.abs(error) > 5) {
            power = Math.max(power, 0.08);
        }
        else {
            power = 0;
        }

        // APPLY POWER
        turret.setPower(power);
    }

    public double angleWrap(double degrees) {

        while (degrees > 180) {
            degrees -= 360;
        }

        while (degrees < -180) {
            degrees += 360;
        }

        return degrees;
    }
}