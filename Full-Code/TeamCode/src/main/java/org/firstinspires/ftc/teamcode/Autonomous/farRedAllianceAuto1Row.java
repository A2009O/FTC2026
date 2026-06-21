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

@Autonomous(preselectTeleOp = "RedAllianceTeleOp")
public class farRedAllianceAuto1Row extends OpMode {

    private DcMotor turret;

    double TICKS_PER_DEGREE = 4.75;
    double MIN_DEG = -100;
    double MAX_DEG = 150;

    // AIMING POWER SETTINGS
    double kP = 0.004;
    double MAX_POWER = 1;

    double turretDeg = 0.0;

    double TARGET_X = 144;
    double TARGET_Y = 134;

    DcMotorEx Shooter;
    public double targetVelocity = 1420;
    double F = 14.23;
    double P = 500;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private boolean shotStarted = false;

    private final FarMechanisms shooter = new FarMechanisms();

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD1,
        DRIVE_SHOOTPOS_INTAKEPOS,
        DRIVE_INTAKE_SHOOTPOS,
        SHOOT_PRELOAD2,
        DRIVE_SHOOTPOS_HUMANPOS1,
        DRIVE_HUMANPOS_SHOOTPOS1,
        SHOOT_PRELOAD3,
        DRIVE_SHOOTPOS_HUMANPOS2,
        DRIVE_HUMANPOS_SHOOTPOS2,
        SHOOT_PRELOAD4,
        DRIVE_SHOOTPOS_HUMANPOS3,
        DRIVE_HUMANPOS_SHOOTPOS3,
        SHOOT_PRELOAD5,
        DRIVE_SHOOTPOS_ENDPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(90.42803009345795, 4.18037383177568, Math.toRadians(3));
    private final Pose shootPose = new Pose(84.47663551401868, 11.813084112149527, Math.toRadians(0));
    private final Pose intakePose = new Pose(131.8130836728972, 33, Math.toRadians(0));
    private final Pose intakeCurve = new Pose(72.03364485981308, 35.822429906542055);
    private final Pose humanPose = new Pose(138.58878504672896, 4.299065420560762, Math.toRadians(0));
    private final Pose endPose = new Pose(87.03738317757009, 32.32710280373833, Math.toRadians(0));

    private PathChain driveStartPosShootPos, driveShootPosIntakePos, driveIntakePosShootPos, driveShootPosHumanPos1,
            driveHumanPosShootPos1, driveShootPosHumanPos2, driveHumanPosShootPos2, driveShootPosHumanPos3,
            driveHumanPosShootPos3, driveShootPosEndPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        driveShootPosIntakePos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intakeCurve, intakePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        driveIntakePosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(intakePose, intakeCurve, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        driveShootPosHumanPos1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, humanPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        driveHumanPosShootPos1 = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        driveShootPosHumanPos2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, humanPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        driveHumanPosShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        driveShootPosHumanPos3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, humanPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        driveHumanPosShootPos3 = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    shooter.setPause();
                    follower.followPath(driveStartPosShootPos, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 3) {

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
                    setPathState(PathState.DRIVE_SHOOTPOS_HUMANPOS1);
                }
                break;

            case DRIVE_SHOOTPOS_HUMANPOS1:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveShootPosHumanPos1, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 2) {

                    setPathState(PathState.DRIVE_HUMANPOS_SHOOTPOS1);
                }
                break;

            case DRIVE_HUMANPOS_SHOOTPOS1:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    shooter.setPause();
                    follower.followPath(driveHumanPosShootPos1, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 0.5) {

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
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEPOS);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEPOS:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveShootPosIntakePos, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 0.5) {

                    setPathState(PathState.DRIVE_INTAKE_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKE_SHOOTPOS:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    shooter.setPause();
                    follower.followPath(driveIntakePosShootPos, true);
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
                    setPathState(PathState.DRIVE_SHOOTPOS_HUMANPOS2);
                }
                break;

            case DRIVE_SHOOTPOS_HUMANPOS2:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveShootPosHumanPos2, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 2) {

                    setPathState(PathState.DRIVE_HUMANPOS_SHOOTPOS2);
                }
                break;

            case DRIVE_HUMANPOS_SHOOTPOS2:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    shooter.setPause();
                    follower.followPath(driveHumanPosShootPos2, true);
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
                    setPathState(PathState.DRIVE_SHOOTPOS_HUMANPOS3);
                }
                break;

            case DRIVE_SHOOTPOS_HUMANPOS3:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(driveShootPosHumanPos3, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 2) {

                    setPathState(PathState.DRIVE_HUMANPOS_SHOOTPOS3);
                }
                break;

            case DRIVE_HUMANPOS_SHOOTPOS3:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    shooter.setPause();
                    follower.followPath(driveHumanPosShootPos3, true);
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
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    shooter.setPause();
                    follower.followPath(driveShootPosEndPos, true);
                }

                if (!follower.isBusy()
                        && pathTimer.getElapsedTimeSeconds() > 0.5) {
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

        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
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

        int targetTicks =
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