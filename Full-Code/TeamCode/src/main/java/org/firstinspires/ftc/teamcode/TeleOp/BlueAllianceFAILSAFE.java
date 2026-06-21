package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotMemory;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class BlueAllianceFAILSAFE extends OpMode{

    DcMotor rf, rr, lf, lr;
    DcMotor Turret, Intake, Transfer;
    DcMotorEx Shooter;

    boolean autoAim = true;
    public double targetVelocity = 0.0;

    double F = 14.23;
    double P = 500;

    Follower follower;

    boolean track = true;
    double TICKS_PER_DEGREE = 4.75;
    double MIN_DEG = -100;
    double MAX_DEG = 150;

    // AIMING POWER SETTINGS
    double kP = 0.004;
    double MAX_POWER = 1;

    double turretDeg = 0.0;

    double TARGET_X = 5;
    double TARGET_Y = 134;

    double headingOffset = 0;

    boolean resetPressed = false;

    @Override
    public void init() {

        Turret = hardwareMap.get(DcMotor.class, "Turret");

        Turret.setTargetPosition(0);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        track = true;

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(132.766,
                4.781,
                Math.toRadians(177)));

        // ===== DRIVE =====
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");

        rf.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== SHOOTER =====
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        autoAim = true;

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");

        Intake.setDirection(DcMotor.Direction.REVERSE);

        resetPressed = false;

        telemetry.addLine("READY");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper) {
            Intake.setPower(0.7);
            Transfer.setPower(-1);
        } else if (gamepad1.left_bumper) {
            Intake.setPower(-1);
            Transfer.setPower(-1);
        } else if (gamepad1.right_trigger > 0) {
            Intake.setPower(1);
            Transfer.setPower(1);
        } else {
            Intake.setPower(0);
            Transfer.setPower(0);
        }

        if(gamepad1.xWasPressed()) reset();

        follower.update();

        Pose pose = follower.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();

        double headingDeg = Math.toDegrees(pose.getHeading());

        double headingRad;

        if(resetPressed) {
            headingRad = follower.getTotalHeading() - headingOffset + Math.PI;
        }
        else {
            headingRad = follower.getTotalHeading();
        }

        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        double distance = Math.hypot(dx, dy);

        if(autoAim)
            targetVelocity = 741.7823 +
                    (23.71982*distance) -
                    (0.4833158*distance*distance) +
                    (0.004071122*distance*distance*distance) -
                    (0.00001110041*distance*distance*distance*distance);
        else
            targetVelocity = 1200;

        Shooter.setVelocity(targetVelocity);

        if(track) turretTrack(dx, dy, headingDeg);
        else Turret.setPower(0);

        mecanumDrive(headingRad);

        telemetry.addData("Current X: ", robotX);
        telemetry.addData("Current Y: ", robotY);
        telemetry.addData("Heading Deg: ", headingDeg);
        telemetry.addData("Heading Rad: ", headingRad);
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Turret Deg: ", turretDeg);
        telemetry.update();

        if(gamepad1.dpadLeftWasPressed()) track = !track;
        if(gamepad1.yWasPressed()) autoAim = !autoAim;
    }

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

        Turret.setTargetPosition(targetTicks);

        double error =
                targetTicks
                        - Turret.getCurrentPosition();

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
        Turret.setPower(power);
    }

    public void mecanumDrive(double headingRad){

        // ---------------- DRIVE ----------------
        double y = -gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
        double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeft  = (rotY + rotX + rx) / denominator;
        double backLeft   = (rotY - rotX + rx) / denominator;
        double frontRight = (rotY - rotX - rx) / denominator;
        double backRight  = (rotY + rotX - rx) / denominator;

        lf.setPower(frontLeft);
        lr.setPower(backLeft);
        rf.setPower(frontRight);
        rr.setPower(backRight);
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

    public void reset() {

        follower.setPose(new Pose(
            132.766,
            4.781,
            Math.toRadians(177)
        ));

        headingOffset = follower.getTotalHeading() - Math.toRadians(177);

        resetPressed = true;
    }
}
