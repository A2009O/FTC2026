package org.firstinspires.ftc.teamcode.TestCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class shooterTargetVelocity extends OpMode {

    DcMotorEx Shooter;
    double F = 14.23;
    double P = 400;

    double TARGET_X = 11;
    double TARGET_Y = 138;

    Follower follower;

    public double targetVelocity = 0.0;

    double[] stepSizes = {500.0, 100.0, 50.0, 10.0, 1.0, 0.1, 0.01};

    int stepIndex = 0;

    DcMotor Intake, Transfer;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        // Pose(X, Y, HeadingRadians)

        follower.setPose(new Pose(
                37.375700934579434,
                52.526168224299046,
                Math.toRadians(177)
        ));

        // ===== SHOOTER =====
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");

        Intake.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Done INIT");
        telemetry.update();

    }

    @Override
    public void loop() {

        follower.update();

        Pose pose = follower.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();

        double dx = TARGET_X - robotX;
        double dy = TARGET_Y - robotY;

        double distance = Math.hypot(dx, dy);

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

        if(gamepad1.aWasPressed()) {
            targetVelocity -= stepSizes[stepIndex];
        }
        if(gamepad1.yWasPressed()) {
            targetVelocity += stepSizes[stepIndex];
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        Shooter.setVelocity(targetVelocity);

        telemetry.addData("Distance (INCH): ", distance);
        telemetry.addData("Target Velocity (Y/A): ", targetVelocity);
        telemetry.addData("StepSize (B): ", stepSizes[stepIndex]);
        telemetry.update();
    }
}