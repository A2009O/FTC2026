package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class shooterPIDF extends OpMode {
    DcMotor rf, rr, lf, lr;
    DcMotorEx Shooter;

    public double minVelocity = 700;
    public double maxVelocity = 0.0;

    public double targetVelocity = 0.0;
    double F = 14.23;
    double P = 400;

    double[] stepSizes = {500.0, 100.0, 50.0, 10.0, 1.0, 0.1, 0.01};

    int stepIndex = 0;

    DcMotor Intake, Transfer;

    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {

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

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");

        Intake.setDirection(DcMotor.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "PinPoint");
        pinpoint.setOffsets(-5.243017001414863, 1.7831387257012778, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Done INIT");
        telemetry.update();

    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper) {
            Intake.setPower(0.7);
            Transfer.setPower(-1);
        }
        else if (gamepad1.left_bumper) {
            Intake.setPower(-1);
            Transfer.setPower(-1);
        }
        else if (gamepad1.right_trigger > 0) {
            Intake.setPower(1);
            Transfer.setPower(1);
        } else {
            Intake.setPower(0);
            Transfer.setPower(0);
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double heading = pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeft  = (rotY + rotX + rx) / denominator;
        double backLeft   = (rotY - rotX + rx) / denominator;
        double frontRight = (rotY - rotX - rx) / denominator;
        double backRight  = (rotY + rotX - rx) / denominator;

        lf.setPower(frontLeft);
        lr.setPower(backLeft);
        rf.setPower(frontRight);
        rr.setPower(backRight);

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.xWasPressed()) {
            if (targetVelocity != minVelocity){
                maxVelocity = targetVelocity;
                targetVelocity = minVelocity;
            }
            else{
                targetVelocity = maxVelocity;
            }
        }

        if(gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if(gamepad1.aWasPressed()) {
            targetVelocity -= stepSizes[stepIndex];
        }
        if(gamepad1.yWasPressed()) {
            targetVelocity += stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        Shooter.setVelocity(targetVelocity);

        double velocity = Shooter.getVelocity();
        double error = targetVelocity - velocity;

        telemetry.addData("Step Size (B): ", stepSizes[stepIndex]);
        telemetry.addData("targetVelocity (Y/A): ", targetVelocity);
        telemetry.addData("currentVelocity: ", velocity);
        telemetry.addData("Error: ", error);
        telemetry.addData("P (Up/Down): ", P);
        telemetry.addData("F (Right/Left): ", F);
        telemetry.addLine("To switch to slower velocity: (X)");
        telemetry.update();
    }
}