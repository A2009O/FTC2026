package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorTest extends OpMode {

    DcMotor Turret, Intake, Transfer, rr, rf, lf, lr;
    DcMotorEx Shooter;

    @Override
    public void init(){

        Turret = hardwareMap.get(DcMotor.class, "Turret");

        // ===== DRIVE =====
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");

        rf.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);

        // ===== SHOOTER =====
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setDirection(DcMotor.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");

        Intake.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("READY");
        telemetry.update();
    }

    /*
        Turret ->
        Shooter ->
        Intake ->
        Transfer ->
        rr ->
        rf ->
        lf ->
        lr ->
     */

    @Override
    public void loop() {
        if(gamepad1.a){
            Turret.setPower(0.3);
        }
        else if(gamepad1.b){
            Shooter.setPower(0.5);
        }
        else if(gamepad1.x){
            Intake.setPower(0.3);
        }
        else if(gamepad1.y) {
            Transfer.setPower(0.3);
        }
        else if(gamepad1.dpad_up) {
            rr.setPower(1);
        }
        else if(gamepad1.dpad_down) {
            rf.setPower(1);
        }
        else if(gamepad1.dpad_left) {
            lr.setPower(1);
        }
        else if(gamepad1.dpad_right) {
            lf.setPower(1);
        }
        else{
            lf.setPower(0);
            rf.setPower(0);
            rr.setPower(0);
            lr.setPower(0);
            Transfer.setPower(0);
            Intake.setPower(0);
            Shooter.setPower(0);
            Turret.setPower(0);
        }
    }
}
