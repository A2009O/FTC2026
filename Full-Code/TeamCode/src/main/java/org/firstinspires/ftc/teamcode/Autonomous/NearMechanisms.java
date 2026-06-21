package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class NearMechanisms {
    private DcMotor Intake, Transfer;

    private ElapsedTime stateTimer = new ElapsedTime();

    public enum ShooterState {
        INTAKE,
        LAUNCH,
        PAUSE
    }
    public ShooterState shooterState;
    private static final double SHOOT_TIME = 0.6;

    public void init(HardwareMap hardwareMap) {
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");

        Intake.setDirection(DcMotor.Direction.REVERSE);

        shooterState = ShooterState.PAUSE;
    }

    public void update(){
        switch (shooterState) {
            case INTAKE:
                Intake.setPower(0.7);
                Transfer.setPower(-0.7);
                break;

            case LAUNCH:
                Intake.setPower(1);
                Transfer.setPower(0.7);
                if (stateTimer.seconds() >= SHOOT_TIME) {
                    shooterState = ShooterState.INTAKE;
                }
                break;

            case PAUSE:
                Intake.setPower(0);
                Transfer.setPower(0);
                break;
        }
    }

    public void setLaunch() {
        if (shooterState != ShooterState.LAUNCH) {
            stateTimer.reset();
            shooterState = ShooterState.LAUNCH;
        }
    }

    public void setPause() {
        shooterState = ShooterState.PAUSE;
    }

    public boolean isBusy() {
        return shooterState == ShooterState.LAUNCH;
    }

}
