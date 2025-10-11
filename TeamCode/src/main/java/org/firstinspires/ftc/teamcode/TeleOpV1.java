package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TeleOp Version 1.0")

public class TeleOpV1 extends LinearOpMode {
    private DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    private DcMotor leftLauncher, rightLauncher;
    private CRServo intakeStage1, intakeStage2, intakeStage3;

    boolean endgame = false;

    @Override

    public void runOpMode() {

        frontRightWheel = hardwareMap.get(DcMotorEx.class, "backLeftWheel");
        frontLeftWheel = hardwareMap.get(DcMotorEx.class, "frontLeftWheel");
        backRightWheel = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
        backLeftWheel = hardwareMap.get(DcMotorEx.class, "backLeftWheel");

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        intakeStage1 = hardwareMap.get(CRServo.class, "stage1");
        intakeStage2 = hardwareMap.get(CRServo.class, "stage2");
        intakeStage3 = hardwareMap.get(CRServo.class, "stage3");

        leftLauncher.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.REVERSE);
        intakeStage1.setDirection(CRServo.Direction.REVERSE);
        intakeStage2.setDirection(CRServo.Direction.REVERSE);
        intakeStage3.setDirection(CRServo.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Endgame", endgame);
                telemetry.addData("Intake Power", intakeStage1.getPower() + " " + intakeStage2.getPower() + " " + intakeStage3.getPower());
                telemetry.addData("Shooter Power", leftLauncher.getPower() + " " + rightLauncher.getPower());
                telemetry.update();

                double forward = gamepad1.left_stick_y;
                double right = gamepad1.left_stick_x;
                double rotate = gamepad1.right_stick_x;

                drive(forward, right, rotate);

                intake();
                shoot();

                if (gamepad1.y) endgame = !endgame;
                endgameExpansion();
            }
        }
    }

    private void drive(double forward, double right, double rotate) {
        telemetry.addData("Forward", forward);
        telemetry.addData("Right", right);
        telemetry.addData("Rotate", rotate);
        telemetry.update();

        frontRightWheel.setPower(forward - right - rotate);
        frontLeftWheel.setPower(forward + right + rotate);
        backRightWheel.setPower(forward + right - rotate);
        backLeftWheel.setPower(forward - right + rotate);
    }

    private void shoot() {
        if (gamepad2.right_trigger >= 0.1) { //for shooting in far launch zone
            leftLauncher.setPower(0.75);
            rightLauncher.setPower(0.75);
        }

        else if (gamepad2.left_trigger >= 0.1) { //for shooting in close launch zone
            leftLauncher.setPower(0.3);
            rightLauncher.setPower(0.3);
        }
    }

    private void intake() {
        if (gamepad2.dpad_up) {
            intakeStage1.setPower(1);
            intakeStage2.setPower(1);
            intakeStage3.setPower(1);
        }

        if (gamepad2.dpad_down) {
            intakeStage1.setPower(-1);
            intakeStage2.setPower(-1);
            intakeStage3.setPower(-1);
        }
    }

    private void endgameExpansion() {
        if (gamepad1.x) {
            // extend up to height limit to fit both robots in base
        }
    }

}
