package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class Temporary_Long_Auto extends LinearOpMode {
    // we cannot shoot from the far launch zone so if we start there then use this
    private DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    public void runOpMode() {
        frontRightWheel = hardwareMap.get(DcMotor.class, "backLeftWheel");
        frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeftWheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "frontRightWheel");
        backLeftWheel = hardwareMap.get(DcMotor.class, "backLeftWheel");

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {
            frontLeftWheel.setPower(0.6);
            frontRightWheel.setPower(0.6);
            backLeftWheel.setPower(0.6);
            backRightWheel.setPower(0.6);
            sleep(1500);
            frontLeftWheel.setPower(0);
            frontRightWheel.setPower(0);
            backLeftWheel.setPower(0);
            backRightWheel.setPower(0);
        }
    }
}
