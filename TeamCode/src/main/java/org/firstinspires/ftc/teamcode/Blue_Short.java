package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name = "Blue_Short Auto")

public class Blue_Short extends LinearOpMode {

    private WebcamName camera;

    private DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    private DcMotor shooter;
    private DcMotor intakeStage1, intakeStage2, intakeStage3;
    private Servo blockShooter;

    DriveActions driver;

    double SHORT_SHOT = 0.3;
    double LONG_SHOT = 0.75;

    double open = 0;
    double closed = 0.6;

    boolean blocking = true;
    boolean shooter_active = false;
    boolean lastBlock = false;
    boolean lastShot = false;

    @Override

    public void runOpMode() {
        frontRightWheel = hardwareMap.get(DcMotor.class, "frontRightWheel");
        frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeftWheel");
        backRightWheel = hardwareMap.get(DcMotor.class, "backRightWheel");
        backLeftWheel = hardwareMap.get(DcMotor.class, "backLeftWheel");
        blockShooter = hardwareMap.get(Servo.class, "blockShooter");

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intakeStage1 = hardwareMap.get(DcMotor.class, "stage1");
        intakeStage2 = hardwareMap.get(DcMotor.class, "stage2");
        intakeStage3 = hardwareMap.get(DcMotor.class, "stage3");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        intakeStage1.setDirection(DcMotor.Direction.REVERSE);
        intakeStage2.setDirection(DcMotor.Direction.REVERSE);
        intakeStage3.setDirection(DcMotor.Direction.REVERSE);
        blockShooter.setDirection(Servo.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driver = new DriveActions (frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
                intakeStage1, intakeStage2, intakeStage3, shooter, blockShooter);

        waitForStart();

        if (opModeIsActive()) {
            driver.drive(-52, 0, 0, 1000);
            // driver.adjust(-52, 0, 0);
            //forward and right are inches, rotate is degrees, need to fix later
            shoot();
            driver.turn(-54, 1000);
            driver.drive(-40, 0, 0, 1000);
            sleep(100);
            driver.drive(40, 0, 0, 1000);
            driver.turn(54, 1000);
            shoot();
        }
    }

    private void shoot() {
        shooter.setPower(1);
        sleep(500);
        for (int i = 0; i < 3; i++) {
            blockShooter.setPosition(open);
            sleep(500);
            intakeStage3.setPower(1);
            sleep(200);
            intakeStage3.setPower(0);
            blockShooter.setPosition(closed);
            driver.startIntake();
            sleep(500);
            driver.stopIntake();
        }
        shooter.setPower(0);
        blockShooter.setPosition(closed);
    }
}