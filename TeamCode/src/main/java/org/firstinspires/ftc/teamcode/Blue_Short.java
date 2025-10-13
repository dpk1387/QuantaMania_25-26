package org.firstinspires.ftc.teamcode;

import android.util.Size;

//import ftc sdk classes for OpMode and hardware control
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//import vision and camera libraries
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Blue_Short Auto")

public class Blue_Short extends LinearOpMode {
    private WebcamName camera;
    private DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel; //mecanum wheels
    private DcMotor leftLauncher, rightLauncher;
    private CRServo intakeStage1, intakeStage2, intakeStage3;

    boolean endgame = false;

    //power constants for launcher
    double SHORT_SHOT = 0.3;
    double LONG_SHOT = 0.75;

    //drive actions helper object that simplifies robot movement
    DriveActions driver = new DriveActions (frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
            intakeStage1, intakeStage2, intakeStage3, leftLauncher, rightLauncher, camera);

    @Override

    public void runOpMode() {

        //INITIALIZE DRIVE MOTORS
        //motor mapping
        frontRightWheel = hardwareMap.get(DcMotorEx.class, "backLeftWheel");
        frontLeftWheel = hardwareMap.get(DcMotorEx.class, "frontLeftWheel");
        backRightWheel = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
        backLeftWheel = hardwareMap.get(DcMotorEx.class, "backLeftWheel");

        //motor directions
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        //when all motors break, power = 0 (prevents sliding)
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initalize launcher and intake systems
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        intakeStage1 = hardwareMap.get(CRServo.class, "stage1");
        intakeStage2 = hardwareMap.get(CRServo.class, "stage2");
        intakeStage3 = hardwareMap.get(CRServo.class, "stage3");
        //missing stage 4!!!!

        //motor directions
        leftLauncher.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.REVERSE);
        intakeStage1.setDirection(CRServo.Direction.REVERSE);
        intakeStage2.setDirection(CRServo.Direction.REVERSE);
        intakeStage3.setDirection(CRServo.Direction.REVERSE);
        //missing stage4

        //run without encoders for free power control
        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //SET UP CAMERA AND APRILTAG DETECTION
        camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        //build april tag processor to detect april tags
        //comments about this is in the AprilTag class
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(629.0, 629.0, 320.0, 240.0)
                .build();

        //build vision portal -- connects camera and processor
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor) //add created tag processor
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();

        waitForStart(); //wait for start

        //MAIN AUTO ROUTINE
        if (opModeIsActive()) {
            //drive back 60 inches (so from goal post, back 60 degrees)
            driver.drive(-60, 0, 0, 1000);

            //fine tune position
            driver.adjust(-60, 0, 0);
            ///forward and right are inches, rotate is degrees, need to fix later

            SHOOT();

            //rotate left (?) 45 degrees

            //goes to pick up the first stack of balls
            driver.turn(-45, 1000);

            //drive back 40 inches
            driver.drive(-40, 0, 0, 1000);

            sleep(100); //small delay for stabilization

            //drive forward to previous spot (40 inches forward)
            driver.drive(40, 0, 0, 1000);

            //turn robot 45 degrees right
            driver.turn(45, 1000);

            //2nd shot
            SHOOT();

            //missing some intermediate steps (i think) - lily
        }
    }

    //SHOOTING MECHANISM
    private void SHOOT() {
        //driver.adjust(0, 60, 0);
        //driver.intake(1000); //don't need until we have 4+ artifacts in auto

        driver.shoot(SHORT_SHOT);
        driver.intake(15000); //intake is on while we are shooting (15 sec)
    }

    //MECANUM WHEEL DRIVE
    private void drive(double forward, double right, double rotate) {
        telemetry.addData("Forward", forward);
        telemetry.addData("Right", right);
        telemetry.addData("Rotate", rotate);
        telemetry.update();

        //formula
        frontRightWheel.setPower(forward - right - rotate);
        frontLeftWheel.setPower(forward + right + rotate);
        backRightWheel.setPower(forward + right - rotate);
        backLeftWheel.setPower(forward - right + rotate);
    }
}