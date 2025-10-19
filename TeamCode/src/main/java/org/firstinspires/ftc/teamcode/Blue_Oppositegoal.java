package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Blue_Oppositegoal Auto")

public class Blue_Oppositegoal extends Blue_Short {

    private WebcamName camera;

    private DcMotor frontRightWheel, frontLeftWheel, backRightWheel, backLeftWheel;

    private DcMotor leftLauncher, rightLauncher;
    private DcMotor intakeStage1, intakeStage3;
    private CRServo intakeStage2;

    boolean endgame = false;

    double SHORT_SHOT = 0.3;
    double LONG_SHOT = 0.75;

    DriveActions driver = new DriveActions (frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
            intakeStage1, intakeStage2, intakeStage3, leftLauncher, rightLauncher, camera);

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
        intakeStage1 = hardwareMap.get(DcMotor.class, "stage1");
        intakeStage2 = hardwareMap.get(CRServo.class, "stage2");
        intakeStage3 = hardwareMap.get(DcMotor.class, "stage3");

        leftLauncher.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.REVERSE);
        intakeStage1.setDirection(DcMotor.Direction.REVERSE);
        intakeStage2.setDirection(CRServo.Direction.FORWARD);
        intakeStage3.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(629.0, 629.0, 320.0, 240.0)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor) //add created tag processor
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();

        waitForStart();

        if (opModeIsActive()) {
            driver.drive(6, 0, 45, 1000); //positive rotation is clockwise, go to ball pickup location
            driver.intake(2000);
            driver.drive(36, 0, 0, 1000);
            sleep(100);
            driver.drive(0, 0, 100, 1000); //let camera see apriltag for positioning
            driver.adjust(0, -24, -45); //adjust to shooting position
            SHOOT();
            driver.adjust(-4, 36, -45, 1000); //realign with balls
          
            driver.drive(0, 0, 180, 0); //second set of balls
            driver.intake(2000);
            driver.drive(30, 0, 0, 1000); 
            sleep(100);
            driver.drive(0, 0, 110, 1000);
            driver.adjust(0, -24, -45);
            SHOOT();
            driver.adjust(-4, 36, -45, 1000);
          
            driver.drive(0, 0, 180, 0); //third set of balls
            driver.intake(2000);
            driver.drive(54, 0, 0, 1000); 
            sleep(100);
            driver.drive(0, 0, 120, 1000);
            driver.adjust(0, -24, -45);
            SHOOT();
        }
    }
}
