package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class runcode extends LinearOpMode {

    DcMotor fL = null;
    DcMotor fR = null;
    DcMotor bL = null;
    DcMotor bR = null;
    DcMotor iT = null;
    DcMotor dM = null;
    DcMotor iM = null;

    Servo a1 = null;
    Servo a2 = null;
    Servo aB = null;
    Servo i1 = null;
    Servo i2 = null;
    Servo hit = null;

    //    NormalizedColorSensor cS;
    TouchSensor tS;
    View relativeLayout;

    float[] hsvValues = new float[3];
    float gain = 4;
    boolean xButtonCurrentlyPressed;
    boolean xButtonPreviouslyPressed;

    AnalogInput potent = null;
    double number = .48;
    double position = .7;
    boolean armLoop = true;
    double runTimer = 0;
    String level = "";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null;
    private WebcamName webcamName = null;

    private boolean targetVisible = false;

    public class Arm extends Thread {
        private Thread t;
        private String threadName;

        Arm(String name) {
            threadName = name;
            System.out.println("Creating" + threadName);
        }

        public void moveArm(double arm1, double arm2) {
            a1.setPosition(arm1);
            a2.setPosition(arm2);

            armLoop = false;
        }
    }

    public class Rail extends Thread{
        private Thread t;
        private String threadName;

        Rail (String name){
            threadName = name;
            System.out.println("Creating" + threadName);
        }

        public void moveRail (int position , double power)
        {
            iT.setTargetPosition(position);
            iT.setPower(.4);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCvCamera webcam;
        VisionCodeStuff.VisionPipeline pipeline = new VisionCodeStuff.VisionPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {

            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addData("Status", "Initualizing");

        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");
        iT = hardwareMap.dcMotor.get("iT");
//        iT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        iM = hardwareMap.dcMotor.get("iM");
        dM = hardwareMap.dcMotor.get("dM");

        a1 = hardwareMap.servo.get("a1");
        a2 = hardwareMap.servo.get("a2");
        aB = hardwareMap.servo.get("aB");
        i1 = hardwareMap.servo.get("i1");
        i2 = hardwareMap.servo.get("i2");
        hit = hardwareMap.servo.get("hit");

//        cS = hardwareMap.get(NormalizedColorSensor.class, "cS");
//        tS = hardwareMap.digitalChannel.get("tS");
        tS = hardwareMap.touchSensor.get("tS");
//        tS = hardwareMap.get(DigitalChannel.class, "sensor_digital");
//        tS.setMode(DigitalChannel.Mode.INPUT);

        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runcode.Arm initial = new runcode.Arm("First");
        initial.moveArm(.73, .27);

        iT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        iT.setTargetPosition(0);
        iT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        iT.setTargetPosition(0);
        iT.setPower(.1);

        aB.setPosition(1);

        boolean touch = tS.isPressed();

        waitForStart();
        //moving to cheese platter
        hit.setPosition(1);
        Thread.sleep(500);

        Forward(700 , .3);
        Thread.sleep(500);

        //Face Warehouse
        TurnLeft(920 , .5);
        Thread.sleep(800);
        StrifeRight(1300 , .3);
        Thread.sleep(300);
        while (opModeIsActive())
        {

            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            if (pipeline.position == VisionCodeStuff.VisionPipeline.thingPos.ONE) {
                sleep(5000);
                webcam.stopStreaming();
                //code to run to zone C
                telemetry.addLine("Placing Low");
                PlaceLow();
                Thread.sleep(500);
                Backward(150 , .3);
                Thread.sleep(1500);
                Drop();
                Thread.sleep(1300);
                Forward(180 , .3);
                Thread.sleep(500);
                Reset();

            }
            if (pipeline.position == VisionCodeStuff.VisionPipeline.thingPos.TWO) {
                sleep(5000);
                webcam.stopStreaming();
                //code to run to zone B
                telemetry.addLine("Placing middle");
                PlaceMiddle();
                Backward(150 , .3);
                Thread.sleep(1500);
                Drop();
                Thread.sleep(1300);
                Forward(180 , .3);
                Thread.sleep(500);
                Reset();

            }
            if (pipeline.position == VisionCodeStuff.VisionPipeline.thingPos.THREE) {
                sleep(5000);
                webcam.stopStreaming();
                //code to run to zone A
                telemetry.addLine("placing high");
                PlaceHigh();
                Backward(180 , .3);
                Thread.sleep(1500);
                Drop();
                Thread.sleep(1300);
                Forward(210 , .3);
                Thread.sleep(500);
                Reset();

            }

        }

    }

    public void PlaceHigh() throws InterruptedException {
        aB.setPosition(.95);

        Thread.sleep(400);

        runcode.Arm movePlease = new runcode.Arm("First");
        movePlease.moveArm(.14 , .86);

        aB.setPosition(.65);

        Thread.sleep(600);

        aB.setPosition(.34);

        Thread.sleep(700);

        runcode.Rail moveR = new runcode.Rail("Second");
        moveR.moveRail(430 , .4);

//                iT.setTargetPosition(400);
//                iT.setPower(.4);

        telemetry.addData("iT position" , iT.getTargetPosition());
    }

    public void PlaceMiddle() throws InterruptedException {
        aB.setPosition(.9);

        Thread.sleep(100);

        runcode.Arm movePlease = new runcode.Arm("First");
        movePlease.moveArm(.32 , .68);

        aB.setPosition(.65);

        Thread.sleep(600);

        aB.setPosition(.52);

        Thread.sleep(700);

        runcode.Rail moveR = new runcode.Rail("Second");
        moveR.moveRail(400 , .4);
    }

    public void PlaceLow() throws InterruptedException {
        aB.setPosition(.9);

        Thread.sleep(100);

        runcode.Arm movePlease = new runcode.Arm("First");
        movePlease.moveArm(.4 , .6);

        aB.setPosition(.65);

        Thread.sleep(700);

        runcode.Rail moveR = new runcode.Rail("Second");
        moveR.moveRail(400 , .4);

        Thread.sleep(400);

        movePlease.moveArm(.5 , .5);
    }

    public void Reset() throws InterruptedException
    {
        runcode.Arm movePlease = new runcode.Arm("First");
        movePlease.moveArm(.4 , .6);

        Thread.sleep(400);

        telemetry.addData("Position" , iT.getCurrentPosition());

        iT.setTargetPosition(0);
        iT.setPower(.4);

        Thread.sleep(400);

        movePlease.moveArm(.73 , .27);

        aB.setPosition(1);
    }

    public void Drop()
    {
        aB.setPosition(.1);
    }

    public void ResetEncoders()
    {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ResetEncodersAll()
    {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RunToPosition() {
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void RunToPositionAll() {
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void NoEncoders() {
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void On(double power)
    {
        fL.setPower(-power);
        fR.setPower(-power);
        bL.setPower(-power);
        bR.setPower(-power);
    }

    public void StrifeRight(int position , double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(-power);
        }
    }

    public void StrifeLeft(int position , double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(position);
        fR.setTargetPosition(-position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(power);
        }
    }

    public void Backward(int position, double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(position);
        bR.setTargetPosition(position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power);
        }
    }

    public void TurnRight(int position, double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(-power);
            bR.setPower(power);
        }
    }

    public void TurnLeft(int position, double power)
    {
        ResetEncodersAll();

        fL.setTargetPosition(position);
        fR.setTargetPosition(-position);
        bL.setTargetPosition(position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy())
        {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(power);
            bR.setPower(-power);
        }
    }

    public void Forward(int position, double power) {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(-position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy()) {
            fL.setPower(-power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(-power);
        }
    }

    public void ScanForward(int position, double power) {
        ResetEncodersAll();

        fL.setTargetPosition(-position);
        fR.setTargetPosition(-position);
        bL.setTargetPosition(-position);
        bR.setTargetPosition(-position);

        RunToPositionAll();

        while (fL.isBusy()) {
            fL.setPower(-power);
            fR.setPower(-power * 1.4);
            bL.setPower(-power);
            bR.setPower(-power);
        }
    }


}
