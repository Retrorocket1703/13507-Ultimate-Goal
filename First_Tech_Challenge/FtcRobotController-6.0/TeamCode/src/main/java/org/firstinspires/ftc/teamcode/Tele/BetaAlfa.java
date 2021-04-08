package org.firstinspires.ftc.teamcode.Tele;

import android.service.autofill.FieldClassification;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;

import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGRectRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.advanced.TeleOpAlignWithPoint;
import org.firstinspires.ftc.teamcode.drive.advanced.TeleOpAugmentedDriving;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.FTCLib.UGBasicHighGoalPipeline;


import com.acmerobotics.roadrunner.control.PIDFController;

import java.lang.annotation.Target;

//@Config
@TeleOp(group = "Beta")
public class BetaAlfa extends LinearOpMode {

    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT,
        AUTOMATIC_CONTROL,
        ALIGN_TO_GOAL
    }
    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(10, 10);

    //Custom Code
    private DcMotor flyWheel = null;
    private DcMotor Intake = null;
    private Servo wobbleClamp = null;
    private CRServo wobbleLift = null;
    private DcMotor Tread = null;
    private Servo intakeLift = null;
    private Servo TreadGate = null;
    private DistanceSensor wobbleDistance = null;

    private UGBasicHighGoalPipeline pipeline;
    private UGContourRingPipeline ringPipeline;

    ColorSensor color_sensor;
    TouchSensor wobbleTop;
    ColorSensor wobbleSensor;

    OpenCvCamera webcam;
    OpenCvCamera FrontCamera;

    private Servo TransferArm = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Initialization step: ", "Booting Cameras");
        telemetry.update();

        //New Camera Init
         {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                    .splitLayoutForMultipleViewports(
                            cameraMonitorViewId, //The container we're splitting
                            2, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

            FrontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[0]);
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[1]);

            FrontCamera.openCameraDevice();
            webcam.openCameraDevice();

            FrontCamera.setPipeline(ringPipeline = new UGContourRingPipeline());
            webcam.setPipeline(pipeline = new UGBasicHighGoalPipeline());

            FrontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }




        telemetry.addData("Initialization step: ", "Loading RoadRunner");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(PoseStorage.currentPose);

        headingController.setInputBounds(-Math.PI, Math.PI);

        //Custom

        telemetry.addData("Initialization step: ", "Loading Custom Properties");
        telemetry.update();
        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotor.class, "intake");
        Tread = hardwareMap.get(DcMotor.class, "Tread");
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Tread.setDirection(DcMotor.Direction.REVERSE);

        TransferArm = hardwareMap.get(Servo.class, "transferArm");
        wobbleClamp = hardwareMap.get(Servo.class, "wobbleClamp");
        wobbleLift = hardwareMap.get(CRServo.class, "wobbleLift");
        intakeLift = hardwareMap.get(Servo.class, "intakeLift");
        TreadGate = hardwareMap.get(Servo.class, "treadGate");


        color_sensor = hardwareMap.colorSensor.get("sensor");
        wobbleTop = hardwareMap.touchSensor.get("wobbleTop");
        wobbleSensor = hardwareMap.colorSensor.get("wobbleSensor");
        wobbleDistance = hardwareMap.get(DistanceSensor.class, "wobbleSensor");

        double speedArc = .3;
        double speedArcStorage = .5;
        boolean diskAvailable = false;
        boolean overrideAll = true; //Todo: Reset this

        boolean CollectGoal = false; //If we should collect the goal
        boolean PrimeCollector = false; //If we should prime the collector
        boolean Moving = false; //If the collector is moving
        boolean LifterStowed = true; //If the collector is stowed
        boolean PreReset = false; //If the timer is reset
        boolean PreReset2 = false;
        boolean PreReset3 = false;
        boolean AutoDeploy = true; //If we should autodeploy
        boolean LowerLifter = false; //If we should put the collector down //Old Variables
        boolean ManOverride = false;
        boolean GoalReady = false;
        boolean ManOverride2 = false;
        boolean DropGoal = false;
        int Mode = 0;

        boolean isDiskPrimed = false;
        int DisksInStowage = 0;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime MatchTimer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();

        // drive.setPoseEstimate(PoseStorage.currentPose);
        telemetry.addLine("Waiting for start");
        telemetry.addData("BetaAlfa Version : ", "1.01");
        telemetry.update();




        waitForStart();

        if (isStopRequested()) return;
        //Pose2d poseEstimate = drive.getPoseEstimate();
        timer.reset();
        MatchTimer.reset();
        timer2. reset();
        timer3.reset();
        TreadGate.setPosition(.7);

        Pose2d poseEstimate = drive.getPoseEstimate();

        Vector2d targetAVector = new Vector2d(15, 15);






        while (opModeIsActive() && !isStopRequested()) {
            //RoadRunner During-Run initialization steps
            poseEstimate = drive.getPoseEstimate();

            if(gamepad1.right_bumper) {
                Mode = 0;
            }

            else if (gamepad1.left_bumper) {
                Mode = 1;
                Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                        .splineTo(targetAVector, 15)
                        .build();

                drive.followTrajectoryAsync(traj1);
            }

            else if (gamepad1.dpad_up) {
                //drive.turn(Math.toRadians(20 - poseEstimate.getHeading()));
                TurnToAngleAuto(drive, poseEstimate);

            }


            if(Mode == 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speedArc,
                                -gamepad1.left_stick_x * speedArc,
                                -gamepad1.right_stick_x * speedArc
                        )
                );
            } /*else if (Mode == 1){

                Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                        .splineTo(targetAVector, 15)
                        .build();

                drive.followTrajectoryAsync(traj1);
            }*/

            headingController.update(poseEstimate.getHeading());

            drive.update();
            //End Roadrunner Stuff

            //Automatic pick up

            if(ringPipeline.getHeight() != UGContourRingPipeline.Height.ZERO){
                Intake.setPower(-1);
                Tread.setPower(1);
                isDiskPrimed = true;
                timer2.reset();
                speedArc = .3;
            }
            else{
                speedArc = speedArcStorage;
            }
            if(isDiskPrimed == true && timer2.milliseconds() < 800){
                Intake.setPower(-1);
                Tread.setPower(1);
            } else {
                Tread.setPower(-gamepad2.right_stick_y);
                if(gamepad2.right_stick_y != 0) {
                    Intake.setPower(gamepad2.right_stick_y);
                } else {
                    Intake.setPower(-.2);
                }
            }


            //End automatic pickup



            /** MANUAL OVERRIDES AND CONTROLS*/
            //Keep the fly wheel spinning at half speed
            flyWheel.setPower(.8);



            if(gamepad2.dpad_left || gamepad1.dpad_left)
                AutoFire(timer4, drive);
                //fullSalvoAuto(true, timer2, drive);

            if(gamepad1.a) {
                speedArc = .3;
                speedArcStorage = .3;
            }
            else if(gamepad1.b) {
                speedArc = .5;
                speedArcStorage = .5;
            }
            else if (gamepad1.y) {
                speedArc = .7;
                speedArcStorage = .7;
            }
            else if (gamepad1.x) {
                speedArc = 1;
                speedArcStorage = 1;
            }

            if(gamepad2.a){
                intakeLift.setPosition(.8);

            } else if (gamepad2.b){
                intakeLift.setPosition(.1);
            }
            if(gamepad2.left_bumper){
                TreadGate.setPosition(.1);
            } else if (gamepad2.right_bumper){
                TreadGate.setPosition(.7);
            }

            if(gamepad2.dpad_up){
                TreadGate.setPosition(.4);
                Tread.setPower(1);
            } else {
                TreadGate.setPosition(.7);
            }

            //Random Telemetry Data

            {
                if(pipeline.isRedVisible()) {
                    telemetry.addData("Img X Pos", pipeline.getRedRect().x);
                }
            telemetry.addData("Distance (cm)", wobbleDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Time:", timer.milliseconds());
            telemetry.addData("red", wobbleSensor.red());
            telemetry.addData("green", wobbleSensor.green());
            telemetry.addData("blue", wobbleSensor.blue());
            telemetry.addData("alpha", wobbleSensor.alpha());
            telemetry.addData("argb", wobbleSensor.argb());
            telemetry.addData("Img Pos", pipeline.getCenterofRect(pipeline.getRedRect()));
            telemetry.addData("Img Size", pipeline.getRedRect());
            telemetry.update();
            }

            /*if(DropGoal == false) {
                if (PrimeCollector == false && GoalReady != true && (gamepad2.y || MatchTimer.seconds() > 80) && DropGoal != true) { //Check if we need to start deploying the lifter
                    PrimeCollector = true;
                }

                if (LifterStowed != false && PrimeCollector == true && DropGoal != true) { //If we need to prime the lifter, do so
                    if (PreReset != true) { // If the Timer hasn't been reset, we do so, but only once
                        timer.reset();
                        PreReset = true;
                    }

                    wobbleClamp.setPosition(0); //Open the claw

                    if (timer.milliseconds() < 2000) //Lift the wobble lifter out of storage
                        wobbleLift.setPower(-.6);
                    else if (timer.milliseconds() > 2000 && timer.milliseconds() < 3500) //Put it back down
                        wobbleLift.setPower(.6);
                    else {
                        wobbleLift.setPower(0);
                        PrimeCollector = false;
                        CollectGoal = true;
                        LifterStowed = false;
                    }
                }
                if (gamepad2.x)
                    ManOverride = true;
                if ((CollectGoal == true && (wobbleSensor.red() > 200 && wobbleSensor.blue() > 65 && wobbleSensor.green() > 65 && wobbleSensor.alpha() > 300)) || (ManOverride == true && CollectGoal == true && DropGoal != true)) {

                    wobbleClamp.setPosition(.8); //Clamp

                    if (PreReset2 != true) {
                        timer.reset();
                        PreReset2 = true;
                    }

                    if (timer.milliseconds() > 500 && timer.milliseconds() < 4000) //Wait for 500 millisecs, then lift for 2500 millisecs
                        wobbleLift.setPower(-.6);
                    else if (wobbleTop.isPressed() == true || timer.milliseconds() > 4000) {
                        wobbleLift.setPower(0);
                        GoalReady = true;
                        CollectGoal = false;
                    }

                }
            }
            if(wobbleTop.isPressed())
                GoalReady = true;
            telemetry.addData("Goal Ready: ", GoalReady);

            if(gamepad2.y && GoalReady == true){
                ManOverride2 = true;
                DropGoal = true;
            }


            if(GoalReady == true && ManOverride2){

                wobbleClamp.setPosition(.1); //Drop

                if(PreReset3 != true){
                    timer.reset();
                    PreReset3 = true;
                }

                if(timer.milliseconds() > 1000 && timer.milliseconds() < 4000)

                    wobbleLift.setPower(.6);

                else if (timer.milliseconds() > 4000){

                    wobbleLift.setPower(0);

                    PreReset = false; //If the timer is reset
                    PreReset2 = false;
                    PreReset3 = false;
                    GoalReady = false;
                    ManOverride = false;
                    ManOverride2 = false;
                    DropGoal = false;
                    LifterStowed = false;
                    PrimeCollector = false;
                    CollectGoal = true;
                    LifterStowed = false;
                }

            }

            //End experimental*/

        }
    }

    public void AutoFire(ElapsedTime timer, SampleMecanumDrive drive) {
        double ResolvedX;
        int TargetX = 175;
        boolean ReadyToFire = false;
        double ResolvedAccuracy = .02;

        flyWheel.setPower(.9);
        Tread.setPower(0);
        Intake.setPower(-.2);
        TreadGate.setPosition(.7);

        if(pipeline.isRedVisible()){

            ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
            timer.reset();
            while((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 1000 && gamepad1.dpad_right != true && gamepad2.dpad_right != true) {
                while ((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 1000 && gamepad1.dpad_right != true && gamepad2.dpad_right != true) {
                    telemetry.addData("Aligning", "");
                    telemetry.update();
                    ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
                    drive.setMotorPowers(ResolvedX, ResolvedX, -ResolvedX, -ResolvedX);
                }
            }

            drive.setMotorPowers(0, 0, 0, 0);

            ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);

            if(ResolvedX > -ResolvedAccuracy && ResolvedX < ResolvedAccuracy || timer.milliseconds() > 900){
                ReadyToFire = true;
            }
        }

        timer.reset();

        while(timer.milliseconds() < 100  && gamepad2.dpad_right != true){
            //Hold until flywheel spins up
            telemetry.addData("Spooling up...", "");
            telemetry.update();
        }

        while (isStopRequested() != true && ReadyToFire == true){

            telemetry.addData("Firing", ": Yes");
            TreadGate.setPosition(.4);
            Tread.setPower(1);
            if(gamepad1.dpad_right || gamepad2.dpad_right){
                ReadyToFire = false;
            }
            telemetry.update();
        }

        flyWheel.setPower(.9);
        Tread.setPower(0);
        TreadGate.setPosition(.7);
        timer.reset();
    }

    /*public int fullSalvoAuto(boolean isDiskAvailable, ElapsedTime timer, SampleMecanumDrive drive) {

        boolean Cancel = false;
        double ANGLE = 5;
        double ResolvedDistance = 0;
        int TargetDistance = 175;
        int Accuracy = 5;
        boolean ReadyToFire = true;

        flyWheel.setPower(.85);
        Tread.setPower(0);
        Intake.setPower(-.2);

        if(pipeline.isRedVisible()){
            timer.reset();

            while(Cancel == false && pipeline.isRedVisible() && (pipeline.getCenterofRect(pipeline.getRedRect()).x >= (TargetDistance + Accuracy) || pipeline.getCenterofRect(pipeline.getRedRect()).x <= (TargetDistance - Accuracy)) && isStopRequested() != true) {
                Center(timer, drive, ResolvedDistance, TargetDistance, Accuracy, ReadyToFire, Cancel);
                telemetry.addData("Resolved Distance: ", ResolvedDistance);
                telemetry.update();
                if((ResolvedDistance < .07 && ResolvedDistance > -.07) || gamepad1.dpad_right || timer.milliseconds() < 2000){
                    Cancel = true;
                }
            }
            //drive.setMotorPowers(0, 0, 0, 0);
            if(pipeline.isRedVisible() && (pipeline.getCenterofRect(pipeline.getRedRect()).x >= (TargetDistance + Accuracy) || pipeline.getCenterofRect(pipeline.getRedRect()).x <= (TargetDistance - Accuracy)))
                Center(timer, drive, ResolvedDistance, TargetDistance, Accuracy, ReadyToFire, Cancel);



            if(pipeline.isRedVisible() && (pipeline.getCenterofRect(pipeline.getRedRect()).x <= (TargetDistance + Accuracy) || pipeline.getCenterofRect(pipeline.getRedRect()).x >= (TargetDistance - Accuracy)))
                ReadyToFire = true;

            timer.reset();
            while(timer.milliseconds() < 500) {
                //Hold until fly wheel has spun up
                telemetry.addData("HOLDING", timer.milliseconds());
            }



            //OPEN FIRE
            while(ReadyToFire == true && isDiskAvailable == true && isStopRequested() != true) {
                telemetry.addData("Firing", ": Yes");
                TreadGate.setPosition(.1);
                Tread.setPower(1);
                if(gamepad1.dpad_right){
                    ReadyToFire = false;
                }
                telemetry.update();
            }
            flyWheel.setPower(0);
            Tread.setPower(0);
            TreadGate.setPosition(.7);

        }

        return 1;

    }

    public void Center(ElapsedTime timer, SampleMecanumDrive drive, double ResolvedDistance, int TargetDistance, int Accuracy, boolean ReadyToFire, boolean Cancel){

        timer.reset();
        /*while((pipeline.isRedVisible() && (pipeline.getCenterofRect(pipeline.getRedRect()).x >= (TargetDistance + Accuracy) || pipeline.getCenterofRect(pipeline.getRedRect()).x <= (TargetDistance - Accuracy)) && isStopRequested() != true && Cancel != true)) {
                while ((pipeline.isRedVisible() && (pipeline.getCenterofRect(pipeline.getRedRect()).x >= (TargetDistance + Accuracy) || pipeline.getCenterofRect(pipeline.getRedRect()).x <= (TargetDistance - Accuracy)) && isStopRequested() != true && Cancel != true)) {
                    //Calculate the Resolved Distance
                    ResolvedDistance = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetDistance);
                    //Set motor powers equal to Resolved Distance
                    drive.setMotorPowers(-ResolvedDistance, -ResolvedDistance, ResolvedDistance, ResolvedDistance);
                    //Display information about Pipeline
                    telemetry.addData("Resolved Distance:", ResolvedDistance);
                    telemetry.addData("x", pipeline.getCenterofRect(pipeline.getRedRect()).x);
                    telemetry.update();
                    //Set Ready To Fire to True
                    ReadyToFire = true;
                    if ((ResolvedDistance < .07 && ResolvedDistance > -.07) || gamepad1.dpad_right) {
                        Cancel = true;
                    }
                    telemetry.addData("Resolved Distance: ", ResolvedDistance);
                    telemetry.update();

                }
            }

        drive.setMotorPowers(0, 0, 0, 0); // Old Auto Fire Methods

    }*/ //Old

    public void TurnToAngleAuto(SampleMecanumDrive drive, Pose2d poseEstimate){

        int targetAngle = 10;
        int Accuracy = 2;
        double ResolvedAngle;

        ResolvedAngle = .005 * (Math.toDegrees(poseEstimate.getHeading()) - targetAngle);

        while(poseEstimate.getHeading() > Math.toRadians(targetAngle) + Accuracy || poseEstimate.getHeading() < Math.toRadians(targetAngle) - Accuracy && isStopRequested() != true){
            while(poseEstimate.getHeading() > Math.toRadians(targetAngle) + Accuracy || poseEstimate.getHeading() < Math.toRadians(targetAngle) - Accuracy && isStopRequested() != true) {
                drive.setMotorPowers(ResolvedAngle, ResolvedAngle, -ResolvedAngle, -ResolvedAngle);
            }
        }

    }

}