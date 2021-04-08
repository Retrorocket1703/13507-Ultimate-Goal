package org.firstinspires.ftc.teamcode.Autonomous;

import android.service.autofill.FieldClassification;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;

import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGRectRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Power;
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

@Autonomous(group = "Beta")
public class BetaCharlie extends LinearOpMode {

    private UGBasicHighGoalPipeline pipeline;
    private UGContourRingPipeline ringPipeline;

    public static double DISTANCE = 60;

    //TouchSensor wobbleTop;
    //ColorSensor wobbleSensor;

    OpenCvCamera webcam;
    OpenCvCamera FrontCamera;

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

        boolean isDiskPrimed = false;
        int DisksInStowage = 0;
        int HeightStorage = 4;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        Pose2d startPose = new Pose2d(-60, -25, 0);

        drive.setPoseEstimate(startPose);

        Trajectory GenericStartTraj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-30, -20), -45)
                .build();

        Trajectory TrajHeightZero = drive.trajectoryBuilder(GenericStartTraj.end())
                .splineToLinearHeading(new Pose2d(10, -45, Math.toRadians(90)), Math.toRadians(0))
                //.splineTo(new Vector2d(10, -45), 0)
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()

                    // Run your action in here!
                })
                .splineTo(new Vector2d(-10, -30), 180)
                .addDisplacementMarker(() -> {
                    // This marker runs after the second splineTo()

                    // Run your action in here!
                })
                .splineTo(new Vector2d(-50, -40), 0)
                .addDisplacementMarker(() -> {
                    // This marker runs after the second splineTo()

                    // Run your action in here!
                })
                .splineTo(new Vector2d(0, -50), 60)
                .build();


        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectory(GenericStartTraj);

        if(ringPipeline.getHeight() == UGContourRingPipeline.Height.ZERO)
            HeightStorage = 0;
        else if (ringPipeline.getHeight() == UGContourRingPipeline.Height.ONE)
            HeightStorage = 1;
        else if (ringPipeline.getHeight() == UGContourRingPipeline.Height.FOUR)
            HeightStorage = 4;

        if(HeightStorage == 0)
            drive.followTrajectory(TrajHeightZero);



        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("Height", HeightStorage);
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();


        while (!isStopRequested() && opModeIsActive()) ;

    }

    public void AutoFire(ElapsedTime timer, SampleMecanumDrive drive) {
        double ResolvedX;
        int TargetX = 165;
        boolean ReadyToFire = false;
        double ResolvedAccuracy = .02;

        drive.flyWheel.setPower(.65);
        drive.Tread.setPower(0);
        drive.Intake.setPower(-.2);
        drive.TreadGate.setPosition(.7);

        if (pipeline.isRedVisible()) {

            ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
            timer.reset();

            while ((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 2000 && gamepad1.dpad_right != true) {
                telemetry.addData("Aligning", "");
                telemetry.update();
                ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
                drive.setMotorPowers(-ResolvedX, -ResolvedX, ResolvedX, ResolvedX);
            }

            drive.setMotorPowers(0, 0, 0, 0);

            ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);

            if (ResolvedX > -ResolvedAccuracy && ResolvedX < ResolvedAccuracy || timer.milliseconds() > 1900) {
                ReadyToFire = true;
            }
        }

        timer.reset();

        while (timer.milliseconds() < 1000) {
            //Hold until flywheel spins up
            telemetry.addData("Spooling up...", "");
            telemetry.update();
        }

        timer.reset();

        while (isStopRequested() != true && ReadyToFire == true && timer.milliseconds() < 2000) {

            telemetry.addData("Firing", ": Yes");
            drive.TreadGate.setPosition(.1);
            drive.Tread.setPower(1);
            if (gamepad1.dpad_right) {
                ReadyToFire = false;
            }
            telemetry.update();
        }

        drive.flyWheel.setPower(.5);
        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.7);
    }

    public void PowerShotsAuto(ElapsedTime timer, SampleMecanumDrive drive, int TargetX) {
        double ResolvedX;
        boolean ReadyToFire = false;
        double ResolvedAccuracy = .02;

        drive.flyWheel.setPower(1);
        drive.Tread.setPower(0);
        drive.Intake.setPower(-.2);
        drive.TreadGate.setPosition(.7);

        if (pipeline.isRedVisible()) {

            ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
            timer.reset();
            while ((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 2000 && gamepad1.dpad_right != true) {
                while ((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 2000 && gamepad1.dpad_right != true) {
                    telemetry.addData("Aligning", "");
                    telemetry.update();
                    ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
                    drive.setMotorPowers(-ResolvedX, -ResolvedX, ResolvedX, ResolvedX);
                }
            }

            drive.setMotorPowers(0, 0, 0, 0);

            ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);

            if (ResolvedX > -ResolvedAccuracy && ResolvedX < ResolvedAccuracy || timer.milliseconds() > 1900) {
                ReadyToFire = true;
            }
        }

        timer.reset();

        while (timer.milliseconds() < 1000) {
            //Hold until flywheel spins up
            telemetry.addData("Spooling up...", "");
            telemetry.update();
        }

        timer.reset();

        while (isStopRequested() != true && ReadyToFire == true && timer.milliseconds() < 2000) {

            telemetry.addData("Firing", ": Yes");
            drive.TreadGate.setPosition(.1);
            drive.Tread.setPower(1);
            if (gamepad1.dpad_right) {
                ReadyToFire = false;
            }
            telemetry.update();
        }

        drive.flyWheel.setPower(.5);
        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.7);
    }


}

/****** BONEYARD ******
 *
 *         /*Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
 *                 .forward(20)
 *                 .build();

 //drive.followTrajectory(trajectory);

 //AutoFire(timer, drive);
 //PowerShotsAuto(timer, drive, 200);


 //This is a random spline, and is not particularly optimized
 /*Trajectory traj = drive.trajectoryBuilder(new Pose2d())
 .splineTo(new Vector2d(30, 30), 0)
 .build();

 drive.followTrajectory(traj);

timer.reset();
while(timer.milliseconds() < 5)*/