package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import androidx.core.location.GnssStatusCompat;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.FTCLib.UGBasicHighGoalPipeline;
import com.qualcomm.robotcore.hardware.ColorSensor;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

@Autonomous(group = "Charlie")
public class CharlieAlfa extends LinearOpMode{

    private UGBasicHighGoalPipeline pipeline;
    private UGContourRingPipeline ringPipeline;

    OpenCvCamera webcam;
    OpenCvCamera FrontCamera;

    ColorSensor Low_Sensor;
    ColorSensor High_Sensor;

    private int HeightStorage = 0;

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

        Low_Sensor = hardwareMap.colorSensor.get("LowSensor");
        High_Sensor = hardwareMap.colorSensor.get("HighSensor");

        telemetry.addData("Initialization step: ", "Loading RoadRunner");
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-60, -25, Math.toRadians(180))); // -60, -25

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime MatchTimer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();

        //drive.intakeLift.setPosition(.1);
        //Below is what drops the intake
        //drive.intakeLift.setPosition(.9);

        /** GENERIC MOVEMENT */

        Trajectory trajGeneric1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                //Move to the starter stack at 140 degrees
                .lineToLinearHeading(new Pose2d(-10, -22, Math.toRadians(185)))
                //.splineToSplineHeading(new Pose2d(-10, -22, Math.toRadians(180)), Math.toRadians(0)) // -27.4, -26, 207, 0
                .build();

        Trajectory trajGeneric2 = drive.trajectoryBuilder(trajGeneric1.end().plus(new Pose2d(0, 0, 11)))
                //Drive to pickup disks
                .lineToLinearHeading(new Pose2d(-25, -42, Math.toRadians(250)))
                //.splineToSplineHeading(new Pose2d(-25, -42, Math.toRadians(230)), Math.toRadians(0), DriveConstants.SLOW_CONSTRAINTS)
                //.splineToSplineHeading(new Pose2d(-20, -30, Math.toRadians(180)), Math.toRadians(180))
                .build();

        /*Trajectory trajNull1 = drive.trajectoryBuilder(trajGeneric2.end())
                .back(15)
                .build();*/
        Trajectory trajNull2 = drive.trajectoryBuilder(trajGeneric2.end())
                .forward(20)
                .build();
        Trajectory trajNull3 = drive.trajectoryBuilder(trajNull2.end())
                .back(20)
                .build();
        /*Trajectory trajNull4 = drive.trajectoryBuilder(trajNull3.end())
                .forward(15)
                .build();*/

        //Fire powershots

        /** HEIGHT ZERO */

        Trajectory trajHeightZERO = drive.trajectoryBuilder(trajNull3.end())
                //Drive to wobble goal delivery
                .lineToLinearHeading(new Pose2d(5, -55, Math.toRadians(0)))
                //.splineToSplineHeading(new Pose2d(5, -55, Math.toRadians(0)), Math.toRadians(180))
                .build();

        Trajectory trajHeightZERO2 = drive.trajectoryBuilder(trajHeightZERO.end())
                //Move to collect second wobble goal
                .lineToLinearHeading(new Pose2d(-40, -40, Math.toRadians(-50)))
                //.splineToSplineHeading(new Pose2d(-40, -40, Math.toRadians(-50)), Math.toRadians(180))
                .build();

        Trajectory trajHeightZERO3 = drive.trajectoryBuilder(trajHeightZERO2.end())

                //Move to second wobble goal delivery
                .lineToLinearHeading(new Pose2d(0, -55, Math.toRadians(-50)))
                //.splineToSplineHeading(new Pose2d(0, -55, Math.toRadians(-50)), Math.toRadians(90))
                .build();

        Trajectory trajHeightZERO4 = drive.trajectoryBuilder(trajHeightZERO3.end())
                //Move to sit on line
                .lineToLinearHeading(new Pose2d(10, -20, Math.toRadians(0)))
                //.splineToSplineHeading(new Pose2d(10, -20, Math.toRadians(0)), Math.toRadians(0))
                .build();

        /** HEIGHT ONE */

        Trajectory trajHeightONE = drive.trajectoryBuilder(trajNull3.end())
                //Drive to wobble goal delivery
                .splineToSplineHeading(new Pose2d(40, -30, Math.toRadians(0)), Math.toRadians(0))
                //Drop first wobble goal
                .build();

        Trajectory trajHeightONE2 = drive.trajectoryBuilder(trajHeightONE.end())
                //Drive to shoot
                .splineToSplineHeading(new Pose2d(-40, -45, Math.toRadians(-50)), Math.toRadians(180))
                //Activate autofire
                .build();

        Trajectory trajHeightONE3 = drive.trajectoryBuilder(trajHeightONE2.end())
                //Move to pickup second wobble goal
                .splineToSplineHeading(new Pose2d(30, -255, Math.toRadians(50)), Math.toRadians(180))
                //Pickup second wobble goal
                .build();

        Trajectory trajHeightONE4 = drive.trajectoryBuilder(trajHeightONE3.end())
                //Drive to wobble goal delivery zone
                .splineToSplineHeading(new Pose2d(10, -25, Math.toRadians(0)), Math.toRadians(0))
                //Drop second wobble goal in zone
                .build();

        /** HEIGHT FOUR */

        /*Trajectory trajHeightFOUR =  drive.trajectoryBuilder(trajGeneric2.end())
                //Drive to wobble goal delivery zone
                .splineToSplineHeading(new Pose2d(50, -50, 50), 180)
                //Drop first wobble goal
                .addDisplacementMarker(() -> {
                    //TODO: Insert code to drop wobble goal
                    //Turn on flywheel and intake systems
                    drive.flyWheel.setPower(.8);
                    //TODO: setup intake system
                })
                //Drive to pick up disks and firing solution
                .splineToSplineHeading(new Pose2d(-20, -35, 180), 180)
                //Autofire
                .addDisplacementMarker(() -> {
                    //TODO: Quantify autofire if it needs to be
                    AutoFire(timer, drive, drive.getPoseEstimate());
                })
                //Drive to pickup second wobble goal
                .splineToSplineHeading(new Pose2d(-40, -45, -50), 180)
                //Pickup second wobble goal
                .addDisplacementMarker(() -> {
                    //TODO: Insert code to pickup second wobble goal
                    //Probably turn on intake systems
                })
                //Drive to deliver second wobble goal (Possibly pickup remaining disk???)
                .splineToSplineHeading(new Pose2d(50, -50, 50), -90)
                //Drop second wobble goal in target zone
                .addDisplacementMarker(() -> {
                    //TODO: Insert code to drop second wobble goal
                })
                //Move to rest on line
                .splineToSplineHeading(new Pose2d(10, -50, 0), 180)
                .build();*/


        telemetry.addLine("Finished computing, waiting for start");
        telemetry.addData("CharlieAlfa Version : ", "1.08");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        drive.TreadGate.setPosition(.7);

        DeployLifter(drive, timer, 0);

        drive.flyWheel.setVelocity(450, AngleUnit.DEGREES);

        DeployLifter(drive, timer, 1);

        //Move to power shot firing solution
        drive.followTrajectory(trajGeneric1);

        PowerShotsAuto(timer, drive);
        drive.Intake.setPower(-1);
        drive.Tread.setPower(1);

        drive.followTrajectory(trajGeneric2);
        //drive.followTrajectory(trajNull1);
        drive.followTrajectory(trajNull2);
        drive.followTrajectory(trajNull3);
        //drive.followTrajectory(trajNull4);

        /*drive.turn(Math.toRadians(20));
        drive.Intake.setPower(1);
        drive.turn(Math.toRadians(-20));
        drive.Intake.setPower(-1);
        drive.followTrajectory(trajNull1);
        drive.turn(Math.toRadians(10));*/

        drive.Tread.setPower(.6);


        sleep(1400);

        drive.Tread.setPower(0);
        //Detect starter stack
        HeightStorage = detectColorHeight();



        if(HeightStorage == 0){
            //Shut down the Fly wheel because we won't need it
            drive.flyWheel.setPower(0);
            //Move to Zero wobble goal delivery zone
            drive.followTrajectory(trajHeightZERO);
            //Drop first wobble goal
            DeployLifter(drive, timer, 2);
            //Move to second wobble goal pickup
            drive.followTrajectory(trajHeightZERO2);
            //Pickup second wobble goal
            DeployLifter(drive, timer, 1);
            //Move to second wobble goal delivery
            drive.followTrajectory(trajHeightZERO3);
            //Drop second wobble goal
            DeployLifter(drive, timer, 2);
            //Move to sit on line
            drive.followTrajectory(trajHeightZERO4);
        } else if (HeightStorage == 1){
            drive.flyWheel.setVelocity(510, AngleUnit.DEGREES);
            drive.turn(Math.toRadians(-80));
            AutoFire(timer, drive, drive.getPoseEstimate());
            //Move to One wobble goal Delivery zone
            drive.followTrajectory(trajHeightONE);
            drive.flyWheel.setPower(0);
            //Drop first wobble goal
            DeployLifter(drive, timer, 2);
            //Move to second wobble goal pickup
            drive.followTrajectory(trajHeightONE2);
            DeployLifter(drive, timer, 1);
            //Move to pickup second wobble goal
            drive.followTrajectory(trajHeightONE3);
            //Pickup second wobble goal
            DeployLifter(drive, timer, 2);
            //Drive to drop second wobble goal
            //drive.followTrajectory(trajHeightONE4);
            //Drop second wobble goal

            //Drive to sit on line

        } else if (HeightStorage == 4) {

        }

        telemetry.addData("Guessed Height: ", HeightStorage);
        telemetry.update();

        while(opModeIsActive() && isStopRequested() == false);


    }

    public int detectColorHeight(){
        //Low: 123, 83, 56
        //High: 400, 255, 155
        //----Empty
        //Low: 37, 46, 42
        //High: 97, 139, 70
        if((High_Sensor.red() > 248 && High_Sensor.green() > 197 && High_Sensor.blue() > 112) && (Low_Sensor.red() > 80 && Low_Sensor.green() > 64 && Low_Sensor.blue() > 49))
            return 4;
        else if ((High_Sensor.red() > 248 && High_Sensor.green() > 197 && High_Sensor.blue() > 112))
            return 1;
        else
            return 0;
    }

    /** AUTOMATIC SALVO SYSTEMS */
    public void AutoFire(ElapsedTime timer, SampleMecanumDrive drive, Pose2d poseEstimate) {
        //VERSION 1.01
        //Autonomous Version
        double ResolvedX;
        int TargetX = 175;
        boolean ReadyToFire = false;
        double ResolvedAccuracy = 0.02;
        double kV = .004;

        telemetry.update();

        //Turn on assorted systems
        drive.Tread.setPower(0.0);
        drive.Intake.setPower(-0.2);
        drive.TreadGate.setPosition(0.7);

        timer.reset();

        //Is the high goal visible? If yes, continue firing routine.
        if(pipeline.isRedVisible()){

            ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
            timer.reset();
            while((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 1000 && gamepad1.dpad_right != true && gamepad2.dpad_right != true) {
                while ((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 1000 && gamepad1.dpad_right != true && gamepad2.dpad_right != true) {
                    telemetry.addLine("Auto Aligning.");
                    telemetry.update();
                    ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
                    drive.setMotorPowers(ResolvedX, ResolvedX, -ResolvedX, -ResolvedX);
                }
            }

            drive.setMotorPowers(0, 0, 0, 0);

            ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);

            if(ResolvedX > -ResolvedAccuracy && ResolvedX < ResolvedAccuracy || timer.milliseconds() > 950){
                ReadyToFire = true;
            }
        }

        timer.reset();
        while (isStopRequested() != true && opModeIsActive() && ReadyToFire == true){

            telemetry.addData("Flywheel Voltage ", drive.flyWheel.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Firing", ": Yes");
            drive.TreadGate.setPosition(0.4);

            if(drive.flyWheel.getVelocity(DEGREES) > 515 ){
                drive.Tread.setPower(0.6);
            } else {
                drive.Tread.setPower(0);
            }

            if(timer.milliseconds() > 2000 && HeightStorage == 1)
                ReadyToFire = false;
            else if (timer.milliseconds() > 5000 && HeightStorage == 4)
                ReadyToFire = false;

            telemetry.addData("Flywheel Velocity", drive.flyWheel.getVelocity(DEGREES));
            telemetry.addData("Target X Pos", pipeline.getCenterofRect(pipeline.getRedRect()).x);

            telemetry.update();
        }

        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.7);
        timer.reset();
    }

    public void DeployLifter(SampleMecanumDrive drive, ElapsedTime timer, int mode){
        //Mode 0: Initial deployment
        //Mode 1: Pickup
        //Mode 2: Drop
        if(mode == 0){
            drive.wobbleClamp.setPosition(1);

            timer.reset();
            while(timer.milliseconds() < 1500){
                drive.wobbleLift.setPower(.6);
            }

            timer.reset();
            while(timer.milliseconds() < 1500){
                drive.wobbleLift.setPower(-.6);
            }
            drive.wobbleLift.setPower(0);

        } else if (mode == 1){
            drive.wobbleClamp.setPosition(0);

            timer.reset();
            while(timer.milliseconds() < 2000){
                drive.wobbleLift.setPower(.6);
            }
            drive.wobbleLift.setPower(0);
        } else if (mode == 2){
            drive.wobbleClamp.setPosition(1);

            timer.reset();
            while(timer.milliseconds() < 2000){
                drive.wobbleLift.setPower(-.6);
            }

            drive.wobbleLift.setPower(0);
        }
    }

    public void PowerShotsAuto(ElapsedTime timer, SampleMecanumDrive drive) {

        DeadFire(timer, drive, 0);

        drive.turn(Math.toRadians(5));

        DeadFire(timer, drive, 0);

        drive.turn(Math.toRadians(6));

        DeadFire(timer, drive, 0);

        drive.flyWheel.setPower(1);

    }

    public void DeadFire (ElapsedTime timer, SampleMecanumDrive drive, double flyWheelPower) {
        drive.flyWheel.setVelocity(450, AngleUnit.DEGREES);
        drive.Tread.setPower(0);
        drive.Intake.setPower(-.2);
        drive.TreadGate.setPosition(.7);

        timer.reset();
        while(timer.milliseconds() < 500 && isStopRequested() != true && opModeIsActive());

        drive.TreadGate.setPosition(.1);
        drive.Tread.setPower(1);

        timer.reset();
        while(timer.milliseconds() < 400 && isStopRequested() != true && opModeIsActive());

        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.7);
    }
}

/********** BONEYARD **************
 *

 while(drive.isBusy()){
 if(isStopRequested() == true || opModeIsActive() == false){
 drive.cancelFollowing();
 break;
 }
 drive.update();
 telemetry.addData("Pose Estimate ", drive.getPoseEstimate());
 telemetry.addData("Height ", HeightStorage);
 telemetry.update();
 }
 */
