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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.FTCLib.UGBasicHighGoalPipeline;
import com.qualcomm.robotcore.hardware.ColorSensor;

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

        drive.setPoseEstimate(new Pose2d(-60, -25, 180));

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime MatchTimer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        ElapsedTime timer3 = new ElapsedTime();
        ElapsedTime timer4 = new ElapsedTime();

        //drive.intakeLift.setPosition(.1);
        //Below is what drops the intake
        //drive.intakeLift.setPosition(.9);

        /** GENERIC MOVEMENT */

        Trajectory trajGeneric = drive.trajectoryBuilder(drive.getPoseEstimate())
                //Move to the starter stack at 140 degrees
                .splineToSplineHeading(new Pose2d(-30, -20, 140), 0)
                //Run code to detect the height of the starter stack and store it
                .addDisplacementMarker(() -> {
                    HeightStorage = detectColorHeight();
                    drive.flyWheel.setPower(.8);
                })
                //Move to firing Solution for powershots
                .splineToSplineHeading(new Pose2d(-10, -5, 180), 90)
                //Start firing for Powershots
                .addDisplacementMarker(() -> {
                    //TODO: Repair this to turn correctly
                    drive.TreadGate.setPosition(.4);
                    drive.Tread.setPower(.5);
                })
                .build();

        /** HEIGHT ZERO */

        Trajectory trajHeightZERO = drive.trajectoryBuilder(trajGeneric.end())
                //Drive to wobble goal delivery
                .splineToSplineHeading(new Pose2d(10, -55, 0), 180)
                //Drop the first wobble goal
                .addDisplacementMarker(() -> {
                    //TODO: Test this
                    drive.wobbleClamp.setPosition(1);
                })
                //Move to collect second wobble goal
                .splineToSplineHeading(new Pose2d(-40, -40, -50), 180)
                //Pickup the second wobble goal
                .addDisplacementMarker(() -> {
                    //TODO: Test this
                    drive.wobbleClamp.setPosition(0);
                })
                //Move to second wobble goal delivery
                .splineToSplineHeading(new Pose2d(0, -50), 90)
                //Drop second wobble goal
                .addDisplacementMarker(() -> {
                    //TODO: Test this
                    drive.wobbleClamp.setPosition(1);
                })
                //Move to sit on line
                .splineToSplineHeading(new Pose2d(10, -20, 0), 0)
                .build();

        /** HEIGHT ONE */

        Trajectory trajHeightONE = drive.trajectoryBuilder(trajGeneric.end())
                //Drive to wobble goal delivery
                .splineToSplineHeading(new Pose2d(40, -30, 0), 0)
                //Drop first wobble goal
                .addDisplacementMarker(() -> {
                    //TODO: Insert code to drop wobble goal
                    //TODO: setup intake system
                    drive.flyWheel.setPower(.8);
                })
                //Drive to pickup disk
                .splineToSplineHeading(new Pose2d(-20, -30, 180), 180)
                //Activate autofire
                .addDisplacementMarker(() -> {
                    AutoFire(timer, drive, drive.getPoseEstimate());
                })
                //Move to pickup second wobble goal
                .splineToSplineHeading(new Pose2d(-40, -45, -50), 180)
                //Pickup second wobble goal
                .addDisplacementMarker(() -> {
                    //TODO: Insert code to pickup second wobble goal
                })
                //Drive to wobble goal delivery zone
                .splineToSplineHeading(new Pose2d(30, -25, 50), 180)
                //Drop second wobble goal in zone
                .addDisplacementMarker(() -> {
                    //TODO: Insert code to drop second wobble goal
                })
                //Drive to rest on line
                .splineToSplineHeading(new Pose2d(10, -25, 0), 0)
                .build();

        /** HEIGHT FOUR */

        Trajectory trajHeightFOUR =  drive.trajectoryBuilder(trajGeneric.end())
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
                .build();


        telemetry.addLine("Finished computing, waiting for start");
        telemetry.addData("CharlieAlfa Version : ", "1.03");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        drive.TreadGate.setPosition(.7);

        drive.followTrajectory(trajGeneric);

        if(HeightStorage == 0){
            drive.followTrajectory(trajHeightZERO);
        } else if (HeightStorage == 1){
            drive.followTrajectory(trajHeightONE);
        } else if (HeightStorage == 4) {
            drive.followTrajectory(trajHeightFOUR);
        }


    }

    public int detectColorHeight(){
        if((High_Sensor.red() > 180 && High_Sensor.green() > 90 && High_Sensor.blue() > 30))
            return 4;
        else if ((Low_Sensor.red() > 180 && Low_Sensor.green() > 90 && Low_Sensor.blue() > 30))
            return 1;
        else
            return 0;
    }

    /** AUTOMATIC SALVO SYSTEMS */
    public void AutoFire(ElapsedTime timer, SampleMecanumDrive drive, Pose2d poseEstimate) {
        double ResolvedX;
        int TargetX = 180;
        boolean ReadyToFire = false;
        double ResolvedAccuracy = 0.02;

        telemetry.update();

        drive.Tread.setPower(0.0);
        drive.Intake.setPower(-0.2);
        drive.TreadGate.setPosition(0.7);

        poseEstimate = drive.getPoseEstimate();

        timer.reset();

        telemetry.update();

        if(pipeline.isRedVisible()){

            ResolvedX = 0.005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
            timer.reset();
            while((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 1000 && gamepad1.dpad_right != true && gamepad2.dpad_right != true) {
                while ((ResolvedX < -ResolvedAccuracy || ResolvedX > ResolvedAccuracy) && isStopRequested() != true && timer.milliseconds() < 1000 && gamepad1.dpad_right != true && gamepad2.dpad_right != true) {
                    telemetry.addData("Aligning", "");
                    telemetry.update();
                    ResolvedX = .005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
                    drive.setMotorPowers(ResolvedX, ResolvedX, -ResolvedX, -ResolvedX);
                }
            }
            telemetry.addData("Flywheel Voltage ", drive.flyWheel.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            drive.setMotorPowers(0, 0, 0, 0);

            ResolvedX = 0.005 * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);

            if(ResolvedX > -ResolvedAccuracy && ResolvedX < ResolvedAccuracy || timer.milliseconds() > 900){
                ReadyToFire = true;
            }
        }

        timer.reset();

        while (isStopRequested() != true && ReadyToFire == true && timer.milliseconds() < 3000){

            telemetry.addData("Flywheel Voltage ", drive.flyWheel.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Firing", ": Yes");
            drive.TreadGate.setPosition(0.4);
            drive.Tread.setPower(.6);
            if(gamepad1.dpad_right || gamepad2.dpad_right){
                ReadyToFire = false;
            }
            telemetry.update();
        }

        //drive.flyWheel.setPower(.9);
        //drive.flyWheel.setVelocity(180, AngleUnit.DEGREES);
        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.7);
        timer.reset();
    }
}
