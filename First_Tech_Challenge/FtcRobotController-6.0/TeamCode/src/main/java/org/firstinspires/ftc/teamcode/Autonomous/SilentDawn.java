 package org.firstinspires.ftc.teamcode.Autonomous;

 import com.acmerobotics.roadrunner.geometry.Pose2d;
 import com.acmerobotics.roadrunner.trajectory.Trajectory;
 import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
 import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.ColorSensor;
 import com.qualcomm.robotcore.hardware.DistanceSensor;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
 import org.firstinspires.ftc.teamcode.FTCLib.UGBasicHighGoalPipeline;
 import org.firstinspires.ftc.teamcode.drive.DriveConstants;
 import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
 import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
 import org.openftc.easyopencv.OpenCvCamera;
 import org.openftc.easyopencv.OpenCvCameraFactory;
 import org.openftc.easyopencv.OpenCvCameraRotation;

 import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

 @Autonomous(group = "Charlie")
 public class SilentDawn extends LinearOpMode{

     private UGBasicHighGoalPipeline pipeline;
     private UGContourRingPipeline ringPipeline;

     OpenCvCamera webcam;
     OpenCvCamera FrontCamera;

     ColorSensor Low_Sensor;
     ColorSensor High_Sensor;

     private DistanceSensor rangeRear;
     private DistanceSensor rangeSide;

     private int HeightStorage = 0;
     private int HeightStorage2 = 0;

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
         rangeRear = hardwareMap.get(DistanceSensor.class, "rangeRear");
         rangeSide = hardwareMap.get(DistanceSensor.class, "rangeSide");

         telemetry.addData("Initialization step: ", "Loading RoadRunner");
         telemetry.update();
         SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

         //drive.setPoseEstimate(new Pose2d(-63, -25, Math.toRadians(180))); // -60, -25
         drive.setPoseEstimate(new Pose2d(3.5, 0, 0));

         drive.wobbleClamp.setPosition(0);
         drive.wobbleRotate.setPosition(1);
         drive.TreadGate.setPosition(.65);

         ElapsedTime timer = new ElapsedTime();
         ElapsedTime MatchTimer = new ElapsedTime();
         ElapsedTime timer2 = new ElapsedTime();
         ElapsedTime timer3 = new ElapsedTime();
         ElapsedTime timer4 = new ElapsedTime();

         int EstimatedDisks = 0;
         boolean OnDisk = false;

         drive.intakeLift.setPosition(1);

        /** GENERIC */
         Trajectory Generic1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                 .lineToLinearHeading(new Pose2d(-60, 0, Math.toRadians(-2)))
                 .build();

         Trajectory Generic2 = drive.trajectoryBuilder(Generic1.end().plus(new Pose2d(0, 0, Math.toRadians(44))))
                 .lineToLinearHeading(new Pose2d(-55, 15, Math.toRadians(0)))
                 .build();

         Trajectory Generic3 = drive.trajectoryBuilder(Generic2.end())
                 .forward(35 , DriveConstants.SLOW_CONSTRAINTS)
                 .addDisplacementMarker(1, () -> {

                     drive.flyWheel.setVelocity(-295, DEGREES);
                 })
                 .build();

         /** HEIGHT ZERO */
         //Drive to first wobble goal delivery
         Trajectory HeightZero1 = drive.trajectoryBuilder(Generic3.end())
                 .lineToLinearHeading(new Pose2d(-70, 30, Math.toRadians(200)))
                 .build();

         //Drive to collect second wobble goal
         Trajectory HeightZero2 = drive.trajectoryBuilder(HeightZero1.end())
                 .splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(180)), Math.toRadians(-180))
                 .build();


         //Drive to deliver second wobble goal part 1
         Trajectory HeightZero3 = drive.trajectoryBuilder(HeightZero2.end())
                 .lineToLinearHeading(new Pose2d(-70, 20, Math.toRadians(200)))
                 .build();


         /** HEIGHT ONE */
         //Drive to first wobble goal delivery
         Trajectory HeightOne1 = drive.trajectoryBuilder(Generic3.end())
                 .lineToLinearHeading(new Pose2d(-90, 10, Math.toRadians(200)))
                 .build();

         //Drive to collect second wobble goal
         Trajectory HeightOne2 = drive.trajectoryBuilder(HeightOne1.end())
                 .splineToLinearHeading(new Pose2d(-42, 0, Math.toRadians(190)), Math.toRadians(-180))
                 .build();


         //Drive to deliver second wobble goal part 1
         Trajectory HeightOne3 = drive.trajectoryBuilder(HeightOne2.end())
                 .lineToLinearHeading(new Pose2d(-70, 0, Math.toRadians(200)))
                 .build();

         //Drive to deliver second wobble goal part 2
         Trajectory HeightOne4 = drive.trajectoryBuilder(HeightOne3.end())
                 .lineToLinearHeading(new Pose2d(-90, 0, Math.toRadians(240)))
                 .build();

         //Drive to rest on line
         Trajectory HeightOne5 = drive.trajectoryBuilder(HeightOne4.end())
                 .lineToLinearHeading(new Pose2d(-70, 0, Math.toRadians(180)))
                 .build();



         /** HEIGHT FOUR */
         //Drive to first wobble goal delivery
         Trajectory HeightFour1 = drive.trajectoryBuilder(Generic3.end())
                 .lineToLinearHeading(new Pose2d(-110, 40, Math.toRadians(200)))
                 .build();

         //Drive to collect second wobble goal
         Trajectory HeightFour2 = drive.trajectoryBuilder(HeightFour1.end())
//                 .splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(150)), Math.toRadians(-180))//, DriveConstants.SLOW_CONSTRAINTS)
                 .splineToLinearHeading(new Pose2d(-42, 0, Math.toRadians(190)), Math.toRadians(-180))
                 .build();

         //Drive to deliver second wobble goal part 1
         Trajectory HeightFour3 = drive.trajectoryBuilder(HeightFour2.end())
                 .lineToLinearHeading(new Pose2d(-70, -10, Math.toRadians(200)))
                 .build();

         //Drive to deliver second wobble goal part 2
         Trajectory HeightFour4 = drive.trajectoryBuilder(HeightFour3.end())
                 .lineToLinearHeading(new Pose2d(-130, 5, Math.toRadians(240)))
                 .build();

         //Drive to rest on line
         Trajectory HeightFour5 = drive.trajectoryBuilder(HeightFour4.end())
                 .lineToLinearHeading(new Pose2d(-70, 30, Math.toRadians(180)))
                 .build();

         telemetry.addLine("Finished computing, waiting for start");
         telemetry.addData("CharlieAlfa Version : ", "1.11");
         telemetry.update();






         waitForStart();

         if (isStopRequested()) return;

         MatchTimer.reset();



         drive.TreadGate.setPosition(.65);
         drive.intakeLift.setPosition(0);
         drive.flyWheel.setVelocity(-280, AngleUnit.DEGREES);
         drive.Intake.setPower(-1);
         drive.Tread.setPower(.5);

         sleep(500);

         drive.followTrajectory(Generic1);

         PowerShotsAuto(timer, drive);

         drive.TreadGate.setPosition(.65);

         drive.followTrajectory(Generic2);

         drive.TreadGate.setPosition(.1);

         drive.Tread.setPower(1);

         drive.followTrajectoryAsync(Generic3);

         //sleep(1000);

         while(drive.isBusy()) {

             if((High_Sensor.red() > 248 && High_Sensor.green() > 197 && High_Sensor.blue() > 112)){

                 HeightStorage += 1;

             }

             if((Low_Sensor.red() > 54)){

                 HeightStorage2 += 1;

             }

             drive.update();

         }

         HeightStorage = (HeightStorage + HeightStorage2) / 2;

         telemetry.addData("Raw Height", HeightStorage);



         if(HeightStorage >= 8)
             HeightStorage = 4;
         else if (HeightStorage >= 2)
             HeightStorage = 1;
         else
             HeightStorage = 0;

         telemetry.addData("Calculated Height", HeightStorage);

         //TODO Yeah
         //HeightStorage = 1;


         if(HeightStorage == 0){

             drive.followTrajectoryAsync(HeightZero1);

             drive.flyWheel.setVelocity(0, DEGREES);
             drive.TreadGate.setPosition(.65);

             DeployLifter(drive, timer, 0);

             while(drive.isBusy() && opModeIsActive()) drive.update();

             DeployLifter(drive, timer, 1);

             sleep(400);

             drive.followTrajectory(HeightZero2);

             //Do fancy crap to align
             goToViaRange(drive, 20.0, 35.0, timer);

             DeployLifter(drive, timer, 2);

             drive.followTrajectory(HeightZero3);

             DeployLifter(drive, timer, 1);


         }
         else if (HeightStorage == 1) {

             drive.followTrajectoryAsync(HeightOne1);

             drive.flyWheel.setVelocity(0, DEGREES);
             drive.TreadGate.setPosition(.65);

             DeployLifter(drive, timer, 0);

             while(drive.isBusy() && opModeIsActive()) drive.update();

             DeployLifter(drive, timer, 1);

             sleep(400);

             drive.followTrajectory(HeightOne2);

             //Do fancy crap to align
             goToViaRange(drive, 20.0, 35.0, timer);

             DeployLifter(drive, timer, 2);

             drive.followTrajectory(HeightOne3);

             drive.followTrajectory(HeightOne4);

             DeployLifter(drive, timer, 1);

             drive.followTrajectory(HeightOne5);


         }
         else if (HeightStorage == 4) {

            drive.followTrajectoryAsync(HeightFour1);

            drive.flyWheel.setVelocity(0, DEGREES);
            drive.TreadGate.setPosition(.65);

            DeployLifter(drive, timer, 0);

            while(drive.isBusy() && opModeIsActive()) drive.update();

            DeployLifter(drive, timer, 1);

            sleep(400);

            drive.followTrajectory(HeightFour2);

            goToViaRange(drive, 20.0, 35.0, timer);

            DeployLifter(drive, timer, 2);

            drive.followTrajectory(HeightFour3);

             drive.followTrajectory(HeightFour4);

            DeployLifter(drive, timer, 1);

            drive.followTrajectory(HeightFour5);


         }



         drive.Intake.setPower(0);
         drive.Tread.setPower(0);
         Pose2d poseEstimate = drive.getPoseEstimate();
         telemetry.addData("Distance Rear", rangeRear.getDistance(DistanceUnit.INCH));
         telemetry.addData("Distance Side", rangeSide.getDistance(DistanceUnit.INCH));
         telemetry.addData("Match time: ", MatchTimer.milliseconds());
         telemetry.addData("Height Estimate: ", HeightStorage);
         telemetry.addData("finalX", poseEstimate.getX());
         telemetry.addData("finalY", poseEstimate.getY());
         telemetry.addData("finalHeading", poseEstimate.getHeading());
         telemetry.update();

         while(opModeIsActive() && isStopRequested() != true);


     }

     /** GO TO SENSOR POS */
     public void goToViaRange(SampleMecanumDrive drive, double rearTarg, double sideTarg, ElapsedTime timer){
         timer.reset();
         while(timer.milliseconds() < 2000) {
             if (rangeRear.getDistance(DistanceUnit.INCH) > rearTarg) {

                 drive.setMotorPowers(-.3, -.3, -.3, -.3);

             } else {
                 drive.setMotorPowers(0, 0, 0, 0);
             }


         }
         timer.reset();
         while(timer.milliseconds() < 2000) {
             if (rangeSide.getDistance(DistanceUnit.INCH) > sideTarg) {

                 drive.setMotorPowers(.3, -.3, .3, -.3);

             } else {
                 drive.setMotorPowers(0, 0, 0, 0);
             }

         }
         drive.setMotorPowers(0, 0, 0, 0);
     }

     /*public int detectColorHeight(){
         //Low: 123, 83, 56
         //High: 400, 255, 155
         //----Empty
         //Low: 37, 46, 42
         //High: 97, 139, 70
         if((Low_Sensor.red() > 54))// && Low_Sensor.green() > 60 && Low_Sensor.blue() > 48))
             return 4;
         else if ((High_Sensor.red() > 248 && High_Sensor.green() > 197 && High_Sensor.blue() > 112))
             return 1;
         else
             return 0;
         //(High_Sensor.red() > 248 && High_Sensor.green() > 197 && High_Sensor.blue() > 112) &&
     }*/

     /** AUTOMATIC SALVO SYSTEMS */
     /*public void AutoFire(ElapsedTime timer, SampleMecanumDrive drive) {
         //VERSION 1.04a-a-s
         //Code: [Current code][Extra code]-[Tele/Auto]-[Experimental/Stable]
         /**
          * Changelog:
          * 1.02e: Added Automatic shutoff via color sensor recognition. Added assorted comments.
          * 1.03e: Removed Automatic shutoff via color sensor. Added wait for start in an attempt to remove first shot misses.
          * 1.03a-s: Working consistently, only change if in error.
          * 1.03b-t-s: Adjusted the speed of the robot during Manual Override
          * 1.03b-a-s: Update for autonomous mode
          * 1.04a-a-s: Updated to new firing system. No longer relies on finding the target.
          *
         double ResolvedX;
         int TargetX = 185;
         boolean ReadyToFire = false;
         double ResolvedAccuracy = 0.01;
         double kV = .006;

         telemetry.update();

         //Turn on assorted systems
         drive.Tread.setPower(0.0);
         drive.Intake.setPower(-0.2);
         drive.TreadGate.setPosition(0.65);
         drive.flyWheel.setVelocity(520, AngleUnit.DEGREES);

         telemetry.update();

         if(pipeline.isRedVisible()){

             ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
             timer.reset();

             while(timer.milliseconds() < 2000 && isStopRequested() != true && opModeIsActive()){
                 telemetry.addLine("Auto Aligning. Press Dpad Right to cancel.");
                 telemetry.addData("Runtime: ",  timer.milliseconds());
                 telemetry.update();
                 ResolvedX = kV * (pipeline.getCenterofRect(pipeline.getRedRect()).x - TargetX);
                 drive.setMotorPowers(ResolvedX, ResolvedX, -ResolvedX, -ResolvedX);
             }

             drive.setMotorPowers(0, 0, 0, 0);
             ReadyToFire = true;

         }

         while(drive.flyWheel.getVelocity(DEGREES) < 510) drive.update();

         ReadyToFire = true;

         timer.reset();
         while (isStopRequested() != true && opModeIsActive() && ReadyToFire == true){

             drive.update();

             telemetry.addData("Flywheel Voltage ", drive.flyWheel.getCurrent(CurrentUnit.AMPS));

             telemetry.addData("Firing", ": Yes");



             //If the flywheel is at the correct velocity, open the gate and fire
             if(drive.flyWheel.getVelocity(DEGREES) > 510){
                 drive.Tread.setPower(1);
                 drive.TreadGate.setPosition(0.4);
             } else {
                 drive.Tread.setPower(0);
             }
             if(timer.milliseconds() > 2300 && HeightStorage == 1)
                 ReadyToFire = false;
             else if (timer.milliseconds() > 4000 && HeightStorage == 4)
                 ReadyToFire = false;

             telemetry.addData("Flywheel Velocity", drive.flyWheel.getVelocity(DEGREES));
             telemetry.addData("Target X Pos", pipeline.getCenterofRect(pipeline.getRedRect()).x);

             telemetry.update();
         }

         drive.Tread.setPower(0);
         drive.TreadGate.setPosition(.65);
         timer.reset();
     }*/

     public void DeployLifter(SampleMecanumDrive drive, ElapsedTime timer, int mode){
         //Mode 0: Initial deployment
         //Mode 1: Drop
         //Mode 2: Pickup

         if(mode == 0){

             timer.reset();
             while(timer.milliseconds() < 1500){
                 drive.wobbleLift.setPower(-.6);
                 drive.update();
             }

             /*timer.reset();
             while(timer.milliseconds() < 1500){
                 drive.wobbleLift.setPower(.6);
                 drive.update();
             }*/
             drive.wobbleRotate.setPosition(.6);
             drive.wobbleLift.setPower(0);

         }
         else if (mode == 1){
             drive.wobbleRotate.setPosition(.6);
             drive.wobbleClamp.setPosition(.35);
         }
         else if (mode == 2){
             drive.wobbleClamp.setPosition(0);
             sleep(300);
             drive.wobbleRotate.setPosition(.8);
         }

     }

     public void PowerShotsAuto(ElapsedTime timer, SampleMecanumDrive drive) {

         //Open the gate and start firing
         drive.TreadGate.setPosition(.65);
         drive.DiskGate.setPosition(.1);
         drive.Tread.setPower(0);




//         drive.TreadGate.setPosition(.1);
//
//         timer.reset();
//         while(timer.milliseconds() < 500){
//             drive.Tread.setPower(.5);
//         }
//         drive.Tread.setPower(0);




         /*timer.reset();

         while(timer.milliseconds() < 2000) {
             while (rangeSide.getDistance(DistanceUnit.INCH) < 40) {
                 drive.setMotorPowers(-.3, -.3, .3, .3);
             }

             while (rangeSide.getDistance(DistanceUnit.INCH) > 40) {
                 drive.setMotorPowers(.3, .3, -.3, -.3);
             }

             drive.setMotorPowers(0, 0, 0, 0);
         }

         drive.TreadGate.setPosition(.1);*/

//         while(rangeSide.getDistance(DistanceUnit.INCH) < 40) {
//             drive.setMotorPowers(-.3, -.3, .3, .3);
//         }
//
//         drive.setMotorPowers(0, 0, 0, 0);

         drive.TreadGate.setPosition(.1);

         timer.reset();
         while(timer.milliseconds() < 500){
             drive.Tread.setPower(.5);
         }
         drive.Tread.setPower(0);

         drive.turnAsyncVelControl(Math.toRadians(17), Math.toRadians(21), Math.toRadians(21));

         while(drive.isBusy()) drive.update();



         /*telemetry.addData("Powershot Side Distance Estimate: ", rangeSide.getDistance(DistanceUnit.INCH));
         telemetry.addData("Powershot Rear Distance Estimate: ", rangeRear.getDistance(DistanceUnit.INCH));*/

//         while(rangeSide.getDistance(DistanceUnit.INCH) > 40) {
//             drive.setMotorPowers(.3, .3, -.3, -.3);
//         }


         //drive.setMotorPowers(0, 0, 0, 0);

         timer.reset();
         while(timer.milliseconds() < 500){
             drive.Tread.setPower(.5);
         }
         drive.Tread.setPower(0);

         drive.turnAsyncVelControl(Math.toRadians(18), Math.toRadians(21), Math.toRadians(21));

         while(drive.isBusy()) drive.update();

         /*while(rangeSide.getDistance(DistanceUnit.INCH) < 44) {
             drive.setMotorPowers(-.3, -.3, .3, .3);
         }

         drive.setMotorPowers(0, 0, 0, 0);*/



         timer.reset();
         while(timer.milliseconds() < 1000){
             drive.Tread.setPower(.5);
         }
         drive.Tread.setPower(0);


         /*while(rangeSide.getDistance(DistanceUnit.INCH) < 46 || rangeSide.getDistance(DistanceUnit.INCH) > 48) {
             if(rangeSide.getDistance(DistanceUnit.INCH) < 46 )
                drive.setMotorPowers(-.3, -.3, .3, .3);
             else if (rangeSide.getDistance(DistanceUnit.INCH) > 48)
                 drive.setMotorPowers(.3, .3, -.3, -.3);
         }*/

         /*drive.setMotorPowers(0, 0, 0, 0);

         timer.reset();
         while(timer.milliseconds() < 1000){
             drive.Tread.setPower(.5);
         }
         drive.Tread.setPower(0);*/

         //Turn slowly
         /*drive.turnAsyncVelControl(Math.toRadians(22), Math.toRadians(21), Math.toRadians(21)); //34


         //Wait for the drive to finish
         timer.reset();
         while(drive.isBusy()) {
             drive.update();
             if(timer.milliseconds() > 100 && timer.milliseconds() < 400){
                drive.TreadGate.setPosition(.65);
             }
             else if (timer.milliseconds() > 900 && timer.milliseconds() < 1200){
                 drive.Tread.setPower(.4);
                 drive.TreadGate.setPosition(.1);
             }
             /*else if (timer.milliseconds() > 1300) {
                 drive.TreadGate.setPosition(.65);
             }*
         }*/

         drive.TreadGate.setPosition(.1);

         telemetry.addData("AFTER FIRE Powershot Side Distance Estimate: ", rangeSide.getDistance(DistanceUnit.INCH));
         telemetry.addData("AFTER FIRE Powershot Rear Distance Estimate: ", rangeRear.getDistance(DistanceUnit.INCH));
         sleep(600);

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

    public void DeadFire (ElapsedTime timer, SampleMecanumDrive drive, double flyWheelPower) {
        drive.flyWheel.setVelocity(450, AngleUnit.DEGREES);
        drive.Tread.setPower(0);
        drive.Intake.setPower(-.2);
        drive.TreadGate.setPosition(.65);

        timer.reset();
        while(timer.milliseconds() < 500 && isStopRequested() != true && opModeIsActive());

        drive.TreadGate.setPosition(.1);
        drive.Tread.setPower(1);

        timer.reset();
        while(timer.milliseconds() < 400 && isStopRequested() != true && opModeIsActive());

        drive.Tread.setPower(0);
        drive.TreadGate.setPosition(.65);
    }*/