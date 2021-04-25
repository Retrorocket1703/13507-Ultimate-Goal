package org.firstinspires.ftc.teamcode.Testing;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTCLib.RingPipelineNewt;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(group = "Beta")
public class SimpleCameraOnly extends LinearOpMode {

    private RingPipelineNewt pipeline1;
    private RingPipelineNewt pipeline2;

    OpenCvCamera webcam;
    OpenCvCamera FrontCamera;


    @Override
    public void runOpMode() throws InterruptedException {

        //Camera initialization
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

            FrontCamera.setPipeline(pipeline2 = new RingPipelineNewt());
            webcam.setPipeline(pipeline1 = new RingPipelineNewt());

            FrontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Bottom Average", pipeline1.getBottomAverage());
            telemetry.addData("Top Average", pipeline1.getTopAverage());
            telemetry.addData("Threshold", pipeline1.getThreshold());

            telemetry.addData("Bottom Average 2", pipeline2.getBottomAverage());
            telemetry.addData("Top Average 2", pipeline2.getTopAverage());
            telemetry.addData("Threshold 2", pipeline2.getThreshold());
            telemetry.update();

        }
    }
}
