package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTCLib.RingPipelineNewt;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(group = "Beta")
public class SimpleColorOnly extends LinearOpMode {

    ColorSensor Low_Sensor;
    ColorSensor High_Sensor;

    @Override
    public void runOpMode() throws InterruptedException {

        Low_Sensor = hardwareMap.colorSensor.get("LowSensor");
        High_Sensor = hardwareMap.colorSensor.get("HighSensor");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {


            telemetry.addLine("------ Low Sensor -----");
            telemetry.addData("Low Sensor ARGB ", Low_Sensor.argb());
            telemetry.addData("Low Sensor Red", Low_Sensor.red());
            telemetry.addData("Low Sensor Green", Low_Sensor.green());
            telemetry.addData("Low Sensor Blue", Low_Sensor.blue());
            telemetry.addLine("------ High Sensor -----");
            telemetry.addData("High Sensor ARGB ", High_Sensor.argb());
            telemetry.addData("High Sensor Red", High_Sensor.red());
            telemetry.addData("High Sensor Green", High_Sensor.green());
            telemetry.addData("High Sensor Blue", High_Sensor.blue());
            telemetry.addLine("------ Other -----");
            telemetry.addData("Guessed height ", detectColorHeight());

            telemetry.update();

        }
    }

    public int detectColorHeight(){
        if((High_Sensor.red() > 38 && High_Sensor.green() > 46 && High_Sensor.blue() > 38))
            return 4;
        else if ((Low_Sensor.red() > 38 && Low_Sensor.green() > 46 && Low_Sensor.blue() > 38))
            return 1;
        else
            return 0;
    }
}
