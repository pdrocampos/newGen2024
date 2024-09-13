package org.firstinspires.ftc.teamcode.ProgVision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RolexCore;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Vision Test", group = "Vision")
public class Camera_Test extends LinearOpMode {
    DetectionProcessor detectionProcessor;

    private VisionPortal portal;
    @Override
    public void runOpMode(){
        detectionProcessor = new DetectionProcessor(RolexCore.AllianceType.BLUE);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .addProcessor(detectionProcessor)
                .build();

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Detected", detectionProcessor.getDetectedSide());
            telemetry.update();
        }
    }
}