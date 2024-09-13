package org.firstinspires.ftc.teamcode.ProgVision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ProgVision.DetectionProcessor;
import org.firstinspires.ftc.teamcode.RolexCore;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "TestingVisionMotor", group = "VisionMotor")
public class TestingForMotorUsingVision extends LinearOpMode {

    private DcMotorEx motorLF;
    DetectionProcessor detectionProcessor;
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLF = hardwareMap.get(DcMotorEx.class, "motorLeftFront");

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
            if(detectionProcessor.getDetectedSide() == DetectionProcessor.Detection.LEFT){
                motorLF.setPower(1);
            }
            telemetry.addData("Detected", detectionProcessor.getDetectedSide());
            telemetry.update();
        }
    }
}
