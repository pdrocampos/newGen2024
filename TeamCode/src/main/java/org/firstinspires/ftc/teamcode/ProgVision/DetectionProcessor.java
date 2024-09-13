package org.firstinspires.ftc.teamcode.ProgVision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.RolexCore;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class DetectionProcessor implements VisionProcessor {
    //O mano detector.
    public enum Detection {
        LEFT,
        MIDDLE,
        RIGHT
    }
    //O tipo de aliança passada para o construtor.
    private final RolexCore.AllianceType ALLIANCE_TYPE;
    //As matrizes, que trabalham fazendo uma média dos pixeis da cor alvo.
    private final Mat testMat = new Mat();
    private final Mat lowMat = new Mat();
    private final Mat highMat = new Mat();
    private final Mat finalMat = new Mat();
    //Estes serão os retângulos que representam as áreas escaneadas pra detectar objetos.
    private final Rect LEFT_RECTANGLE;
    private final Rect RIGHT_RECTANGLE;
    //Constantes da cor vermelha.
    private final double RED_THRESHOLD = 0.5;
    private final Scalar LOW_HSV_RED_LOWER = new Scalar(0, 100, 20);
    private final Scalar LOW_HSV_RED_UPPER = new Scalar(10, 255, 255);
    private final Scalar HIGH_HSV_RED_LOWER = new Scalar(160, 100, 20);
    private final Scalar HIGH_HSV_RED_UPPER = new Scalar(180, 255, 255);

    //constantes da cor azul.
    private final double BLUE_THRESHOLD = 0.2;
    private final Scalar LOW_HSV_BLUE_LOWER = new Scalar(100, 100, 20);
    private final Scalar LOW_HSV_BLUE_UPPER = new Scalar(130, 255, 255);
    private final Scalar HIGH_HSV_BLUE_LOWER = new Scalar(220, 100, 20);
    private final Scalar HIGH_HSV_BLUE_UPPER = new Scalar(250, 255, 255);

    public Detection detectedSide = Detection.LEFT;
    //Calibração da câmera
    private CameraCalibration cameraCalibration;

    public DetectionProcessor(RolexCore.AllianceType allianceType) {

        this.ALLIANCE_TYPE = allianceType;

        // Init rectangles
        // Red
        LEFT_RECTANGLE = allianceType == RolexCore.AllianceType.RED ? new Rect(
                // left top corner
                new Point(37, 260),
                // right bottom corner
                new Point(200, 400)

                // Blue
        ) : new Rect(
                new Point(40, 270),
                new Point(210, 405)
        );

        // Red
        RIGHT_RECTANGLE = allianceType == RolexCore.AllianceType.RED ? new Rect(
                new Point(437, 260),
                new Point(600, 400)

                // Blue
        ) : new Rect(
                new Point(440, 270),
                new Point(610, 405)
        );
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //Dá pra adicionar futuramente um código pra leitura de distância
        cameraCalibration = calibration;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        if (this.ALLIANCE_TYPE == RolexCore.AllianceType.RED) {
            Core.inRange(testMat, LOW_HSV_RED_LOWER, LOW_HSV_RED_UPPER, lowMat);
            Core.inRange(testMat, HIGH_HSV_RED_LOWER, HIGH_HSV_RED_UPPER, highMat);
        } else {
            Core.inRange(testMat, LOW_HSV_BLUE_LOWER, LOW_HSV_BLUE_UPPER, lowMat);
            Core.inRange(testMat, HIGH_HSV_BLUE_LOWER, HIGH_HSV_BLUE_UPPER, highMat);
        }


        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255;

        double allianceBasedThreshold = ALLIANCE_TYPE == RolexCore.AllianceType.RED ? RED_THRESHOLD : BLUE_THRESHOLD;

        if (averagedLeftBox > allianceBasedThreshold) {
            detectedSide = Detection.LEFT;
        } else if (averagedRightBox > allianceBasedThreshold){
            detectedSide = Detection.RIGHT;
        } else {
            detectedSide = Detection.MIDDLE;
        }

        // comment out during competition
//        finalMat.copyTo(frame);

        return null;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(2);

        RectF leftRect = new RectF(LEFT_RECTANGLE.x * scaleBmpPxToCanvasPx, LEFT_RECTANGLE.y * scaleBmpPxToCanvasPx, (LEFT_RECTANGLE.x + LEFT_RECTANGLE.width) * scaleBmpPxToCanvasPx, (LEFT_RECTANGLE.y + LEFT_RECTANGLE.height) * scaleBmpPxToCanvasPx);
        canvas.drawRect(leftRect, paint);

        paint.setColor(Color.BLUE);
        RectF rightRect = new RectF(RIGHT_RECTANGLE.x * scaleBmpPxToCanvasPx, RIGHT_RECTANGLE.y * scaleBmpPxToCanvasPx, (RIGHT_RECTANGLE.x + RIGHT_RECTANGLE.width) * scaleBmpPxToCanvasPx, (RIGHT_RECTANGLE.y + RIGHT_RECTANGLE.height) * scaleBmpPxToCanvasPx);
        canvas.drawRect(rightRect, paint);
    }
    public Detection getDetectedSide() {
        return this.detectedSide;
    }
}
