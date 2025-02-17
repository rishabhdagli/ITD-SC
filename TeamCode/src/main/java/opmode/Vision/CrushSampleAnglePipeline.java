package opmode.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

public class CrushSampleAnglePipeline implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private Bitmap bitmap;

    private Mat hsvMask = new Mat();
    private Mat processedMask = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();
    private double detectedAngle = 0.0;

    private boolean detectRed = true;
    private boolean detectBlue = true;
    private boolean detectYellow = true;

    // Define HSV color ranges for red, blue, and yellow
    private final Scalar lowerRed = new Scalar(0, 100, 100);
    private final Scalar upperRed = new Scalar(10, 255, 255);
    private final Scalar lowerBlue = new Scalar(100, 100, 100);
    private final Scalar upperBlue = new Scalar(140, 255, 255);
    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public Bitmap getLastFrame() {
        return lastFrame.get();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);

        // Convert to HSV color space
        Imgproc.cvtColor(frame, hsvMask, Imgproc.COLOR_RGB2HSV);

        // Create masks for each color
        Mat maskRed = new Mat();
        Mat maskBlue = new Mat();
        Mat maskYellow = new Mat();

        if (detectRed) Core.inRange(hsvMask, lowerRed, upperRed, maskRed);
        if (detectBlue) Core.inRange(hsvMask, lowerBlue, upperBlue, maskBlue);
        if (detectYellow) Core.inRange(hsvMask, lowerYellow, upperYellow, maskYellow);

        // Combine masks into a single processed mask
        processedMask = new Mat();
        Core.bitwise_or(maskRed, maskBlue, processedMask);
        Core.bitwise_or(processedMask, maskYellow, processedMask);

        // Morphological operations to reduce noise
        Imgproc.morphologyEx(processedMask, processedMask, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.morphologyEx(processedMask, processedMask, Imgproc.MORPH_OPEN, new Mat());

        // Find contours
        contours.clear();
        Imgproc.findContours(processedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Rect boundingBox = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        Point imageCenter = new Point(frame.width() / 2.0, frame.height() / 2.0);

        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            double area = rect.area();
            Point rectCenter = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
            double distance = Math.hypot(rectCenter.x - imageCenter.x, rectCenter.y - imageCenter.y);
            double score = area - (1.5 * distance); // Favoring centrality slightly more than area

            if (score > bestScore) {
                bestScore = score;
                boundingBox = rect;
            }
        }

        if (boundingBox != null) {
            Imgproc.rectangle(frame, boundingBox.tl(), boundingBox.br(), new Scalar(255, 0, 0), 2);
            detectedAngle = calculateDominantAngle(processedMask, boundingBox);
        }

        Imgproc.putText(frame, "Angle: " + detectedAngle, new Point(50, 50), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
        Utils.matToBitmap(frame, bitmap);
        lastFrame.set(bitmap);

        return null;
    }

    /**
     * Calculates the dominant angle of lines detected within the mask (using the bounding box region as context).
     * The method uses the probabilistic Hough Transform to detect lines, computes each line's angle,
     * then aggregates lengths per angle. Finally, it selects the angle with the maximum total length.
     *
     * The angle is adjusted to fall within the range 0–180 degrees.
     */
    private double calculateDominantAngle(Mat mask, Rect boundingBox) {
        Mat edges = new Mat();
        Imgproc.Canny(mask, edges, 50, 150);
        Mat lines = new Mat();
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 50, 50, 10);

        Map<Double, Double> angleLengths = new HashMap<>();
        double dominantAngle = 0.0;

        if (lines.rows() > 0) {
            for (int i = 0; i < lines.rows(); i++) {
                double[] line = lines.get(i, 0);
                double x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

                // Compute the angle in degrees using atan2
                double angle = Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));
                // Shift negative angles into the 0–180 range
                if (angle < 0) {
                    angle += 180;
                }
                // Optionally, if you prefer to force the value within [0,180) (since 180° is equivalent to 0° for lines)
                // you could add: if (angle >= 180) { angle -= 180; }
                angle = Math.round(angle);
                double length = Math.hypot(x2 - x1, y2 - y1);

                // Sum up the lengths of lines having this angle
                angleLengths.put(angle, angleLengths.getOrDefault(angle, 0.0) + length);
            }

            dominantAngle = angleLengths.entrySet().stream()
                    .max((a, b) -> Double.compare(a.getValue(), b.getValue()))
                    .map(Map.Entry::getKey)
                    .orElse(0.0);
        }

        return dominantAngle;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

    public double getDetectedAngle() {
        return detectedAngle;
    }

    public void setColorDetection(boolean detectRed, boolean detectBlue, boolean detectYellow) {
        this.detectRed = detectRed;
        this.detectBlue = detectBlue;
        this.detectYellow = detectYellow;
    }
}
