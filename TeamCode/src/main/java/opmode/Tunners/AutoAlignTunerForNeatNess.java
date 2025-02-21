package opmode.Tunners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import Subsystems.Boxtube;
import opmode.Vision.CrushSampleAnglePipelineTurretTrial;

@Config
@Disabled
@TeleOp(name = "Neat AutoAlign")
public class AutoAlignTunerForNeatNess extends LinearOpMode {

    // Vision Pipeline and Dashboard
    private CrushSampleAnglePipelineTurretTrial pipeline;
    private VisionPortal visionPortal;
    private FtcDashboard dashboard;
    private MultipleTelemetry tele;

    // Boxtube Subsystem
    private Boxtube boxtube;

    // Servos
    private Servo wrist, arm1, arm2, hand, claw, turret;

    // Default positions
    public static double Wrist = 0.7, Arms = 0.6, Claw = 0.8;
    public static int state = 0;
    private int prevState = 0;

    // Parameter classes for tuning
    public static class BoxtubeParam {
        public double kpExtension = 0;
        public double errorY = 0;
        public double middleLine = 240;
    }

    public static class TurretParams {
        public double turret = 0.5;
        public double mStandard = 0.003;
        public double bStandard = 0.2;
        public double turretAngle = 90;
        public double aGain = 0;
        public double bGain = 0;
        public double cGain = 0;
        public double error = 0;
        public double servoGain = 0;
    }

    public static class HandParams {
        public double hand = 0.5;
        public double mStandard = 0.0035;
        public double bStandard = 0.185;
        public double handAngle = 90;
    }

    public static TurretParams tp = new TurretParams();
    public static HandParams hp = new HandParams();
    public static BoxtubeParam bp = new BoxtubeParam();

    /**
     * Sets the arm servo position.
     *
     * @param pos The desired position.
     */
    public void setArmPosition(double pos) {
        arm1.setPosition(pos);
        // Uncomment below if using a second arm servo:
        // arm2.setPosition(pos);
    }

    /**
     * Regulates the servo value to be within [0, 1]. Note: If values exceed 1, a modulo-like
     * operation is applied.
     *
     * @param x The input value.
     * @return The regulated value.
     */
    public double servoRegulator(double x) {
        return (x > 1) ? (((int) (x * 100)) % 100) / 100.0 : x;
    }

    /**
     * Adjusts an angle to be within 0 to 180 degrees.
     *
     * @param x The input angle.
     * @return The adjusted angle.
     */
    public double handPerpendicularRegulator(double x) {
        if (x >= 0 && x <= 180) {
            return x;
        } else if (x < 0) {
            return x + 180;
        } else {
            return x - 180;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems and dashboard
        boxtube = new Boxtube(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Map servos
        wrist = hardwareMap.get(Servo.class, "Servo6");   // wrist
        arm1 = hardwareMap.get(Servo.class, "Servo7");    // arm
        hand = hardwareMap.get(Servo.class, "Servo8");    // hand
        claw = hardwareMap.get(Servo.class, "Servo9");    // claw
        turret = hardwareMap.get(Servo.class, "Servo10"); // turret

        // Initialize vision pipeline and camera stream
        pipeline = new CrushSampleAnglePipelineTurretTrial();
        dashboard.startCameraStream(pipeline, 0);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

        /*
         * Instructions:
         * 1. Turret:
         *    a. At 12 o'clock, the turret position should be 0.5.
         *    b. Use a sample point to interpolate.
         * 2. Servo Hand Interpolation:
         *    a. Choose one side as the "head" and set it to 12 o'clock (90°).
         *    b. Move the servo clockwise until the head reaches 3 o'clock (0°).
         */

        // Initialization loop: configure interpolation states
        while (opModeInInit()) {  // Replace with "while (!isStarted() && !isStopRequested())" if needed
            wrist.setPosition(Wrist);
            setArmPosition(Arms);
            claw.setPosition(Claw);

            switch (state) {
                // Turret Interpolation
                case 1:
                    tele.addLine("Turret Interpolation: Ensure turret is straight; re-zero if needed.");
                    tele.addLine("(0° is 3 o'clock) Adjust mStandard and bStandard in TurretParams using Desmos.");
                    tele.addLine("Switch to state 2 for Hand Interpolation.");
                    turret.setPosition(tp.turret);
                    break;
                // Hand Interpolation
                case 2:
                    tele.addLine("Hand Interpolation: Position the servo to 12 o'clock on the desired side.");
                    tele.addLine("(0° is 3 o'clock) Adjust mStandard and bStandard in HandParams using Desmos.");
                    tele.addLine("Switch to state 3 for Inverse Kinematics.");
                    hand.setPosition(hp.hand);
                    break;
                // Inverse Kinematics Check
                case 3:
                    tele.addLine("Inverse Kinematics: Adjust turret and hand angles accordingly.");
                    turret.setPosition(servoRegulator(tp.mStandard * tp.turretAngle + tp.bStandard));
                    hand.setPosition(servoRegulator(hp.mStandard * (hp.handAngle - tp.turretAngle + 90) + hp.bStandard));
                    tele.addLine("Click Play to start.");
                    break;
                default:
                    tele.addLine("Switch to state 1 for interpolation setup or press Play for Vision setup.");
                    tele.addLine("Ensure servos are properly positioned.");
                    break;
            }
            tele.update();
        } // End of initialization loop

        waitForStart();
        state = 0; // Reset state after start

        // Main loop
        while (opModeIsActive()) {
            wrist.setPosition(Wrist);
            setArmPosition(Arms);
            claw.setPosition(Claw);

            switch (state) {
                // Turret Interpolation Test
                case 1:
                    tele.addData("Pipeline Angle", pipeline.getDetectedAngle());
                    tele.addLine("Testing turret interpolation. Switch to state 2 for hand alignment.");
                    hp.handAngle = 0;
                    prevState = 0;
                    hand.setPosition(servoRegulator(hp.mStandard * (hp.handAngle - tp.turretAngle + 90) + hp.bStandard));
                    turret.setPosition(servoRegulator(tp.mStandard * tp.turretAngle + tp.bStandard));
                    break;
                // Hand Interpolation Test
                case 2:
                    if (prevState == 0) {
                        tele.addLine("Switch back to test or move to turret alignment.");
                        // +90 accounts for the reverse 0 and perpendicular adjustment
                        hp.handAngle = handPerpendicularRegulator(pipeline.getDetectedAngle() + 90);
                        hand.setPosition(servoRegulator(hp.mStandard * (hp.handAngle - tp.turretAngle + 90) + hp.bStandard));
                        turret.setPosition(servoRegulator(tp.mStandard * tp.turretAngle + tp.bStandard));
                        prevState = 1;
                    }
                    break;
                // Gain Interpolator (Linear/Quadratic)
                case 3:
                    tele.addLine("Using gain interpolator: x = error (pixels), y = servo gain.");
                    tele.addData("Middle Line X", pipeline.getMiddleLineX());
                    tp.error = 320 - pipeline.getMiddleLineX();
                    if (tp.error > 0) {
                        turret.setPosition(turret.getPosition() + tp.servoGain);
                    } else if (tp.error < 0) {
                        turret.setPosition(turret.getPosition() - tp.servoGain);
                    }
                    hp.handAngle = 90;
                    break;
                // Turret Test by Moving the Sample
                case 4:
                    tele.addLine("Test: Move the sample around.");
                    tp.servoGain = tp.aGain * (tp.error * tp.error) + tp.bGain * tp.error + tp.cGain;
                    tele.addData("Middle Line X", pipeline.getMiddleLineX());
                    tp.error = 320 - pipeline.getMiddleLineX();
                    tele.addData("Error", tp.error);
                    if (tp.error > 0) {
                        turret.setPosition(turret.getPosition() + tp.servoGain);
                    } else if (tp.error < 0) {
                        turret.setPosition(turret.getPosition() - tp.servoGain);
                    }
                    break;
                // Boxtube Alignment
                case 5:
                    tele.addLine("Align boxtube: Ensure wrist and hand are in proper pickup position.");
                    tele.addData("Middle Line", bp.middleLine);
                    bp.errorY = pipeline.getMiddleLineY() - bp.middleLine;
                    tele.addData("Pixel Error", bp.errorY);
                    tele.addData("Motor Power", bp.kpExtension * bp.errorY);
                    boxtube.ExtensionPower(bp.kpExtension * bp.errorY);
                    break;
                // Move Everything to Next Position
                case 6:
                    tele.addLine("Moving to the next position.");
                    boxtube.ExtensionPower(0);
                    turret.setPosition(0.47);
                    hand.setPosition(0.5);
                    prevState = 0;
                    break;
                // Combined Boxtube and Turret Alignment
                case 7:
                    tele.addLine("Align boxtube and turret. Ensure wrist and hand are in proper pickup position.");
                    tele.addData("Middle Line", bp.middleLine);
                    bp.errorY = pipeline.getMiddleLineY() - bp.middleLine;
                    tele.addData("Pixel Error", bp.errorY);
                    tele.addData("Motor Power", bp.kpExtension * bp.errorY);
                    boxtube.ExtensionPower(bp.kpExtension * bp.errorY);

                    tele.addLine("Adjust turret by moving the sample.");
                    tp.servoGain = tp.aGain * (tp.error * tp.error) + tp.bGain * tp.error + tp.cGain;
                    tele.addData("Middle Line X", pipeline.getMiddleLineX());
                    tp.error = 320 - pipeline.getMiddleLineX();
                    tele.addData("Error", tp.error);
                    if (tp.error > 0) {
                        turret.setPosition(turret.getPosition() + tp.servoGain);
                    } else if (tp.error < 0) {
                        turret.setPosition(turret.getPosition() - tp.servoGain);
                    }
                    if (Math.abs(bp.errorY) < 50) {
                        // Adjust for pickup position
                        tp.servoGain = tp.aGain * (tp.error * tp.error) + tp.bGain * tp.error + tp.cGain;
                        tele.addData("Middle Line X", pipeline.getMiddleLineX());
                        tp.error = 320 - pipeline.getMiddleLineX();
                        tele.addData("Error", tp.error);
                        if (tp.error > 0) {
                            turret.setPosition(turret.getPosition() + tp.servoGain);
                        } else if (tp.error < 0) {
                            turret.setPosition(turret.getPosition() - tp.servoGain);
                        }
                    }
                    if (prevState == 0 && Math.abs(bp.errorY) < 10) {
                        // Calculate turret angle from current servo position
                        tp.turretAngle = (turret.getPosition() - tp.bStandard) / tp.mStandard;
                        // Adjust hand angle (accounts for reverse and perpendicular offset)
                        hp.handAngle = handPerpendicularRegulator(pipeline.getDetectedAngle() + 90);
                        hand.setPosition(servoRegulator(hp.mStandard * (hp.handAngle - tp.turretAngle + 90) + hp.bStandard));
                        prevState = 1;
                    }
                    break;
                default:
                    tele.addLine("Vision Setup: Switch to state 1 for pixel-to-angle mapping.");
                    break;
            }
            tele.update();
        } // End of main loop
    }
}
