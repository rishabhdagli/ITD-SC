package opmode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name="Crush Sample Auto Align")
public class CrushSampleAutoAlign extends LinearOpMode{
    private CrushSampleAnglePipeline pipeline;
    private VisionPortal VP;
    private FtcDashboard dash;
    private MultipleTelemetry tele;

    public static double Angle,turner = 0;

    Gamepad lastg;

    Servo wrist,arm,hand,claw,turret;

    @Override
    public void runOpMode() throws InterruptedException {
        lastg = new Gamepad();



        wrist = hardwareMap.get(Servo.class, "Servo6"); //wrist
        arm = hardwareMap.get(Servo.class, "Servo7"); //arm
        hand = hardwareMap.get(Servo.class, "Servo8"); //hand
        claw = hardwareMap.get(Servo.class, "Servo9");//claw
        turret = hardwareMap.get(Servo.class, "Servo10");//turret

        // Initialize the pipeline
        pipeline = new CrushSampleAnglePipeline();
        dash = FtcDashboard.getInstance();
        dash.getInstance().startCameraStream(pipeline,0);
        // Create the VisionPortal with the pipeline
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

        waitForStart();

        while (opModeIsActive()) {
            lastg.copy(gamepad1);
            turret.setPosition(0.47);
            arm.setPosition(0.566);
            wrist.setPosition(0.6521);
            claw.setPosition(0.7);
            double offsetAngle = -272.72727 * (hand.getPosition()) + 226.36364;
            if(!gamepad1.a && lastg.a) {
                hand.setPosition(-0.00366666667 * (pipeline.getDetectedAngle() - offsetAngle) + 0.83);
            }

            telemetry.addData("Angle", pipeline.getDetectedAngle());
            telemetry.update();
        }
    }

}
