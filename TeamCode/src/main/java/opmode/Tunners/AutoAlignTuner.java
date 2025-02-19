package opmode.Tunners;

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

import opmode.Vision.CrushSampleAnglePipeline;

@Config
@TeleOp(name="Auto Align Tuner (for 6 sevos) assuming offset")
public class AutoAlignTuner extends LinearOpMode{
    private CrushSampleAnglePipeline pipeline;
    private VisionPortal VP;
    private FtcDashboard dash;
    private MultipleTelemetry tele;

    Servo wrist,arm1,arm2,hand,claw,turret;

    public static double Wrist,Arms,Hand,Claw,Turret=0.5;

    public static int State = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        wrist = hardwareMap.get(Servo.class, "Servo6"); //wrist
        arm1 = hardwareMap.get(Servo.class, "Servo7"); //arm
        arm2 = hardwareMap.get(Servo.class, "Servo8"); //arm
        hand = hardwareMap.get(Servo.class, "Servo9"); //hand
        claw = hardwareMap.get(Servo.class, "Servo10");//claw
        turret = hardwareMap.get(Servo.class, "Servo11");//turret

        // Initialize the pipeline
        pipeline = new CrushSampleAnglePipeline();
        dash = FtcDashboard.getInstance();
        dash.getInstance().startCameraStream(pipeline,0);
        // Create the VisionPortal with the pipeline
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);


        /*
        1.Turret
        1a. The turrent at 12oclock should be 0.5
        1b. use random point to interpolate
        2. Interpolate Servo Hand - Proper way decribed in crush sample auto align
        2a. Choose one side as head move to 12oclock (90 deg)
        2b. move servo to the clock wise till head at 3 o clock (0 deg)


         */


        while (opModeInInit()){
switch (State){
    case 1:
        tele.addLine("Turret Interpolation: Make sure turret is straight or close else re-zero");
        turret.setPosition(Turret);

        break;

    default:
       tele.addLine("Pls switch to state 1 to continue");
}
tele.update();
        }

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
