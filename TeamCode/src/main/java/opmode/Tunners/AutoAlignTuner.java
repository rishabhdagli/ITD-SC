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

    public static double Wrist,Arms,Hand,Claw;

    public static int State = 0;

    public static class TurretParams{ public double Turret=0.5,MStandard = 0,BStandard=0,TurretAngle= 90;}

    public static class HandParams{ public double Hand=0.5,MStandard = 0,BStandard=0,HandAngle = 90;}

    public void Arm(double pos){
        arm1.setPosition(pos);
        arm2.setPosition(pos);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        TurretParams tp = new TurretParams();
        HandParams hp = new HandParams();

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
            wrist.setPosition(Wrist);
            Arm(Arms);
            claw.setPosition(Claw);
switch (State){
    //Turrent Interplot
    case 1:
        tele.addLine("Turret Interpolation: Make sure turret is straight or close else re-zero");
        tele.addLine("(0 degrees is 3oclock)" +
                " Change MStanard and BStandard in turret based on interPLUT " +
                "(x is angle y is turret position) on desmos");
        tele.addLine("Switch to state 2 to continue Hand Interpolation");
        turret.setPosition(tp.Turret);
        break;
        //Hand Interpolation
        case 2:
        tele.addLine("Hand Interpolation: Pick the side where the servo is at set it to 12oClock");
        tele.addLine("(0 degrees is 3oclock)" +
                " Change MStanard and BStanard in hand based on interPLUT" +
                "(x is angle y is hand position) on desmos");
        tele.addLine("Switch to 3 inverse kinematics");
            hand.setPosition(hp.Hand);
            //Inverse Kinematics check
    case 3:
        tele.addLine("Move turret Angle and Hand Angle (check if turret angle change hand pos)");
        turret.setPosition(tp.MStandard*(tp.TurretAngle)+tp.BStandard);
        hand.setPosition(hp.MStandard*(hp.HandAngle - tp.TurretAngle) + hp.BStandard);
        tele.addLine("Switch to 3 inverse kinematics");
        break;
    case 4:
        break;

    default:
       tele.addLine("Pls switch to state 1 to continue with the Interpolation setup else continue (play) to vision Setup" +
               "Also make sure the servos don't break");
}
tele.update();
        }

        waitForStart();


        while (opModeIsActive()) {


        }
    }

}
