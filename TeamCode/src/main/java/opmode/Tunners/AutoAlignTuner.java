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
import opmode.Vision.CrushSampleAnglePipelineTurretTrial;

@Config
@TeleOp(name="Auto Align Tuner (for 6 sevos) assuming offset")
public class AutoAlignTuner extends LinearOpMode{
    private CrushSampleAnglePipelineTurretTrial pipeline;
    private VisionPortal VP;
    private FtcDashboard dash;
    private MultipleTelemetry tele;


    //aryan
    //charat
    //
    Servo wrist,arm1,arm2,hand,claw,turret;

    public static double Wrist=0.7,Arms=0.6,Claw =0.8;

    public static int State = 0;

    private int prevState = 0;

    public static class TurretParams{ public double Turret=0.5,MStandard = 0.003,BStandard=0.2,TurretAngle= 90;}

    public static class HandParams{ public double Hand=0.5,MStandard = 0.0035,BStandard=0.185,HandAngle = 90;}

    public void Arm(double pos){
        arm1.setPosition(pos);
        //arm2.setPosition(pos);
    }
    //0.003667 , 0.185
    //0.0038889, 0.15
    public double ServoRegulizer(double x) {
        return (x > 1) ? (((int)(x * 100)) % 100) / 100.0 : x;
    }

    public static TurretParams tp = new TurretParams();
   public static HandParams hp = new HandParams();

   public double HandPerpendicularRegulaizer(double x){
       if(0<=x && x<=180){
           return x;
       }
       else if(x < 0){
           return  x+180;
       }
       else {
           return  x - 180;
       }
   }



    @Override
    public void runOpMode() throws InterruptedException {
         dash = FtcDashboard.getInstance();
         tele = new MultipleTelemetry(telemetry, dash.getTelemetry());




        //for new bot

//        wrist = hardwareMap.get(Servo.class, "Servo6"); //wrist
//        arm1 = hardwareMap.get(Servo.class, "Servo7"); //arm
//        arm2 = hardwareMap.get(Servo.class, "Servo8"); //arm
//        hand = hardwareMap.get(Servo.class, "Servo9"); //hand
//        claw = hardwareMap.get(Servo.class, "Servo10");//claw
//        turret = hardwareMap.get(Servo.class, "Servo11");//turret

        wrist = hardwareMap.get(Servo.class, "Servo6"); //wrist
        arm1 = hardwareMap.get(Servo.class, "Servo7"); //arm
        hand = hardwareMap.get(Servo.class, "Servo8"); //hand
        claw = hardwareMap.get(Servo.class, "Servo9");//claw
        turret = hardwareMap.get(Servo.class, "Servo10");//turret

        // Initialize the pipeline
        pipeline = new CrushSampleAnglePipelineTurretTrial();
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
            break;
            //Inverse Kinematics check
    case 3:
        tele.addLine("Move turret Angle and Hand Angle (check if turret angle change hand pos)");
        turret.setPosition(ServoRegulizer(tp.MStandard*(tp.TurretAngle)+tp.BStandard));
        hand.setPosition(ServoRegulizer(hp.MStandard*(hp.HandAngle - tp.TurretAngle+90) + hp.BStandard));
        tele.addLine("Click play");
        break;

    default:
       tele.addLine("Pls switch to state 1 to continue with the Interpolation setup else continue (play) to vision Setup" +
               "Also make sure the servos don't break");
       break;
}
tele.update();
        } //init loop

        waitForStart();
        State = 0;


        while (opModeIsActive()) {
            wrist.setPosition(Wrist);
            Arm(Arms);
            claw.setPosition(Claw);

            turret.setPosition(ServoRegulizer(tp.MStandard*(tp.TurretAngle)+tp.BStandard));

            switch (State){
                //Turrent Interplot
                case 1:
                  tele.addData("pipeline angle: ",pipeline.getDetectedAngle());
                  tele.addLine("Switch states to 2 to move hand and place sample under");
                    hp.HandAngle = 0;
                    prevState = 0;
                    hand.setPosition(ServoRegulizer(hp.MStandard*(hp.HandAngle - tp.TurretAngle+90) + hp.BStandard));

                    break;
                //Hand Interpolation
                case 2:
                    if(prevState == 0){
                        tele.addLine("Switch back to test or move on to turret");
                        //the 90 accoutns for the reverse 0 and the perpendicular
                        hp.HandAngle = HandPerpendicularRegulaizer(pipeline.getDetectedAngle() +90);
                        hand.setPosition(ServoRegulizer(hp.MStandard*(hp.HandAngle - tp.TurretAngle+90) + hp.BStandard));
                        prevState = 1;
                    }
                    break;

                case 3:
                    tele.addLine("Interpolate the camera to the turret.(use many points) make sure the Y value is the same");
                    tele.addLine("switch at boxtube at pos 0");
                    tele.addData("Middle point X", pipeline.getMiddleLineX());
                    tele.addData("Middle point Y", pipeline.getMiddleLineY());
                    hp.HandAngle = 90;
                    break;
                case 4:
                    tele.addLine("Interpolate the camera to the turret.(use many points)");
                    tele.addLine("switch at boxtube at pos 0");

                    break;


                default:
                    tele.addLine("Vision Setup. Switch to state 1 for pixel to Angle");
                    break;
            }

            tele.update();

        }//opmode is active
    }

}
