package opmode.Tunners;

import static opmode.Tunners.AutoAlignTuner.hp;
import static opmode.Tunners.AutoAlignTuner.tp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import Subsystems.Boxtube;
import opmode.Vision.CrushSampleAnglePipelineTurretTrial;

public class AutoAlignPickupMethodTuner extends LinearOpMode {
    private CrushSampleAnglePipelineTurretTrial pipeline;
    private VisionPortal VP;
    private FtcDashboard dash;
    private MultipleTelemetry tele;

    private Boxtube boxtube;

    public static class ScanningParams{ double wrist=0,Arms=0,turret=0,claw=0,hand=0;}
    public static class PickupParams{double wrist=0,Arms=0,turret=0,claw=0,hand=0;}
    public void Arms(double d){
        Arm1.setPosition(d);
        Arm2.setPosition(d);
    }


    //aryan
    //charat
    //
    Servo Wrist,Arm1,Arm2,Hand,Claw,Turret;

    public static double wrist=0.7,arm=0.6,claw =0.8,hand = 0.5,turret = 0.5;

    public static int State = 0;

    public static PickupParams Pparms = new PickupParams();
    public static ScanningParams Sparms = new ScanningParams();
    private ElapsedTime timer = new ElapsedTime();
//TODO ADD BOXTUBE STUFF

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            boxtube = new Boxtube(hardwareMap);
            dash = FtcDashboard.getInstance();
            tele = new MultipleTelemetry(telemetry, dash.getTelemetry());


        pipeline = new CrushSampleAnglePipelineTurretTrial();
        dash = FtcDashboard.getInstance();
        dash.getInstance().startCameraStream(pipeline,0);
        // Create the VisionPortal with the pipeline
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

            //TODO: MAKE RAul change this

        Wrist = hardwareMap.get(Servo.class, "Servo6"); //wrist
        Arm1 = hardwareMap.get(Servo.class, "Servo7"); //arm
        Arm2 = hardwareMap.get(Servo.class, "Servo7"); //arm
        Hand = hardwareMap.get(Servo.class, "Servo8"); //hand
        Claw = hardwareMap.get(Servo.class, "Servo9");//claw
        Turret = hardwareMap.get(Servo.class, "Servo10");//turret


        while(opModeInInit()){
            Wrist.setPosition(wrist);
            Hand.setPosition(hand);
            Turret.setPosition(turret);
            Wrist.setPosition(wrist);
            Claw.setPosition(claw);
            Arms(arm);

            switch (State){
                case 1:
                    wrist = Pparms.wrist;
                    arm = Pparms.Arms;
                    hand = Pparms.hand;
                    claw = Pparms.claw;
                    turret = Pparms.turret;

                    break;
                case 2:
                tp.ServoGain = (tp.error>0)? (tp.AGain*(tp.error*tp.error) + tp.BGain*(tp.error) + tp.CGain):-1*(tp.AGain*(tp.error*tp.error) + tp.BGain*(tp.error) + tp.CGain);
                tele.addData("Middle Line X", pipeline.getMiddleLineX());
                tp.error  =  320 - pipeline.getMiddleLineX();
                tele.addData("Error",tp.error );

                if(tp.error > 0  && Math.abs(tp.error) > tp.thresholdPixelError){
                    turret = Turret.getPosition() - tp.ServoGain;
                }
                else if (tp.error <0 && Math.abs(tp.error) > tp.thresholdPixelError){
                    turret = Turret.getPosition() + tp.ServoGain;
                }
                else if(Math.abs(tp.error) < tp.thresholdPixelError){
                    tp.TurretAngle = (Turret.getPosition() - tp.BStandard)/tp.MStandard;
                    hp.HandAngle = HandPerpendicularRegulaizer(pipeline.getDetectedAngle());
                    hand = (ServoRegulizer(hp.MStandard*(hp.HandAngle - tp.TurretAngle) + hp.BStandard));
                }

                break;
                case 3:
                    wrist = Sparms.wrist;
                    arm = Sparms.Arms;
                    hand = Sparms.hand;
                    claw = Sparms.claw;
                    turret = Sparms.turret;
                    break;


            }


        }
        waitForStart();
        timer.reset();
        State = 1;
        while(opModeIsActive()){
            switch (State){
                case 1:
                    wrist = Pparms.wrist;
                    arm = Pparms.Arms;
                    hand = Pparms.hand;
                    claw = Pparms.claw;
                    turret = Pparms.turret;
                    if(timer.seconds() > 2){
                        State = 2;
                        timer.reset();
                    }

                    break;
                case 2:
                    tp.ServoGain = (tp.error>0)? (tp.AGain*(tp.error*tp.error) + tp.BGain*(tp.error) + tp.CGain):-1*(tp.AGain*(tp.error*tp.error) + tp.BGain*(tp.error) + tp.CGain);
                    tele.addData("Middle Line X", pipeline.getMiddleLineX());
                    tp.error  =  320 - pipeline.getMiddleLineX();
                    tele.addData("Error",tp.error );

                    if(tp.error > 0  && Math.abs(tp.error) > tp.thresholdPixelError){
                        turret = Turret.getPosition() - tp.ServoGain;
                    }
                    else if (tp.error <0 && Math.abs(tp.error) > tp.thresholdPixelError){
                        turret = Turret.getPosition() + tp.ServoGain;
                    }
                    else if(Math.abs(tp.error) < tp.thresholdPixelError){
                        tp.TurretAngle = (Turret.getPosition() - tp.BStandard)/tp.MStandard;
                        hp.HandAngle = HandPerpendicularRegulaizer(pipeline.getDetectedAngle());
                        hand = (ServoRegulizer(hp.MStandard*(hp.HandAngle - tp.TurretAngle) + hp.BStandard));
                    }

                    if(timer.seconds() > 2){
                        State = 3;
                        timer.reset();
                    }

                    break;
                case 3:
                    wrist = Sparms.wrist;
                    arm = Sparms.Arms;
                    hand = Sparms.hand;
                    claw = Sparms.claw;
                    turret = Sparms.turret;
                    if(timer.seconds() > 2){
                        State = 1;
                        timer.reset();
                    }
                    break;


            }
        }

    }

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

    public double ServoRegulizer(double x) {
        return (x > 1) ? (((int)(x * 100)) % 100) / 100.0 : x;
    }
}
