package opmode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Tester Class")
public class Tester extends LinearOpMode {

    //FTCDASH send what ever config stuff before the runopmode starts. Only intilize is before.


    //Max postiotion of extension: 81578.0
    //KP: 0.0001

    private double kPPivot,kDPivot,FFPivot,power;
    public static boolean Hanging;

    DcMotorEx Pivot, BT1, BT2, BT3;

    ElapsedTime timer;

    public static class ServoControl{
        public double  wristAngle, armAngle, hand, claw, turret = 0.5;

    }




    double PivotDownKp = 0.002, PivotDownKd = 0.001, PivotkP = 0.007, PivotKd = 0.0008,Tick90 = 1147,FF = 0.09,period = (2*Math.PI)/(Tick90*4),
            ExtensionKp,ExtensionKd,lasterror,KpExt = 0.0005;

    public static double targetPosPivot,extensionTargetPos;



    private Servo servo0,servo1, servo2, servo3, servo4,  servo5, servo6, servo7, servo8, servo9, servo10, servo11;
    double wristTicks, armTicks;


    //aboslute encoder things
    AnalogInput PivotAbs, boxtubeAbs;
    double offsetPivotTicks, offsetTubeTicks;

    public static ServoControl servoControl = new ServoControl();



    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.startTime();



//        servo0 = hardwareMap.get(Servo.class, "Servo0");
//        servo1 = hardwareMap.get(Servo.class, "Servo1");
//        servo2 = hardwareMap.get(Servo.class, "Servo2");
//        servo3 = hardwareMap.get(Servo.class, "Servo3");
//        servo4 = hardwareMap.get(Servo.class, "Servo4");
//        servo5 = hardwareMap.get(Servo.class, "Servo5");

        Pivot = hardwareMap.get(DcMotorEx.class, "pivotENC");
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servo6 = hardwareMap.get(Servo.class, "Servo6"); //wrist
        servo7 = hardwareMap.get(Servo.class, "Servo7"); //arm
        servo8 = hardwareMap.get(Servo.class, "Servo8"); //hand
        servo9 = hardwareMap.get(Servo.class, "Servo9");//claw
        servo10 = hardwareMap.get(Servo.class, "Servo10");//turret
        //servo11 = hardwareMap.get(Servo.class, "Servo11");

        BT1 = hardwareMap.get(DcMotorEx.class, "Boxtube1ENC");
        BT2 = hardwareMap.get(DcMotorEx.class, "Boxtube2odoleft");
        BT3 = hardwareMap.get(DcMotorEx.class, "Boxtube3odoright");

        BT2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BT1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BT3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        BT1.setDirection(DcMotorSimple.Direction.REVERSE);
        BT2.setDirection(DcMotorSimple.Direction.REVERSE);
        BT3.setDirection(DcMotorSimple.Direction.REVERSE);

        BT1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BT1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BT2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BT3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        double offset = 0.1;
//        double armAngle = 12;
//        double wristPosition = (armAngle * 1.2) + offset;
        //

        PivotAbs = hardwareMap.get(AnalogInput.class, "pivotAbs");
        boxtubeAbs = hardwareMap.get(AnalogInput.class, "boxtubeAbs");

        offsetPivotTicks = 0;
        //-1233.33333*(PivotAbs.getVoltage()) + 482.233333;
        offsetTubeTicks = 0;
        //1243.083*(boxtubeAbs.getVoltage()) - 2292.24506;



        waitForStart();

        servo6.setPosition(0.5);
        servo7.setPosition(0.5);
        servo8.setPosition(0.5);
        servo9.setPosition(0);
        servo10.setPosition(0.5);





        while (opModeIsActive()) {



//            servo6.setPosition(wrist);
//            servo7.setPosition(arm);
            servo8.setPosition(servoControl.hand);
            servo9.setPosition(servoControl.claw);
            servo10.setPosition(servoControl.turret);
            // servo11.setPosition(pos11);

//            tele.addData("Servo0 Position", servo0.getPosition());
//            tele.addData("Servo1 Position", servo1.getPosition());
//            tele.addData("Servo2 Position", servo2.getPosition());
//            tele.addData("Servo3 Position", servo3.getPosition());
//            tele.addData("Servo4 Position", servo4.getPosition());
//            tele.addData("Servo5 Position", servo5.getPosition());
            tele.addData("Servo6 Position", servo6.getPosition());
            tele.addData("Servo7 Position", servo7.getPosition());
            tele.addData("Servo8 Position", servo8.getPosition());
            tele.addData("Servo9 Position", servo9.getPosition());
            tele.addData("Servo10 Position", servo10.getPosition());

            // tele.addData("Servo11 Position", servo11.getPosition());

            double armTicks = 0.0033333*(servoControl.armAngle) + 0.5;
            //step 1(angle to tick)

            //step 2 (ticks to offset angle)
            double offsetAngle = 391.30435 * (armTicks) - 195.65217;
            //Could be offsetAngle = armAngle *1.2

            //step 3 (offset to wrist ofset)
            double wirstTicks = -0.0031111*(servoControl.wristAngle+25+offsetAngle) + 0.5;

            tele.addData("Wrist ticks", wirstTicks);
            tele.addData("Arm ticks", armTicks);

            servo6.setPosition(wirstTicks); //wrist servo
            servo7.setPosition(armTicks); //arm servo

            double pivotCurrentPos = offsetPivotTicks + (-Pivot.getCurrentPosition());
            double boxtubeCurrentPos = offsetTubeTicks + (-BT1.getCurrentPosition());


            double Pivoterror = targetPosPivot - pivotCurrentPos;

            if (Pivoterror > 0) {
                double power = PivotkP * Pivoterror + PivotKd * (Pivoterror - lasterror) / timer.seconds() + FF * Math.cos(period * Pivot.getCurrentPosition());
                Pivot.setPower(power);
                lasterror = Pivoterror;

                timer.reset();
            } else if (Pivoterror < 0) {
                double power = PivotDownKp * Pivoterror + PivotDownKd * (Pivoterror - lasterror) / timer.seconds();
                Pivot.setPower(power);
                lasterror = Pivoterror;

                timer.reset();

            }


            tele.addData("Pivot power:", Pivot.getPower());
            tele.addData("Pivot position", pivotCurrentPos);
            tele.addData("Target Pos", targetPosPivot);


            double extensionError = (extensionTargetPos)-boxtubeCurrentPos;
            if (BT1.getCurrentPosition() > 0 || extensionTargetPos < 0){ //min position hardstop
                tele.addLine("Hard Stop Hit");
                if(extensionError > 0){power = KpExt*extensionError;}
                else { power = 0;}
            }
            else if (BT1.getCurrentPosition() < -35000 || extensionTargetPos > 35000){ //max position hardstop
                tele.addLine("Hard Stop Hit");
                if (extensionError < 0){power = KpExt*extensionError;}
                else { power = 0;}
            }
            else {power = KpExt*extensionError;}

            BT1.setPower(power);
            BT2.setPower(power);
            BT3.setPower(power);

            tele.addData("Boxtube current position: ", boxtubeCurrentPos);

            tele.update();
        }
    }
}