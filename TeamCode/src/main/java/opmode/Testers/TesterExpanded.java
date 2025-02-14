package opmode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Tester - Expanded")
public class TesterExpanded extends LinearOpMode {

    //FTCDASH send what ever config stuff before the runopmode starts. Only intilize is before.


    //Max postiotion of extension: 81578.0
    //KP: 0.0001

    private double kPPivot,kDPivot,FFPivot,power;
    public static boolean Hanging;

    DcMotorEx Pivot, BT1, BT2, BT3;

    AnalogInput PivotAbs, boxtubeAbs;

    ElapsedTime timer;

    public static class ServoControl{
        public double pos0, pos1, pos2, pos3, pos4, pos5, wristAngle, armAngle, hand=0.5, claw, turret = 0.5, pos11;

    }


    public static class BoxtubePivot {
        public double PivotDownKp = 0.003, PivotDownKd = 0, PivotkP = 0.003, PivotKd = 0, Tick90 = 1000, FF = 0.05, period = (2 * Math.PI) / (Tick90 * 4),
                ExtensionKp, ExtensionKd, lasterror, targetPosPivot, hangkP = 0, offsetPivotTicks, offsetTubeTicks;
    }


    private Servo servo0,servo1, servo2, servo3, servo4,  servo5, servo6, servo7, servo8, servo9, servo10, servo11;
    public static double extensionTargetPos, KpExt = 0.0005;
    double wristTicks, armTicks;

    public static ServoControl servoControl = new ServoControl();
    public static BoxtubePivot boxtubePivot = new BoxtubePivot();


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
        PivotAbs = hardwareMap.get(AnalogInput.class, "pivotAbs");
        boxtubeAbs = hardwareMap.get(AnalogInput.class, "boxtubeAbs");
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
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BT2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BT3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        double offset = 0.1;
//        double armAngle = 12;
//        double wristPosition = (armAngle * 1.2) + offset;
        //


        waitForStart();

        servo6.setPosition(0.5);
        servo7.setPosition(0.5);
        servo8.setPosition(0.5);
        servo9.setPosition(0);
        servo10.setPosition(0.5);
        boxtubePivot.offsetPivotTicks = 0;
        //-1233.33333*(PivotAbs.getVoltage()) + 482.233333;
        boxtubePivot.offsetTubeTicks = 0;
        //1243.083*(boxtubeAbs.getVoltage()) - 2292.24506;





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
//            tele.addData("Pivot Voltage", PivotAbs.getVoltage());
//            tele.addData("Boxtube Voltage", boxtubeAbs.getVoltage());
//            tele.addData("Pivot ticks", Pivot.getCurrentPosition());

            // tele.addData("Servo11 Position", servo11.getPosition());


            double armTicks = 0.0033333*(servoControl.armAngle) + 0.5;
            //step 1(angle to tick)

            //step 2 (ticks to offset angle)
            double offsetAngle = 391.30435 * (armTicks) - 195.65217;
            //Could be offsetAngle = armAngle *1.2

            //step 3 (offset to wrist ofset)
            double wirstTicks = -0.0031111*(servoControl.wristAngle+25+offsetAngle) + 0.5;

            servo6.setPosition(wirstTicks); //wrist servo
            servo7.setPosition(armTicks); //arm servo

            //theese need to reinitlize everytime
            double pivotCurrentPos = boxtubePivot.offsetPivotTicks + (-Pivot.getCurrentPosition());
            double boxtubeCurrentPos = boxtubePivot.offsetTubeTicks + (-BT1.getCurrentPosition());

            double Pivoterror = boxtubePivot.targetPosPivot - pivotCurrentPos;

            if (!Hanging) {
                if (Pivoterror > 0) {
                    double power = boxtubePivot.PivotkP * Pivoterror + boxtubePivot.PivotKd * (Pivoterror - boxtubePivot.lasterror) / timer.seconds() + boxtubePivot.FF * Math.cos(boxtubePivot.period * Pivot.getCurrentPosition());
                    Pivot.setPower(power);
                    boxtubePivot.lasterror = Pivoterror;

                    timer.reset();
                } else if (Pivoterror < 0) {
                    double power = boxtubePivot.PivotDownKp * Pivoterror + boxtubePivot.PivotDownKd * (Pivoterror - boxtubePivot.lasterror) / timer.seconds();
                    Pivot.setPower(power);
                    boxtubePivot.lasterror = Pivoterror;

                    timer.reset();

                }
            }
            else{
                double power = boxtubePivot.hangkP * Pivoterror;
                Pivot.setPower(power);
            }

            tele.addData("Pivot power:", Pivot.getPower());
            tele.addData("Pivot position", pivotCurrentPos);
            tele.addData("Target Pos", boxtubePivot.targetPosPivot);


            double extensionError = (extensionTargetPos)-boxtubeCurrentPos;
            if (BT1.getCurrentPosition() > 0 || extensionTargetPos < 0){ //min position hardstop
                tele.addLine("Hard Stop Hit");
                if(extensionError > 0){power = KpExt*extensionError;}
                else { power = 0;}
            }
            else if (BT1.getCurrentPosition() < -81578.0 || extensionTargetPos > 81578.0){ //max position hardstop
                tele.addLine("Hard Stop Hit");
                if (extensionError < 0){power = KpExt*extensionError;}
                else { power = 0;}
            }
            else {power = KpExt*extensionError;}

            BT1.setPower(power);
            BT2.setPower(power);
            BT3.setPower(power);

            tele.addData("Current boxtube Position", boxtubeCurrentPos);

            tele.update();
        }
    }
}