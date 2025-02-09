package config.auto;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Boxtube {

    Telemetry t;
    HardwareMap h;

    AnalogInput PivotAbs;
    AnalogInput boxtubeAbs;

    public double pivotoffset,Boxtubeoffset;
    final int MaxExtension = 30500;

    public DcMotorEx Pivot, BT1, BT2, BT3;
    public DcMotorEx PivotEnc;
    //    public AnalogInput PivotEnc, ExtensionEnc;
    public Servo Wrist, Arm, Turret, Hand, Claw;

    public double offsetAngle,  KpExt = 0.0005, ExtPwr;

    ElapsedTime timer;

    double PivotDownKp = 0.003, PivotDownKd = 0, PivotkP = 0.003, PivotKd = 0,Tick90 = 1000,FF = 0.05,period = (2*Math.PI)/(Tick90*4),
            ExtensionKp,ExtensionKd,lasterror;
    
    public Boxtube(HardwareMap hardwareMap, Telemetry telemetry) {
        this.h = hardwareMap;
        this.t = telemetry;

        // Initialize motors with proper names
        Pivot = h.get(DcMotorEx.class, "pivotENC");
        BT1 = h.get(DcMotorEx.class, "Boxtube1ENC");
        BT2 = h.get(DcMotorEx.class, "Boxtube2odoleft");
        BT3 = h.get(DcMotorEx.class, "Boxtube3odoright");

        //PivotEnc = h.get(DcMotorEx.class, "Pivot");

        // Reset encoders
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BT1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BT2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BT3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run without encoders
        Pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BT3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        Pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BT1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BT2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BT3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Correct motor directions if needed
        Pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        //theese are good
        BT1.setDirection(DcMotorSimple.Direction.REVERSE);
        BT2.setDirection(DcMotorSimple.Direction.REVERSE);
        BT3.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize other hardware
//        PivotEnc = h.get(AnalogInput.class, "PivotEnc");
//        ExtensionEnc = h.get(AnalogInput.class, "ExtensionEnc");

        Wrist = h.get(Servo.class, "Servo6");
        Arm = h.get(Servo.class, "Servo7");
        Turret = h.get(Servo.class, "Servo10");
        Hand = h.get(Servo.class, "Servo8");
        Claw = h.get(Servo.class, "Servo9");

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.startTime();

        PivotAbs = h.get(AnalogInput.class, "pivotAbs");
        boxtubeAbs = h.get(AnalogInput.class, "boxtubeAbs");

        pivotoffset = 0;
        //-1233.33333*(PivotAbs.getVoltage()) + 482.233333;
        Boxtubeoffset = 0;
        //1243.083*(boxtubeAbs.getVoltage()) - 2292.24506;

    }




    public void PivotMove(double targetPos) {
        double currentPivot = pivotoffset + (-Pivot.getCurrentPosition());
        double error = targetPos - currentPivot;


        if (error >  0 ) {
            double power = PivotkP * error + PivotKd*(error - lasterror)/timer.seconds() +   FF * Math.cos(period * Pivot.getCurrentPosition());
            Pivot.setPower(power);
            lasterror = error;

            timer.reset();
        }
        else if (error < 0){
            double power = PivotDownKp * error + PivotDownKd*(error - lasterror)/timer.seconds();
            Pivot.setPower(power);
            lasterror = error;

            timer.reset();
        }
    }

    public void ExtensionPower(double power) {
        BT1.setPower(power);
        BT2.setPower(power);
        BT3.setPower(power);
    }

    public void ExtensionMove(double extensionTargetPos){
        double currentBoxtube = Boxtubeoffset = (-BT1.getCurrentPosition());
        double extensionError = (extensionTargetPos)+BT1.getCurrentPosition();
        if (BT1.getCurrentPosition() > 0 || extensionTargetPos < 0){ //min position hardstop
            if(extensionError > 0){ExtPwr = KpExt*extensionError;}
            else { ExtPwr = 0;}
        }
        else if (BT1.getCurrentPosition() < -MaxExtension || extensionTargetPos > MaxExtension){ //max position hardstop
            if (extensionError < 0){ExtPwr = KpExt*extensionError;}
            else { ExtPwr = 0;}
        }
        else {ExtPwr = KpExt*extensionError;}
        ExtensionPower(ExtPwr);
    }

    public void hand(double pos) {
        Hand.setPosition(pos);
    }

    public double handPos() {
        return Hand.getPosition();
    }
    public void turret(double pos) {
        Turret.setPosition(pos);
    }

    public void claw(double pos) {
        Claw.setPosition(pos);

    }
    public void setEndEffector(double armAngle, double wristAngle){
        double armTicks = 0.0033333*(armAngle) + 0.5;
        //step 1(angle to tick)

        //step 2 (ticks to offset angle)
        double offsetAngle = 391.30435 * (armTicks) - 195.65217;
        //Could be offsetAngle = armAngle *1.2

        //step 3 (offset to wrist ofset)
        double wirstTicks = -0.0031111*(wristAngle+25+offsetAngle) + 0.5;

        Arm.setPosition(armTicks);
        Wrist.setPosition(wirstTicks);
    }





}
