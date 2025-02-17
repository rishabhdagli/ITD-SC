package Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Boxtube{

    Telemetry t;
    HardwareMap h;

   AnalogInput PivotAbs;
   AnalogInput boxtubeAbs;

   public double pivotoffset,Boxtubeoffset, targetPiv, targetExt;
    final int MaxExtension = 31500;

    public DcMotorEx Pivot, BT1, BT2, BT3;

    public double offsetAngle,  KpExt = 0.0005, ExtPwr;

    ElapsedTime timer;

    double PivotDownKp = 0.002, PivotDownKd = 0.001, PivotkP = 0.007, PivotKd = 0.0008,Tick90 = 1147,FF = 0.09,period = (2*Math.PI)/(Tick90*4),
            ExtensionKp,ExtensionKd,lasterror;


    public Boxtube(HardwareMap hardwareMap) {
        // Initialize motors with proper names
        Pivot = hardwareMap.get(DcMotorEx.class, "pivotENC");
        BT1 = hardwareMap.get(DcMotorEx.class, "Boxtube1ENC");
        BT2 = hardwareMap.get(DcMotorEx.class, "Boxtube2odoleft");
        BT3 = hardwareMap.get(DcMotorEx.class, "Boxtube3odoright");

        //PivotEnc = hardwareMap.get(DcMotorEx.class, "Pivot");

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

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.startTime();

        PivotAbs = hardwareMap.get(AnalogInput.class, "pivotAbs");
        boxtubeAbs = hardwareMap.get(AnalogInput.class, "boxtubeAbs");

        pivotoffset = 0;
                //-1233.33333*(PivotAbs.getVoltage()) + 482.233333;
        Boxtubeoffset = 0;
                //1243.083*(boxtubeAbs.getVoltage()) - 2292.24506;

    }

    public void update(){
        updatePiv();
        updateExt();
    }
    public void changekP(double kp){
        PivotkP = kp;
    }


    public void setPivot(double val) {
        targetPiv = val;
    }
    public void updatePiv() {
        PivotMove(targetPiv);
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

    public void setExt(double val) {
        targetExt = val;
    }
    public void updateExt() {
        ExtensionMove(targetExt);
    }


    //for slowing when moving
    public boolean PivotisMoving() {
        //change to if moving slow when down
       if(Pivot.getPower() > 0.3){
           return true;
       }
       else{
           return false;
       }
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
}
