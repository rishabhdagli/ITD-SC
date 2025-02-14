package opmode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name = "Pivot Tunner")
public class PivotTuner extends LinearOpMode {


    /*
         State 0 will display options
         State 1 will continusly reset the Pivot Motor to Zero it
         State 2 will help tune the period function
         State 3 tune amplitude of kcosine
         State 4 will allow you to move the motor in Dry run mode will seeing FF values.
         State 5 will run PDF loop
     */


    private DcMotorEx PivotMotor;

    public static double Kp,Kd,FF,Tick90,targetPos,DownKP,DownKD;
    public static int state;
    private double error,lasterror,period,currentPos,power,Ppower,Dpower,FFpower;
    private int laststate;

    public ElapsedTime time;

    MultipleTelemetry tele;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //dash init

        time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        PivotMotor = hardwareMap.get(DcMotorEx.class, "pivotENC");
        //motor init + encoder

        PivotMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        PivotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        waitForStart();

        time.reset();
        time.startTime();

        while (opModeIsActive()) {
            //clear unused tele and set it back to brake and run without enc
            if (state != laststate){
                tele.clearAll();
                PivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                PivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                laststate = state;
            }
            //reverse for positive power
            currentPos = -PivotMotor.getCurrentPosition();
            tele.addData("Current Position:", currentPos);

            switch (state) {

                case 0:
                    int x = 0;
                    break;

                case 1:
                    tele.addLine("Please Switch to the next state after holding perfectly horizontally with a level");
                    PivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    PivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    break;

                case 2:
                    tele.addLine("Input to 'Tick90' the value of ticks at 90 deg");
                    PivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    break;
                case 3:
                    tele.addLine("Tune the Feed forward value for the arm when arm is horizontal");
                    PivotMotor.setPower(FF);
                    break;
                case 4:
                    period = (2*Math.PI)/(Tick90*4);
                    FFpower = FF*Math.cos(period*currentPos);
                    tele.addData("FF power:", FFpower);
                    break;
                case 5:

                    tele.addData("Target Position:", targetPos);

                    // calculate the error
                    error = targetPos - currentPos;


                    if (error >  0 ) {
                        //prop calcs
                        Ppower = Kp * error;
                        tele.addData("Proportional Power:", Ppower);

                        //derivative calcs
                        Dpower = Kd * (error - lasterror) / time.seconds();
                        time.reset();
                        lasterror = error;
                        tele.addData("Derivative Power:", Dpower);

                        //FeedForward clacs
                        period = (2 * Math.PI) / (Tick90 * 4);
                        FFpower = FF * Math.cos(period * currentPos);
                        tele.addData("FeedForward Power: ", FFpower);


                        //power for motor
                        power = Ppower + Dpower + FFpower;
                        tele.addData("Overall power:", power);
                        PivotMotor.setPower(power);
                    }
                    else if (error < 0){
                        //prop calcs
                        Ppower = DownKP * error;
                        tele.addData("Proportional Power:", Ppower);

                        //derivative calcs
                        Dpower = DownKD * (error - lasterror) / time.seconds();
                        time.reset();
                        lasterror = error;
                        tele.addData("Derivative Power:", Dpower);


                        //power for motor going down
                        power = Ppower + Dpower;
                        tele.addData("Overall power:", power);
                        PivotMotor.setPower(power);
                    }
                    break;

            }//switch

            tele.update();
        }//opmode end
    }
}
