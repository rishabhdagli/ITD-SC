package opmode.Tunners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Array;

@Config
@TeleOp(name="Extention Tuner")
public class ExtentionTuner extends LinearOpMode {
    public static double targetPos = 0.0;
    public static int switchCase = 0;
    private int lastSwitchCase = 0;
    public static double P,D,FF;
    private double error,derivitave, currentPos, pwr, lastError;
    private DcMotorEx one;
    private DcMotorEx two;
    private DcMotorEx three;
    ElapsedTime timer;
    MultipleTelemetry tele;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // Initialize the hardware
        //TODO: Change these configs
        one = hardwareMap.get(DcMotorEx.class, "Boxtube1ENC");
        two = hardwareMap.get(DcMotorEx.class, "Boxtube2odoleft");
        three = hardwareMap.get(DcMotorEx.class, "Boxtube3odoright");

        two.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        one.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        three.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        one.setDirection(DcMotorSimple.Direction.FORWARD);
        two.setDirection(DcMotorSimple.Direction.REVERSE);
        three.setDirection(DcMotorSimple.Direction.REVERSE);


        one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        one.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        two.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        three.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        // TODO: Check Motor directions

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            if(switchCase != lastSwitchCase){
                if(switchCase == 1) {

                    one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                else if(switchCase == 2) {
                    one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                else if(switchCase == 3) {
                    one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                else if(switchCase == 4) {
                    one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            // obtaining encoder position
            currentPos = one.getCurrentPosition();
            tele.addData("Current Position:", currentPos);
            tele.addData("Target Position:", targetPos);

            // calculate error
            error = targetPos - currentPos;


            derivitave = (error - lastError)/timer.seconds();
            timer.reset();
            lastError = error;
            // Reset for change in Time.seconds
            tele.addData("Feed Forward Steady State: ", FF);

            pwr  = (error*P) + (D*derivitave) + FF;

            //DONE: write a switch Case
            // DONE: create a set of instructions for motor directions
            /**
             * In order to tune motor directions, use the switch case
             * case 1 is motor 1, and 2 is 2, and 4 is all of the mtrs
             */

            switch(switchCase){
                case 0:
                    break;
                case 1:
                    one.setPower(pwr);
                    two.setPower(0);
                    three.setPower(0);
                    tele.addData("Motor one power", pwr);
                    break;
                case 2:
                    two.setPower(pwr);
                    one.setPower(0);
                    three.setPower(0);
                    tele.addData("Motor two power", pwr);
                    break;
                case 3:
                    two.setPower(0);
                    one.setPower(0);
                    three.setPower(pwr);
                    tele.addData("Motor three power", pwr);
                    break;
                case 4:
                    one.setPower(pwr);
                    two.setPower(pwr);
                    three.setPower(pwr);
                    tele.addData("All motor power", pwr);
                    break;
            }
            lastSwitchCase = switchCase;

            tele.addData("Derivative", derivitave);
            tele.addData("Total Power", pwr);
            tele.update();

        }
    }
}

