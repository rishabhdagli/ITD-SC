package opmode.Tunners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import opmode.Vision.CrushSampleAnglePipeline;

@Config
@TeleOp(name="Extention Tuner")
public class ExtentionTuner extends LinearOpMode {
    public static double targetPos = 0.0;
    public static int switchCase = 0;
    private int lastSwitchCase = 0;
    public static double upkP, downkD, downkP, horizantalkP;
    public static boolean horizantal;
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
                    three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                else if(switchCase == 5) {
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
//            tele.addData();
            if(!horizantal && error > 0)
                pwr  = (error* upkP);
            else if(!horizantal && error < 0)
                pwr  = (error* downkP) + (downkD *derivitave);
            else if (horizantal)
                pwr = (error * horizantalkP);


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
                    three.setPower(0);
                    tele.addData("1 and 2 motor power", pwr);
                    break;
                case 5:
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

    @Config
    @TeleOp(name="Crush Sample Auto Align")
    public static class CrushSampleAutoAlign extends LinearOpMode{
        private CrushSampleAnglePipeline pipeline;
        private VisionPortal VP;
        private FtcDashboard dash;
        private MultipleTelemetry tele;

        public static double Angle,turner = 0, LowerBounds = 500, UpperBounds = 5000;

        Gamepad lastg;

        Servo wrist,arm,hand,claw,turret;

        @Override
        public void runOpMode() throws InterruptedException {
            lastg = new Gamepad();



            wrist = hardwareMap.get(Servo.class, "Servo6"); //wrist
            arm = hardwareMap.get(Servo.class, "Servo7"); //arm
            hand = hardwareMap.get(Servo.class, "Servo8"); //hand
            claw = hardwareMap.get(Servo.class, "Servo9");//claw
            turret = hardwareMap.get(Servo.class, "Servo10");//turret

            // Initialize the pipeline
            pipeline = new CrushSampleAnglePipeline();
            dash = FtcDashboard.getInstance();
            dash.getInstance().startCameraStream(pipeline,0);
            // Create the VisionPortal with the pipeline
            VP = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

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
                pipeline.setLowerBounds(LowerBounds);
                pipeline.setUpperBounds(UpperBounds);

                telemetry.addData("Angle", pipeline.getDetectedAngle());
                telemetry.addData("Sample Detected", pipeline.isSampleDetected());
                telemetry.update();
            }
        }

    }
}

