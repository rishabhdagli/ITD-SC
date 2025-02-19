package opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.sfdev.assembly.state.StateMachine;

import Subsystems.Boxtube;
import Subsystems.Drivetrain;
import Subsystems.EndEffector;
import Subsystems.TeleRobot;
import Subsystems.StateMachineGenerator;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "MainTele")
public class MainTele extends LinearOpMode {

    public TeleRobot teleRobot;
    public Boxtube boxtube;
    public Follower follower;
    boolean sampleMode = true, check = false;
    double MonkeyExpressFlashBang = 0;

    LED redLED, redLED2, greenLED, greenLED2;

    @Override
    public void runOpMode() throws InterruptedException {
        boxtube= new Boxtube(hardwareMap); //just for pivot check
        teleRobot = new TeleRobot(hardwareMap, gamepad1,gamepad2);

        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);

        follower.setStartingPose(new Pose(22.000, 30.000, Point.CARTESIAN));

        redLED = hardwareMap.get(LED.class, "red");
        greenLED = hardwareMap.get(LED.class, "green");

        redLED2 = hardwareMap.get(LED.class, "red2");
        greenLED2 = hardwareMap.get(LED.class, "green2");


        StateMachine sampleMachine = StateMachineGenerator.GenerateSampleMachine(gamepad2,gamepad1, teleRobot);
        StateMachine specimenMachine = StateMachineGenerator.GenerateSpecimenMachine(gamepad2,gamepad1, teleRobot);
        waitForStart();
        sampleMachine.start();

        telemetry.addData("States", sampleMachine.getStateString());


        while (opModeIsActive()) {
            if(sampleMode){
                sampleMachine.update();
            }
            else{specimenMachine.update();}


            //hopefully this works
            if (gamepad1.right_bumper) {teleRobot.TeleControl(0.7,0.7,0.5);}
            else if(boxtube.PivotisMoving()) {teleRobot.TeleControl(0.7,1,1);}
            else{teleRobot.TeleControl(1,1,1);}


            String state = sampleMachine.getStateString();
            if (state.equals("Stationary")||state.equals("LOITER") || state.equals("SampleHover") || state.equals("CLOSING_CLAW") || state.equals("LoiterSample") || state.equals("ObsZoneRelease") || state.equals("PivotOverCenter") || state.equals("BasketExtend")) {
                // light green!
                redLED.off();
                redLED2.off();
                greenLED.on();
                greenLED2.on();

            } else if (MonkeyExpressFlashBang == 10) {
                // light red!
                redLED.on();
                redLED2.on();
                greenLED.off();
                greenLED2.off();
                MonkeyExpressFlashBang = 0;
            }
                else {
                redLED.off();
                redLED2.off();
                greenLED.off();
                greenLED2.off();
                MonkeyExpressFlashBang +=1;

                }


            if (gamepad2.left_stick_button) {
                check = true;
            }


            if (!gamepad2.left_stick_button && check) {
                check = false;
                sampleMode = !sampleMode;
                if (!sampleMode) {
                    sampleMachine.stop();
                    specimenMachine.reset();

                    specimenMachine.start();
                    telemetry.addLine("Stopped sample, started specimen");
                    gamepad2.rumble(100);

                } else {
                    specimenMachine.stop();

                    sampleMachine.reset();

                    sampleMachine.start();
                    telemetry.addLine("Stopped specimen, started sample");
                    gamepad2.rumble(100);
                }

            }
            if (sampleMode == false){
                telemetry.addData("State: ", specimenMachine.getStateString());
            }
            else{telemetry.addData("State: ", sampleMachine.getStateString());}

            telemetry.addData("X pose: ", follower.getPose().getX());
            telemetry.addData("Y pose: ", follower.getPose().getY());
            telemetry.addData("Heading pose: ", follower.getPose().getHeading());

            follower.update();

            telemetry.update();

        }// opmode loop active
    }//linear opmode end
}