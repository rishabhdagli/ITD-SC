package opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;
import com.sfdev.assembly.state.StateMachine;

import java.util.Arrays;

import Subsystems.Boxtube;
import Subsystems.Robot;
import Subsystems.StateMachineGenerator;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "MainTele")
public class MainTele extends LinearOpMode {

    public static double JoyStickInc = 2250, Ymult = 0.8, rxMult = 0.7, AvoidRiggingWaypointInc = 18;
    public static double specScorex = -37, specScorey = -49, pickupx = -5.000, pickupy = 0, noSlamSpecScoreX = -24;
    public Robot teleRobot;
    public Boxtube boxtube;
    public Follower follower;
    boolean sampleMode = true, check = false;
    double MonkeyExpressFlashBang = 0;
    LED redLED, redLED2, greenLED, greenLED2;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        teleRobot = new Robot(hardwareMap, gamepad1, gamepad2);

        boxtube = teleRobot.boxtube;

        Constants.setConstants(FConstants.class, LConstants.class);

//        follower = new Follower(hardwareMap);

//        follower.setStartingPose(new Pose(22.000, 30.000, Point.CARTESIAN));

        redLED = hardwareMap.get(LED.class, "red");
        greenLED = hardwareMap.get(LED.class, "green");

        redLED2 = hardwareMap.get(LED.class, "red2");
        greenLED2 = hardwareMap.get(LED.class, "green2");


        StateMachine sampleMachine = StateMachineGenerator.GenerateSampleMachine(gamepad2, gamepad1, teleRobot);
        StateMachine specimenMachine = StateMachineGenerator.GenerateSpecimenMachine(gamepad2, gamepad1, teleRobot);
        waitForStart();
        sampleMachine.start();

        telemetry.addData("States", sampleMachine.getStateString());


        while (opModeIsActive()) {


            if (sampleMode) {
                sampleMachine.update();
            } else {
                specimenMachine.update();
            }

            //hopefully this works
            if (gamepad1.right_bumper) {
                teleRobot.TeleControl(Ymult, 1, rxMult);
            } else if (boxtube.PivotisMoving()) {
                teleRobot.TeleControl(1, 1, rxMult);
            }


            // Way pointing stuff
            else if (Math.abs(gamepad1.left_trigger) > 0.5 && !sampleMode) {
                teleRobot.drive.PID2P(specScorey, specScorex);
//                specimenMachine.setState(StateMachineGenerator.States.SpecimenPreScore);
            } else if (Math.abs(gamepad1.left_trigger) > 0 && Math.abs(gamepad1.left_trigger) < 0.5 && !sampleMode) {
                teleRobot.drive.PID2P(specScorey, noSlamSpecScoreX);
//                specimenMachine.setState(StateMachineGenerator.States.SpecimenPreScore);


            } else if (Math.abs(gamepad1.right_trigger) > 0.5 && !sampleMode) {
                if (teleRobot.drive.getPos()[0] > specScorex + AvoidRiggingWaypointInc) {

                    teleRobot.drive.PID2P(pickupy, pickupx);
                    telemetry.addLine("Avoided Rigging");
//                    specimenMachine.setState(StateMachineGenerator.States.SpecimenWall);

                } else {

                    teleRobot.drive.PID2P(specScorey, pickupx);
//                    specimenMachine.setState(StateMachineGenerator.States.SpecimenPostScore);

                    telemetry.addLine("Avoiding Rigging");

                }

            }


            //tele-op regular
            else {
                teleRobot.TeleControl(1, 1, 1);
            }


            //LIght stuff

            String state = sampleMachine.getStateString();
            if (state.equals("SpecimenWall") || state.equals("SpecimenWallGrab") || state.equals("SpecimenWallGrabUp") || state.equals("SpecimenPreScore") || state.equals("SampleHover") || state.equals("CLOSING_CLAW") || state.equals("LoiterSample") || state.equals("ObsZoneRelease") || state.equals("BasketExtend")) {
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
            } else {
                redLED.off();
                redLED2.off();
                greenLED.off();
                greenLED2.off();
                MonkeyExpressFlashBang += 1;

            }


            //state stuff

            if (gamepad2.touchpad) {
                check = true;
            }


            if (!gamepad2.touchpad && check) {
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
            if (sampleMode == false) {
                telemetry.addData("State: ", specimenMachine.getStateString());
            } else {
                telemetry.addData("State: ", sampleMachine.getStateString());
            }

//            telemetry.addData("X pose: ", follower.getPose().getX());
//            telemetry.addData("Y pose: ", follower.getPose().getY());
//            telemetry.addData("Heading pose: ", follower.getPose().getHeading());
//
//            follower.update();

            telemetry.addData("Extention Power: ", boxtube.getExtpow());
            telemetry.addData("Pivot Power: ", boxtube.getPivpow());
            telemetry.addData("Pinpoint position", Arrays.toString(teleRobot.drive.getPos()));
            telemetry.update();
            teleRobot.UPDATE();


        }// opmode loop active
    }//linear opmode end
}