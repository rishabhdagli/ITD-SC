package opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;
import com.sfdev.assembly.state.StateMachine;

import config.teleop.DiddyArm;
import config.teleop.DragonWarrior;
import config.teleop.Robot;
import config.teleop.StateMachineGenerator;

@TeleOp(name = "LosConflictosArmadosTeleOP")
public class LosConflictosArmadosTeleOP extends LinearOpMode {

    public DragonWarrior dragon;
    public Robot robot;
    public DiddyArm diddy;
    boolean sampleMode = true, check = false;
    double MonkeyExpressFlashBang = 0;

    LED redLED, redLED2, greenLED, greenLED2;

    @Override
    public void runOpMode() throws InterruptedException {
        diddy = new DiddyArm();
        robot = new Robot();
        dragon = new DragonWarrior();
        dragon.init(hardwareMap);
        robot.init(hardwareMap, gamepad2);
        diddy.init(hardwareMap);


        redLED = hardwareMap.get(LED.class, "red");
        greenLED = hardwareMap.get(LED.class, "green");

        redLED2 = hardwareMap.get(LED.class, "red2");
        greenLED2 = hardwareMap.get(LED.class, "green2");


        StateMachine sampleMachine = StateMachineGenerator.GenerateSampleMachine(gamepad2, robot);
        StateMachine specimenMachine = StateMachineGenerator.GenerateSpecimenMachine(gamepad2, robot);


        waitForStart();
        sampleMachine.start();

        telemetry.addData("States", sampleMachine.getStateString());


        telemetry.update();
        while (opModeIsActive()) {
            if(sampleMode){
                sampleMachine.update();
            }
            else{specimenMachine.update();}


            if (gamepad1.right_bumper) {
                dragon.TeleopControl(gamepad1.left_stick_y * 0.7, gamepad1.left_stick_x * 0.7, gamepad1.right_stick_x / 2.0);
            } else {
                dragon.TeleopControl(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }



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
            telemetry.addData("Pivot Motors", diddy.getPivotMotor());
            telemetry.addData("Boxtube motors", diddy.getTubePower());

            telemetry.update();

        }// opmode loop active
    }//linear opmode end
}