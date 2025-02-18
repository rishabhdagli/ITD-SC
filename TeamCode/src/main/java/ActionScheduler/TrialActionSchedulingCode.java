package ActionScheduler;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class TrialActionSchedulingCode extends LinearOpMode {
    Event ArmDown;
    Task moveWrist,MoveClaw,movehand;



    Servo blip;



    @Override
    public void runOpMode() throws InterruptedException {
        movehand = new Task() {
            @Override
            public void run() {
                blip.setPosition(0.6);
            }
        };

ArmDown.addTimedAction(movehand,0.23);

    }

}
