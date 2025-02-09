package opmode;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import config.auto.AutoRobot;
import config.auto.FiveSpecPaths;

@Autonomous(name = "5+0")
public class FiveSpec extends LinearOpMode {
    AutoRobot r;

    @Override
    public void runOpMode() throws InterruptedException {
        r = new AutoRobot(hardwareMap, telemetry, new Pose(7.000, 54.500));
        r.t.addData("init", true);
        r.t.update();

        waitForStart();

        //CONVERT TO SWITCH CASE

//        schedule(
//                new RunCommand(r.follower::update),
//                new SequentialCommandGroup(
//                        new FollowPath(r.follower, BomboclatPaths.line1)
//                                .alongWith(new InstantCommand(() -> r.boxtube.PivotMove(435)))
//                                .alongWith(new WaitUntilCommand(() -> r.conditionalEndPivot(435)))
//                                .alongWith(new InstantCommand(r::SpecimenPreLoad)),
//                        new InstantCommand(() -> r.boxtube.ExtensionMove(24500))
//                                .alongWith(new WaitUntilCommand(() -> r.conditionalEndExtension(24500)))
//                        //add more here
//
//                )
//        );
    }
}