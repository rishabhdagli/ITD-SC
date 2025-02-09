package opmode;

import com.arcrobotics.ftclib.command.*;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import config.auto.AutoRobot;
import config.auto.BomboclatPaths;

@Autonomous(name = "Five Spec")
public class Bomboclat extends OpModeCommand {
    AutoRobot r;

    @Override
    public void initialize() {
        r = new AutoRobot(hardwareMap, telemetry, new Pose(7.000, 54.500));
        r.t.addData("init", true);
        r.t.update();
    }

    @Override
    public void start() {
        schedule(
                new RunCommand(r.follower::update),
                new SequentialCommandGroup(
                        new FollowPath(r.follower, BomboclatPaths.line1)
                                .alongWith(new InstantCommand(() -> r.boxtube.PivotMove(435)))
                                .alongWith(new WaitUntilCommand(() -> r.conditionalEndPivot(435)))
                                .alongWith(new InstantCommand(r::zero)),
                        new InstantCommand(() -> r.boxtube.ExtensionMove(24500))
                                .alongWith(new WaitUntilCommand(() -> r.conditionalEndExtension(24500)))
                        //add more here

                )
        );
    }
}