package config.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;


public class StateMachineGenerator {
    public static double num = 0;
    public static StateMachine GenerateSpecimenMachine(Gamepad g, Robot r) {
        return new StateMachineBuilder()


                .state(States.SpecimenWall) // Position to grab from the wall
                .onEnter(r::SpecimenWall)
                .loop(r::SpecimenWall)
                .transition(() -> g.x, States.WAIT1)

                .state(States.WAIT1)
                .transitionTimed(0.25, States.SpecimenWallGrab)


                .state(States.SpecimenWallGrab)   // Grabs the things
                .onEnter(r::SpecimenWallGrab)
                .loop(r::SpecimenWallGrab)
                .transition(() -> g.x, States.WAIT2)
                .transition(()-> g.b, States.WAIT6) //escape state

                .state(States.WAIT6) //excape state wait
                .transitionTimed(0.25,States.SpecimenWall) // goes back to position to grab from wall

                .state(States.WAIT2)
                .transitionTimed(0.25, States.SpecimenWallGrabUp)


                .state(States.SpecimenWallGrabUp) //of the wall
                .onEnter(r::SpecimenWallUp)
                .loop(r::SpecimenWallUp)
                .transition(() -> g.x, States.WAIT3)
                .transition(()-> g.b, States.WAIT6) //in case karthik has autism last escape states

                .state(States.WAIT3)
                .transitionTimed(0.25, States.SpecimenPreScore)

                .state(States.SpecimenPreScore) // Bout to score
                .onEnter(r::SpecimenPreScore)
                .loop(r::SpecimenPreScore)
                .transition(() -> g.x, States.WAIT4)

                .state(States.WAIT4)
                .transitionTimed(0.7, States.SpecimenLatch)


                .state(States.SpecimenLatch) //Scores
                .onEnter(r::SpecimenLatch)
                .loop(r::SpecimenLatch)
                .transitionTimed(1, States.SpecimenWall)

                // Escape state
                .state(States.WAIT6)
                .transitionTimed(0.25, States.SpecimenWall)


                .build();
    }

    // X is primary, A is secondary, B is escape for logitech gamepad
    public static StateMachine GenerateSampleMachine(Gamepad g, Robot r) {
        return new StateMachineBuilder()
                .state(States.Stationary)
                .onEnter(r::Loiter)
                .loop(r::Loiter)
                .transition(() -> g.x, States.WAIT1)

                .state(States.WAIT1)
                .transitionTimed(0.25, States.SampleHover)

                .state(States.SampleHover)
                .onEnter(r::SampleHover)
                .loop(r::SampleHover)
                .transition(()->g.x, States.WAIT2)

                .state(States.WAIT2)
                .transitionTimed(0.25, States.SampleGrab)

                .state(States.SampleGrab)
                .onEnter(r::SampleGrab)
                .loop(r::SampleGrab)
                .transitionTimed(0.25, States.CLOSING_CLAW)

                .state(States.CLOSING_CLAW)
                .onEnter(r::ClawClose)
                .transition(() -> g.b, States.WAIT1) // ESCAPE
                .transition(() -> g.x, States.WAIT3)

                .state(States.WAIT3)
                .transitionTimed(0.25, States.LoiterSample)

                .state(States.LoiterSample) //Has the sample but the pivot and extention are down
                .onEnter(r::LoiterSample) //Does not move Pivot back anymore
                .loop(r::LoiterSample)
                .transition(() -> g.a, States.WAIT5) //obs zone drop
                .transition(() -> g.x, States.WAIT8) //primary high basket
                .transition(()->g.b, States.WAIT1)// ESCAPE


                //OBS ZONE STATES START



                .state(States.WAIT5)
                .transitionTimed(0.25, States.ObsZoneRelease)

                .state(States.ObsZoneRelease) //Moves Pivot and arm to the position
                .onEnter(r::ObszoneScoreing)
                .loop(r::ObszoneScoreing)
                .transition(()-> g.x,States.WAIT7) // primary to basketpos

                .state(States.WAIT7) //drops sample in the obs zone
                .onEnter(r::Score)
                .transitionTimed(0.25,States.Stationary)

                //OBS ZONE STATES END


                // HIGH BASKET STATES START

                .state(States.WAIT8)
                .transitionTimed(0.25,States.PivotOverCenter)

                .state(States.PivotOverCenter) //Moves pivot
                .onEnter(r::MovingPivotforHighBasket)
                .loop(r::MovingPivotforHighBasket)
                .transitionTimed(1, States.BasketExtend)

//                .state(States.WAIT6)
//                .transitionTimed(0.25, States.BasketExtend)

                .state(States.BasketExtend)
                .onEnter(r::MovingExtentionforHighBasket)
                .loop(r::MovingExtentionforHighBasket)
                .transition(()-> g.x,States.WAIT9)

                .state(States.WAIT9)
                .transitionTimed(0.25,States.SCORE)

                .state(States.SCORE)// just opens claw at highbasket
                .onEnter(r::Score)
                .transitionTimed(0.5, States.BasketPosition2)

                .state(States.BasketPosition2)// no hang on high basket
                .onEnter(r::BasketPos2)
                .loop(r::BasketPos2)
                .transitionTimed(0.5, States.Stationary)

                .build();
    }

    enum States {

        Stationary, CLOSING_CLAW, LoiterSample, SampleHover, SampleGrab, BasketExtend, BasketPosition2,PivotOverCenter, ObsZoneRelease, PivotBack, SpecimenWall, SpecimenWallGrab, SpecimenPreScore, SpecimenLatch, WAIT1, WAIT2, WAIT3, WAIT4, WAIT5, WAIT6, WAIT7, WAIT8,WAIT9,WAIT10, SpecimenWallGrabUp, ExtensionDown, SCORE, Passover, SpecimenEscapeState

    }

}