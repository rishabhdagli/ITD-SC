package Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;


public class StateMachineGenerator {
    public static double num = 0;
    public static StateMachine GenerateSpecimenMachine(Gamepad g, Gamepad z, Robot r) {
        return new StateMachineBuilder()

                // X is primary, A is secondary, B is escape for logitech gamepad

                .state(States.SpecimenWall) // Position to grab from the wall
                .onEnter(r::SpecimenWall)
                .loop(r::SpecimenWall)
                .transition(() -> g.x, States.WAIT1)
                .transition(()-> z.a, States.WAIT10)//hang


                .state(States.WAIT1)
                .transitionTimed(0.25, States.SpecimenWallGrab)


                .state(States.SpecimenWallGrab)   // Grabs the things
                .onEnter(r::SpecimenWallGrab)
                .loop(r::SpecimenWallGrab)
                .transition(() -> g.x, States.WAIT2)//lift of thing
                .transition(()-> g.b, States.WAIT6) //escape state
                .transition(()-> z.a, States.WAIT10)//hang


                .state(States.WAIT6) //excape state wait
                .transitionTimed(0.25,States.SpecimenWall) // goes back to position to grab from wall
                .transition(()-> z.a, States.WAIT10)



                .state(States.WAIT2)
                .transitionTimed(0.25, States.SpecimenWallGrabUp)
                .transition(()-> z.a, States.WAIT10)


                .state(States.SpecimenWallGrabUp) //(prolly need to change) bring pivot up
                .onEnter(r::SpecimenWallUp)
                .loop(r::SpecimenWallUp)
                .transition(() -> g.x, States.WAIT3)
                .transition(()-> g.b, States.WAIT6) //in case karthik has autism last escape states
                .transition(()-> z.a, States.WAIT10)



                .state(States.WAIT3)
                .transitionTimed(0.25, States.SpecimenPreScore)


                .state(States.SpecimenPreScore) // Bout to score
                .onEnter(r::SpecimenPreScore)
                .loop(r::SpecimenPreScore)
                .transition(() -> g.x, States.WAIT4)//highchamber score
                .transition(()-> z.a, States.WAIT10)

                .state(States.WAIT4)
                .transitionTimed(0.25, States.SpecimenPostScore)
                .transition(()-> z.a, States.WAIT10)


                .state(States.SpecimenPostScore) //Scores
                .onEnter(r::SpecimenPostScore)
                .loop(r::SpecimenPostScore)
                .transitionTimed(1, States.SpecimenWall)
                .transition(()-> z.a, States.WAIT10)


                // Escape state
                .state(States.WAIT6)
                .transitionTimed(0.25, States.SpecimenWall)
                .transition(()-> z.a, States.WAIT10)



                //Hang States


                .state(States.WAIT10)
                .transitionTimed(0.5, States.HANGUP)


                .state(States.HANGUP)
                //.onEnter(r::HangUp)
                //.loop(r::HangUp)
                .transition(()->z.a, States.HANGDOWN)


                .state(States.WAIT11)
                .transitionTimed(0.5, States.HANGDOWN)


                .state(States.HANGDOWN)
               // .onEnter(r::HangDown)
                //.loop(r::HangDown)
                .transition(()-> z.a, States.WAIT10)



                .build();
    }

    // X is primary, A is secondary, B is escape for logitech gamepad
    public static StateMachine GenerateSampleMachine(Gamepad g, Gamepad z, Robot r) {
        return new StateMachineBuilder()
                .state(States.Stationary)
                .onEnter(r::Loiter)
                .loop(r::Loiter)
                .transition(() -> g.x, States.WAIT1)
                .transition(()-> z.a, States.WAIT10) //hang


                .state(States.WAIT1)
                .transitionTimed(0.25, States.SampleHover)
                .transition(()-> z.a, States.WAIT10)//hang


                .state(States.SampleHover)
                .onEnter(r::SampleHover)
                .loop(r::SampleHover)
                .transition(()->g.x, States.WAIT2)
                .transition(()-> z.a, States.WAIT10)//hang


                .state(States.WAIT2)
                .transitionTimed(0.25, States.SampleGrab)


                .state(States.SampleGrab)
                .onEnter(r::SampleGrab)
                .loop(r::SampleGrab)
                .transitionTimed(0.25, States.CLOSING_CLAW)


                .state(States.CLOSING_CLAW)
                .onEnter(r::InsideGrabBeakOpen)
                .transition(() -> g.b, States.WAIT1) // ESCAPE
                .transition(() -> g.x, States.WAIT3)
                .transition(()-> z.a, States.WAIT10)//hang


                .state(States.WAIT3)
                .transitionTimed(0.25, States.LoiterSample)
                .transition(()-> z.a, States.WAIT10)


                .state(States.LoiterSample) //Has the sample but the pivot and extention are down
                .onEnter(r::LoiterSample) //Does not move Pivot back anymore
                .loop(r::LoiterSample)
                .transition(() -> g.a, States.WAIT5) //obs zone drop
                .transition(() -> g.x, States.WAIT8) //primary high basket
                .transition(()->g.b, States.WAIT1)// ESCAPE
                .transition(()-> z.a, States.WAIT10)



                //OBS ZONE STATES START



                .state(States.WAIT5)
                .transitionTimed(0.25, States.ObsZoneRelease)
                .transition(()-> z.a, States.WAIT10)


                .state(States.ObsZoneRelease) //Moves Pivot and arm to the position
                .onEnter(r::obsZoneRelease)
                .loop(r::obsZoneRelease)
                .transition(()-> g.x,States.WAIT7) // primary to basketpos
                .transition(()-> z.a, States.WAIT10)

                .state(States.WAIT7) //drops sample in the obs zone
                .onEnter(r::ObsZoneScore)
                .transitionTimed(0.25,States.Stationary)
                .transition(()-> z.a, States.WAIT10)



                //OBS Zone states end



                .state(States.WAIT8)
                .transitionTimed(0.25,States.PivotOverCenter)


                .state(States.PivotOverCenter) //Moves pivot
                .onEnter(r::PivotBack)
                .loop(r::PivotBack)
                .transitionTimed(1, States.BasketExtend)


//                .state(States.WAIT6)
//                .transitionTimed(0.25, States.BasketExtend)

                .state(States.BasketExtend)
                .onEnter(r::BasketExtension)
                .loop(r::BasketExtension)
                .transition(()-> g.x,States.WAIT9)
                .transition(()-> z.a, States.WAIT10) //hang


                .state(States.WAIT9)
                .transitionTimed(0.25,States.SCORE)

                .state(States.SCORE)// just opens claw at highbasket
                .onEnter(r::BasketScore)
                .transitionTimed(0.5, States.BasketPosition2)
                .transition(()-> z.a, States.WAIT10)


                .state(States.BasketPosition2)// no hang on high basket
                .onEnter(r::BasketReturn)
                .loop(r::BasketReturn)
                .transitionTimed(1, States.Stationary)
                .transition(()-> z.a, States.WAIT10)



                //HANG STATES


                .state(States.WAIT10)
                .transitionTimed(0.5, States.HANGUP)

                .state(States.HANGUP)
              //  .onEnter(r::HangUp)
                //.loop(r::HangUp)
                .transition(()->z.a, States.HANGDOWN)

                .state(States.WAIT11)
                .transitionTimed(0.5, States.HANGDOWN)

                .state(States.HANGDOWN)
                // .onEnter(r::HangDown)
                //.loop(r::HangDown)
                .transition(()-> z.a, States.WAIT10)

                .build();
    }

    enum States {

        Stationary, HANGUP, HANGDOWN, CLOSING_CLAW, LoiterSample, SampleHover, SampleGrab, BasketExtend, BasketPosition2,PivotOverCenter, ObsZoneRelease, PivotBack, SpecimenWall, SpecimenWallGrab, SpecimenPreScore, SpecimenPostScore, WAIT1, WAIT2, WAIT3, WAIT4, WAIT5, WAIT6, WAIT7, WAIT8,WAIT9,WAIT10, WAIT11, SpecimenWallGrabUp, ExtensionDown, SCORE, Passover, SpecimenEscapeState

    }

}