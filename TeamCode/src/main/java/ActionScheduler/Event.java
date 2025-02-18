package ActionScheduler;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.LinkedList;

public class Event {

    //Event is a group of tasks timed apart

    ElapsedTime timer;

    LinkedList<TimedAction> EventList;

public Event(){
    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    EventList = new LinkedList<>();
}

public void update(){
    //keep going through chain while list has Timed Actions and previous action is finsihed
    while(!EventList.isEmpty() && EventList.getFirst().time <= timer.seconds()){
EventList.removeFirst().t.run();
    }
}

public void addTimedAction(Task t,double Seconds){
        EventList.addLast(new TimedAction(t,Seconds + timer.seconds()));
        //no need to create new event class everytime

    }

class TimedAction{
Task t;
double time;
public TimedAction(Task task,double Seconds){
    t = task;
    time = Seconds;
}} //timed action end class




}
