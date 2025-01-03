//package org.firstinspires.ftc.teamcode.subsystems.Deposit;
//
//import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.doorClose;
//import static org.firstinspires.ftc.teamcode.subsystems.RobotConstants.doorOpen;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.Actions;
//import org.firstinspires.ftc.teamcode.subsystems.pedroPathing.RunAction;
//
//public class DoorSubsystem {
//
//    public enum DoorState {
//        CLOSED, OPEN
//    }
//
//    private Servo door;
//    private DoorState state;
//    public RunAction openDoor, closeDoor;
//
//    public DoorSubsystem(HardwareMap hardwareMap, DoorState doorState) {
//        door = hardwareMap.get(Servo.class, "trans");
//        this.state = doorState;
//
//        openDoor = new RunAction(this::openDoor);
//        closeDoor = new RunAction(this::closeDoor);
//    }
//
//    public void setPos(double doorPos) {
//        door.setPosition(doorPos);
//    }
//
//    public void setState(DoorState doorState) {
//        if (doorState == DoorState.CLOSED) {
//            door.setPosition(doorClose);
//            this.state = DoorState.CLOSED;
//        } else if (doorState == DoorState.OPEN) {
//            door.setPosition(doorOpen);
//            this.state = DoorState.OPEN;
//        }
//    }
//
//    public void switchState() {
//        if (state == DoorState.CLOSED) {
//            setState(DoorState.OPEN);
//        } else if (state == DoorState.OPEN) {
//            setState(DoorState.CLOSED);
//        }
//    }
//
//    public void openDoor() {
//        setState(DoorState.OPEN);
//    }
//
//    public void closeDoor() {
//        setState(DoorState.CLOSED);
//    }
//
//    public void init() {
//        Actions.runBlocking(openDoor);
//    }
//
//    public void start() {
//        Actions.runBlocking(closeDoor);
//    }
//
//
//
//}