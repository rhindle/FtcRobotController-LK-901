package org.firstinspires.ftc.teamcode.robot.Universal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ButtonMgr {

    public LinearOpMode opMode;
    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public ControlData[] controlData;

    ////////////////
    //constructors//
    ////////////////
    public ButtonMgr(LinearOpMode opMode){
        construct(opMode);
    }

    void construct(LinearOpMode opMode){
        this.opMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        //allocate for # objects based on GPbuttons enum
        controlData = new ControlData[Buttons.values().length * 2];
        //create objects and assign index numbers
        for (int i = 0; i < Buttons.values().length * 2; i++) {
            controlData[i] = new ControlData();
            controlData[i].initData(i);
        }
    }

    public void loop() {
        updateAll();
    }

    public void updateAll()
    {
        for (ControlData i : controlData) {
            i.update();
        }
    }

    int getIndex(int controller, Buttons button){
        //This converts an index of 0-27 based on the controller 1-2 and button 0-13
        if (controller < 1 || controller > 2) controller = 0; else controller--;
        return controller * Buttons.values().length + button.ordinal();
    }

    ControlData getAllData(int controller, Buttons button) {
        return controlData[getIndex(controller, button)];
    }

    public boolean wasPressed(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].pressed;
    }
    public boolean wasReleased(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].released;
    }
    public boolean wasTapped(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].tapped;
    }
    public boolean isHeld(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].held;
    }
    public boolean isPressed(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].lastStatus;
    }

    public enum Buttons {  //must match what is in getReading's switch block
        dpad_up,
        dpad_down,
        dpad_left,
        dpad_right,
        a,
        b,
        x,
        y,
        start,
        back,
        left_bumper,
        right_bumper,
        left_stick_button,
        right_stick_button;
    }

    class ControlData {
        int index;
        Buttons name;
        boolean lastStatus;
        long lastTime;
        boolean tapped;
        boolean doubleTapped;
        boolean held;
        boolean pressed;
        boolean released;

        public void initData(int index)  //object
        {
            this.index = index;
            name = Buttons.values()[index % Buttons.values().length];
            lastStatus = false;
            lastTime = System.currentTimeMillis();
            tapped = false;
            doubleTapped = false;
            held = false;
            pressed = false;
            released = false;
        }

        boolean getReading(int index)
        {
            Gamepad gpad;
            if (index >= Buttons.values().length) {
                index -= Buttons.values().length;
                gpad = gamepad2;
            } else {
                gpad = gamepad1;
            }
            switch (Buttons.values()[index]) {
                //must match the elements in the GPbuttons enum
                case dpad_up:             return gpad.dpad_up;
                case dpad_down:           return gpad.dpad_down;
                case dpad_left:           return gpad.dpad_left;
                case dpad_right:          return gpad.dpad_right;
                case a:                   return gpad.a;
                case b:                   return gpad.b;
                case x:                   return gpad.x;
                case y:                   return gpad.y;
                case start:               return gpad.start;
                case back:                return gpad.back;
                case left_bumper:         return gpad.left_bumper;
                case right_bumper:        return gpad.right_bumper;
                case left_stick_button:   return gpad.left_stick_button;
                case right_stick_button:  return gpad.right_stick_button;
                default:                  return false;//something bad happened
            }
        }

        public void update()
        {
            long currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - lastTime;
            boolean currentState = getReading(index);
            if (!lastStatus && currentState) {  // change from not pressed to pressed
                pressed = true;          // this will last for one loop!
                held = false;
                tapped = false;
                lastTime = currentTime;  // reset the time
            } else {
                pressed = false;
            }
            if (lastStatus && !currentState) {  // change from pressed to not pressed
                released = true;         // this will last for one loop!
                held = false;
                if (deltaTime < 500) {
                    tapped = true;
                }
                lastTime = currentTime;  // reset the time
            } else {
                released = false;
            }
            if (lastStatus && currentState) {   // still held
                tapped = false;
                if (deltaTime >= 500) {
                    held = true;
                }
            }
            if (!lastStatus && !currentState) {  // still not held
                tapped = false;
                held = false;
            }
            //need something for doubleTap (put this off until proof-of-concept works)
            //after all the checks are done...
            lastStatus = currentState;
        }
    }
}
