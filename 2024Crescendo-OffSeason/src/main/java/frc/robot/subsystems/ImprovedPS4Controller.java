package frc.robot.subsystems;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

public class ImprovedPS4Controller extends PS4Controller {
    /** Represents a digital button on an XboxController. */
    public enum Button {
        kAutoButton(0),
        /** Square button. */
        kSquare(1),
        /** X button. */
        kCross(2),
        /** Circle button. */
        kCircle(3),
        /** Triangle button. */
        kTriangle(4),
        /** Left Trigger 1 button. */
        kL1(5),
        /** Right Trigger 1 button. */
        kR1(6),
        /** Left Trigger 2 button. */
        kL2(7),
        /** Right Trigger 2 button. */
        kR2(8),
        /** Share button. */
        kShare(9),
        /** Option button. */
        kOptions(10),
        /** Left stick button. */
        kL3(11),
        /** Right stick button. */
        kR3(12),
        /** PlayStation button. */
        kPS(13),
        /** Touchpad click button. */
        kTouchpad(14);
        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }

        /**
         * Get the human-friendly name of the button, matching the relevant methods. This is done by
         * stripping the leading `k`, and if not a Bumper button append `Button`.
         *
         * <p>Primarily used for automated unit tests.
         *
         * @return the human-friendly name of the button.
         */
        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Bumper")) {
                return name;
            }
            if (name.endsWith("Trigger")) {
                return name;
            }
            return name + "Button";
        }
    }
    private double m_triggerThreshold = 0.0;
    public ImprovedPS4Controller(final int port) {
        super(port);
        m_triggerThreshold = 0.5;
        //TODO Auto-generated constructor stub
    }
    /**
     * Read the value of the left trigger (LT) button on the controller.
     *
     * @return The axis of the trigger is greater than the trigger threshold
     */
    public boolean getLeftTrigger(){
        return getL2Axis() > m_triggerThreshold;
    }
    /**
     * Read the value of the right trigger (RT) button on the controller.
     *
     * @return The axis of the trigger is greater than the trigger threshold
     */
    public boolean getRightTrigger(){
        return getR2Axis() > m_triggerThreshold;
    }
    /**
     * Get the button value (starting at button 1).
     *
     * <p>The buttons are returned in a single 16 bit value with one bit representing the state of
     * each button. The appropriate button is returned as a boolean value.
     *
     * <p>This method returns true if the button is being held down at the time that this method is
     * being called.
     *
     * @param button The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getButton(int button){
        if(button==Button.kAutoButton.value)
        {
            return true;
        }
        if(button == Button.kL2.value){
            return getLeftTrigger();
        }
        if(button == Button.kR2.value){
            return getRightTrigger();
        }
        return getRawButton(button);
    }
    public boolean POVPressed(){
        return this.getPOV()>=0;
    }
    public boolean getPOVUp(){
        return this.getPOV()==0;
    }
    public boolean getPOVRight(){
        return this.getPOV()==90;
    }
    public boolean getPOVDown(){
        return this.getPOV()==180;
    }
    public boolean getPOVLeft(){
        return this.getPOV()==270;
    }
}
