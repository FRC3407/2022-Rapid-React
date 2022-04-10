package frc.robot.team3407;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.team3407.commandbased.ToggleTrigger;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;


public class Input {

	public static interface AnalogSupplier extends DoubleSupplier {
        double get();
        @Override default double getAsDouble() { return get(); }
    }
    public static interface DigitalSupplier extends BooleanSupplier {
        boolean get();
        @Override default boolean getAsBoolean() { return get(); }
    }

    /**
     * AnalogSlewSupplier limits the rate output (between calls) to a maximum of that given in units/sec
     */
    public static class AnalogSlewSupplier implements AnalogSupplier {

        private final SlewRateLimiter limit;
        private final AnalogSupplier source;

        public AnalogSlewSupplier(AnalogSupplier src) { this(src, Double.MAX_VALUE); }  // kind of pointless?
        public AnalogSlewSupplier(AnalogSupplier src, double mrate) {
            this.limit = new SlewRateLimiter(mrate, src.get());
            this.source = src;
        }

        @Override public double get() {
            return this.limit.calculate(this.source.get());
        }


    }


    public static class InputDevice extends GenericHID {

        private PovButton[] buttons = null;
        private boolean verifyInfo() {
            if(super.isConnected() && this.buttons == null) {
                this.buttons = new PovButton[super.getButtonCount() + (super.getPOVCount() * 4)];
                return true;
            } else if(!super.isConnected() && this.buttons != null) {
                this.buttons = null;
                return false;
            }
            return this.buttons != null;
        }

        public InputDevice(int p) { 
            super(p); 
            this.verifyInfo();
        }

        @Override
        public boolean getRawButton(int button) {
            int _bc = super.getButtonCount();
            if((button > _bc) && (button <= _bc + (super.getPOVCount() * 4))) {
                button = button - _bc;
                return (super.getPOV((button-1) / 4) / 90.0 + 1) == button;
            } else {
                return super.getRawButton(button);
            }
        }

        public PovButton getCallback(int button) {  // returns dummy if joystick not connected
            if(this.verifyInfo()) {
                if(this.buttons[button-1] == null) {
                    this.buttons[button-1] = new PovButton(this, button);
                }
                return this.buttons[button-1];
            }
            return PovButton.dummy;
        }

        public Trigger connectionTrigger() {
            return new Trigger(()->{ return super.isConnected(); });
        }

    }

    /** Converts any pov's on a controller into button values past those that would normally be assigned */
	public static class PovButton extends Button {
        public static final PovButton dummy = new PovButton();
		private final int 
			port, button;
		private final boolean
			use_pov;

        private PovButton() {
            this.port = DriverStation.kJoystickPorts-1;
            this.button = 0;
            this.use_pov = false;
        }
		public PovButton(GenericHID device, int button) { this(device.getPort(), button); }
		public PovButton(int port, int button) {
			this.port = port;
            int _bc = DriverStation.getStickButtonCount(this.port), _pc = DriverStation.getStickPOVCount(this.port);
            this.use_pov = ((button > _bc) && (button <= _bc + (_pc * 4)));
            if (this.use_pov) { 
				this.button = button - _bc; 
			} else { 
				this.button = button; 
			}
		}
		@Override
        public boolean get() {
            if (this.use_pov) {
                return (DriverStation.getStickPOV(this.port, (this.button-1) / 4) / 90.0 + 1) == this.button;
            } else {
                return DriverStation.getStickButton(this.port, this.button);
            }
        }
        @Override public Button whenPressed(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.whenActive(command, interruptible); }
            return this;
        }
        @Override public Button whenPressed(final Command command) {
            if(this != PovButton.dummy) { super.whenActive(command); }
            return this;
        }
        @Override public Button whenPressed(final Runnable toRun, Subsystem... requirements) {
            if(this != PovButton.dummy) { super.whenActive(toRun, requirements); }
            return this;
        }
        @Override public Button whileHeld(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.whileActiveContinuous(command, interruptible); }
            return this;
        }
        @Override public Button whileHeld(final Command command) {
            if(this != PovButton.dummy) { super.whileActiveContinuous(command); }
            return this;
        }
        @Override public Button whileHeld(final Runnable toRun, Subsystem... requirements) {
            if(this != PovButton.dummy) { super.whileActiveContinuous(toRun, requirements); }
            return this;
        }
        @Override public Button whenHeld(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.whileActiveOnce(command, interruptible); }
            return this;
        }
        @Override public Button whenHeld(final Command command) {
            if(this != PovButton.dummy) { super.whileActiveOnce(command, true); }
            return this;
        }
        @Override public Button whenReleased(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.whenInactive(command, interruptible); }
            return this;
        }
        @Override public Button whenReleased(final Command command) {
            if(this != PovButton.dummy) { super.whenInactive(command); }
            return this;
        }
        @Override public Button whenReleased(final Runnable toRun, Subsystem... requirements) {
            if(this != PovButton.dummy) { super.whenInactive(toRun, requirements); }
            return this;
        }
        @Override public Button toggleWhenPressed(final Command command, boolean interruptible) {
            if(this != PovButton.dummy) { super.toggleWhenActive(command, interruptible); }
            return this;
        }
        @Override public Button toggleWhenPressed(final Command command) {
            if(this != PovButton.dummy) { super.toggleWhenActive(command); }
            return this;
        }
        @Override public Button cancelWhenPressed(final Command command) {
            if(this != PovButton.dummy) { super.cancelWhenActive(command); }
            return this;
        }
	}

    public static interface AnalogMap {
        int getValue();
        int getTotal();
        default boolean compatible(GenericHID i) { return i.getAxisCount() == this.getTotal(); }
        default boolean compatible(int p) { return DriverStation.getStickAxisCount(p) == this.getTotal(); }
        default double getValueOf(GenericHID i) {
            if(this.compatible(i)) {
                return i.getRawAxis(this.getValue());
            }
            return 0.0;
        }
        default double getValueOf(int p) {
            if(this.compatible(p)) {
                return DriverStation.getStickAxis(p, this.getValue());
            }
            return 0.0;
        }
        default AnalogSupplier getSupplier(InputDevice i) {
            if(this.compatible(i)) {
                return ()->i.getRawAxis(this.getValue());
            }
            return ()->0.0;
        }
        default AnalogSupplier getSupplier(int p) {
            if(this.compatible(p)) {
                return ()->DriverStation.getStickAxis(p, this.getValue());
            }
            return ()->0.0;
        }
        default AnalogSlewSupplier getLimitedSupplier(InputDevice i, double mrate) {
            if(this.compatible(i)) {
                return new AnalogSlewSupplier(()->i.getRawAxis(this.getValue()), mrate);
            }
            return new AnalogSlewSupplier(()->0.0);
        }
        default AnalogSlewSupplier getLimitedSupplier(int p, double mrate) {
            if(this.compatible(p)) {
                return new AnalogSlewSupplier(()->DriverStation.getStickAxis(p, this.getValue()), mrate);
            }
            return new AnalogSlewSupplier(()->0.0);
        }
    }
    public static interface DigitalMap {
        int getValue();
        int getTotal();
        default boolean compatible(GenericHID i) {
            return i.getButtonCount() + (i.getPOVCount()*4) == this.getTotal();
        }
        default boolean compatible(int p) {
            return DriverStation.getStickButtonCount(p) + (DriverStation.getStickPOVCount(p)*4) == this.getTotal();
        }
        default boolean isPovBindOf(GenericHID i) {
            if(this.compatible(i)) {
                return this.getValue() > i.getButtonCount();
            }
            return false;
        }
        default boolean isPovBindOf(int p) {
            if(this.compatible(p)) {
                return this.getValue() > DriverStation.getStickPOVCount(p);
            }
            return false;
        }
        default int getPovBindOf(GenericHID i) {
            if(this.compatible(i) && this.isPovBindOf(i)) {
                return (this.getValue() - i.getButtonCount())/4;
            }
            return -1;
        }
        default int getPovBindOf(int p) {
            if(this.compatible(p) && this.isPovBindOf(p)) {
                return (this.getValue() - DriverStation.getStickButtonCount(p))/4;
            }
            return -1;
        }
        default PovButton getCallbackFrom(InputDevice i) {
            if(this.compatible(i)) {
                return i.getCallback(this.getValue());
            }
            return PovButton.dummy;
        }
        default PovButton getCallbackFrom(GenericHID i) {
            if(this.compatible(i)) {
                return new PovButton(i, this.getValue());
            }
            return PovButton.dummy;
        }
        default PovButton getCallbackFrom(int p) {
            if(this.compatible(p)) {
                return new PovButton(p, this.getValue());
            }
            return PovButton.dummy;
        }
        default ToggleTrigger getToggleFrom(InputDevice i) {
            return new ToggleTrigger(getCallbackFrom(i));
        }
        default ToggleTrigger getToggleFrom(GenericHID i) {
            return new ToggleTrigger(getCallbackFrom(i));
        }
        default ToggleTrigger getToggleFrom(int p) {
            return new ToggleTrigger(getCallbackFrom(p));
        }
        default boolean getValueOf(GenericHID i) {
            if(this.isPovBindOf(i)) {
                return (i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
            } else if(this.compatible(i)) {
                return i.getRawButton(this.getValue());
            }
            return false;
        }
        default boolean getValueOf(int p) {
            if(this.isPovBindOf(p)) {
                return (DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
            } else if(this.compatible(p)) {
                return DriverStation.getStickButton(p, this.getValue());
            }
            return false;
        }
        default boolean getPressedValueOf(GenericHID i) {
            if(this.isPovBindOf(i)) {
                return (i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
            } else if(this.compatible(i)) {
                return i.getRawButtonPressed(this.getValue());
            }
            return false;
        }
        default boolean getPressedValueOf(int p) {
            if(this.isPovBindOf(p)) {
                return (DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
            } else if(this.compatible(p)) {
                return DriverStation.getStickButtonPressed(p, this.getValue());
            }
            return false;
        }
        default boolean getReleasedValueOf(GenericHID i) {
            if(this.isPovBindOf(i)) {
                return (i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
            } else if(this.compatible(i)) {
                return i.getRawButtonReleased(this.getValue());
            }
            return false;
        }
        default boolean getReleasedValueOf(int p) {
            if(this.isPovBindOf(p)) {
                return (DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
            } else if(this.compatible(p)) {
                return DriverStation.getStickButtonReleased(p, this.getValue());
            }
            return false;
        }
        default DigitalSupplier getSupplier(GenericHID i) {
            if(this.isPovBindOf(i)) {
                return ()->(i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
            } else if(this.compatible(i)) {
                return ()->i.getRawButton(this.getValue());
            }
            return ()->false;
        }
        default DigitalSupplier getSupplier(int p) {
            if(this.isPovBindOf(p)) {
                return ()->(DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
            } else if(this.compatible(p)) {
                return ()->DriverStation.getStickButton(p, this.getValue());
            }
            return ()->false;
        }
        default DigitalSupplier getPressedSupplier(GenericHID i) {
            if(this.isPovBindOf(i)) {
                return ()->(i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
            } else if(this.compatible(i)) {
                return ()->i.getRawButtonPressed(this.getValue());
            }
            return ()->false;
        }
        default DigitalSupplier getPressedSupplier(int p) {
            if(this.isPovBindOf(p)) {
                return ()->(DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
            } else if(this.compatible(p)) {
                return ()->DriverStation.getStickButtonPressed(p, this.getValue());
            }
            return ()->false;
        }
        default DigitalSupplier getReleasedSupplier(GenericHID i) {
            if(this.isPovBindOf(i)) {
                return ()->(i.getPOV((this.getValue() - i.getButtonCount()-1) / 4) / 90.0 + 1) == this.getValue() - i.getButtonCount();
            } else if(this.compatible(i)) {
                return ()->i.getRawButtonReleased(this.getValue());
            }
            return ()->false;
        }
        default DigitalSupplier getReleasedSupplier(int p) {
            if(this.isPovBindOf(p)) {
                return ()->(DriverStation.getStickPOV(p, (this.getValue() - DriverStation.getStickButtonCount(p)-1) / 4) / 90.0 + 1) == this.getValue() - DriverStation.getStickButtonCount(p);
            } else if(this.compatible(p)) {
                return ()->DriverStation.getStickButtonReleased(p, this.getValue());
            }
            return ()->false;
        }
    }


	public static class Xbox {
        public static enum Analog implements AnalogMap {
            LX(0), RX(4), LY(1), RY(5), LT(2), RT(3), 
            TOTAL(6);
        
            public final int value;
            private Analog(int value) { this.value = value; }

            public int getValue() { return this.value; }
            public int getTotal() { return TOTAL.value; }
        }
        public static enum Digital implements DigitalMap {
            LB(5), RB(6), LS(9), RS(10),
            A(1), B(2), X(3), Y(4),
            BACK(7), START(8),
            DT(11), DR(12), DB(13), DL(14),  // Dpad buttons (only valid with PovButton objects)
            TOTAL(14);
        
            public final int value;
            private Digital(int value) { this.value = value; }

            public int getValue() { return this.value; }
            public int getTotal() { return TOTAL.value; }
        }
    }
    public static class PlayStation {
        public static enum Analog implements AnalogMap {
            LX(0), LY(1), RX(2), RY(5), LT(3), RT(4),
            TOTAL(6);

            public final int value;
            private Analog(int value) { this.value = value; }

            public int getValue() { return this.value; }
            public int getTotal() { return TOTAL.value; }
        }
        public static enum Digital implements DigitalMap {
            SQR(1), X(2), O(3), TRI(4),     // square, cross, circle, triangle
            LB(5), RB(6), L2(7), R2(8),     // right-bumper, left-bumper,   left-trigger, right-trigger (button mode)
            SHARE(9), OPT(10), PS(13),      // share, options, ps button
            TOUCH(14), LS(11), RS(12),      // touchpad, left-stick, right-stick
            TOTAL(14);

            public final int value;
            private Digital(int value) { this.value = value; }

            public int getValue() { return this.value; }
            public int getTotal() { return TOTAL.value; }
        }
    }
    public static class Attack3 {
        public static enum Analog  implements AnalogMap {
            X(0), Y(1), S(2),   // ~ X-Axis, Y-Axis, Slider thing on the bottom
            TOTAL(3);

            public final int value;
            private Analog(int value) { this.value = value; }

            public int getValue() { return this.value; }
            public int getTotal() { return TOTAL.value; }
        }
        public static enum Digital implements DigitalMap {
            TRI(1), TB(2), TT(3), TL(4), TR(5),             // ~ trigger, top-bottom, top-top, top-left, top-right
            B1(6), B2(7), B3(8), B4(9), B5(10), B6(11),     // ~ buttons on the base of the joystick (labeled)
            TOTAL(11);

            public final int value;
            private Digital(int value) { this.value = value; }

            public int getValue() { return this.value; }
            public int getTotal() { return TOTAL.value; }
        }
    }
    public static class Extreme3d {
        public static enum Analog  implements AnalogMap {
            X(0), Y(1), Z(2), S(3),     // x-axis, y-axis, swivell-axis, slider-axis
            TOTAL(4);

            public final int value;
            private Analog(int value) { this.value = value; }

            public int getValue() { return this.value; }
            public int getTotal() { return TOTAL.value; }
        }
        public static enum Digital implements DigitalMap {
            TRI(1), SIDE(2), TLB(3), TRB(4), TLT(5), TRT(6),    // trigger, side, top-left-bottom, top-right-bottom, top-left-top, top-right-top
            B7(7), B8(8), B9(9), B10(10), B11(11), B12(12),     // as printed on the actual joystick
            TOTAL(12);

            public final int value;
            private Digital(int value) { this.value = value; }

            public int getValue() { return this.value; }
            public int getTotal() { return TOTAL.value; }
        }
    }

}