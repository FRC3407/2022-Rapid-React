package frc.robot.outdated;
// package frc.robot;

// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj2.command.button.Button;
// import edu.wpi.first.wpilibj.DriverStation;

// //import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

// //import frc.robot.commands.*;

// public class Input implements Constants.Controls {

//     public static enum Type {
//         XBOX,   // xbox controller
//         PS_,    // playstation controller
//         ATK,    // attack3 joystick
//         EXT;    // extreme3d joystick
//     }

//     public static class Xbox {
//         public static enum Digital {
//             LB(5), RB(6), LS(9), RS(10),
//             A(1), B(2), X(3), Y(4),
//             BACK(7), START(8),
//             DT(11), DR(12), DB(13), DL(14);     // Dpad buttons (only valid with XButton objects)
        
//             public final int value;
//             private Digital(int value) { this.value = value; }
//         }
//         public static enum Analog {
//             LX(0), RX(4), LY(1), RY(5), LT(2), RT(3);
        
//             public final int value;
//             private Analog(int value) { this.value = value; }
//         }
//     }
//     public static class PlayStation {
//         public static enum Digital {
//             SQR(1), X(2), O(3), TRI(4),     // square, cross, circle, triangle
//             LB(5), RB(6), L2(7), R2(8),     // right-bumper, left-bumper,   left-trigger, right-trigger (button mode)
//             SHARE(9), OPT(10), PS(13),      // share, options, ps button
//             TOUCH(14), LS(11), RS(12);      // touchpad, left-stick, right-stick

//             public final int value;
//             private Digital(int value) { this.value = value; }
//         }
//         public static enum Analog {
//             LX(0), LY(1), RX(2), RY(5), LT(3), RT(4);

//             public final int value;
//             private Analog(int value) { this.value = value; }
//         }
//     }
//     public static class Attack3 {
//         public static enum Digital {
//             TRI(1), TB(2), TT(3), TL(4), TR(5),             // ~ trigger, top-bottom, top-top, top-left, top-right
//             B1(6), B2(7), B3(8), B4(9), B5(10), B6(11);     // ~ buttons on the base of the joystick (labeled)

//             public final int value;
//             private Digital(int value) { this.value = value; }
//         }
//         public static enum Analog {
//             X(0), Y(1), S(2);   // ~ X-Axis, Y-Axis, Slider thing on the bottom

//             public final int value;
//             private Analog(int value) { this.value = value; }
//         }
//     }
//     public static class Extreme3d {
//         public static enum Digital {
//             TRI(1), SIDE(2), TLB(3), TRB(4), TLT(5), TRT(6),    // trigger, side, top-left-bottom, top-right-bottom, top-left-top, top-right-top
//             B7(7), B8(8), B9(9), B10(10), B11(11), B12(12);     // as printed on the actual joystick

//             public final int value;
//             private Digital(int value) { this.value = value; }
//         }
//         public static enum Analog {
//             X(0), Y(1), Z(2), S(3);     // x-axis, y-axis, swivell-axis, slider-axis

//             public final int value;
//             private Analog(int value) { this.value = value; }
//         }
//     }



//     private static final DriverStation source = DriverStation.getInstance();
//     private final Constants.Controls controller;

//     //this class maps integers above the normal button counts to any POV's on the device so that they can be used as normal buttons
//     public static class XButton extends Button {

//         private final int device, button;
//         private final boolean use_pov;

//         public XButton(int device, int button) {
//             this.device = device;
//             int _bc = source.getStickButtonCount(this.device), _pc = source.getStickPOVCount(this.device);
//             this.use_pov = ((button > _bc) && (button <= _bc + (_pc * 4)));
//             if (use_pov) {
//                 this.button = button - _bc;
//             } else {
//                 this.button = button;
//             }

//             // System.out.println("Buttons: " + _bc);
//             // System.out.println("POV's: " + _pc);
//             // System.out.println(use_pov);
//         }
//         public XButton(GenericHID device, int button) {
//             this(device.getPort(), button);

//             // System.out.println("Buttons: " + _bc);
//             // System.out.println("POV's: " + _pc);
//             // System.out.println(use_pov);
//         }

//         @Override
//         public boolean get() {
//             if (use_pov) {
//                 return (source.getStickPOV(this.device, (button-1) / 4) / 90.0 + 1) == this.button;
//             } else {
//                 return source.getStickButton(this.device, button);
//             }
//         }
//     }

//     private static class XboxType implements Constants.Controls {
//         private static enum Digital {
//             LB(5), RB(6), LS(9), RS(10),
//             A(1), B(2), X(3), Y(4),
//             BACK(7), START(8),
//             DT(11), DR(12), DB(13), DL(14);     // Dpad buttons (only valid with XButton objects)
        
//             public final int value;
//             private Digital(int value) {
//                 this.value = value;
//             }
//         }
//         private static enum Analog {
//             LX(0), RX(4), LY(1), RY(5), LT(2), RT(3);
        
//             public final int value;
//             private Analog(int value) {
//                 this.value = value;
//             }
//         }

//         private final int id;

//         public XboxType(int id) {
//             this.id = id;
//         }

//         public double getAnalogLX() {
//             return source.getStickAxis(this.id, Analog.LX.value);
//         }
//         public double getAnalogLY() {
//             return source.getStickAxis(this.id, Analog.LY.value);
//         }
//         public double getAnalogRX() {
//             return source.getStickAxis(this.id, Analog.RX.value);
//         }
//         public double getAnalogRY() {
//             return source.getStickAxis(this.id, Analog.RY.value);
//         }

//         public XButton getSpeedToggle() {
//             return new XButton(this.id, Digital.A.value);
//         }

//         public Input.XButton getDumpActuate() {
//             return new XButton(this.id, Digital.DT.value);
//         }
//         public Input.XButton getDumpReset() {
//             return new XButton(this.id, Digital.DB.value);
//         }

//         public Input.XButton getEStop() {
//             return new XButton(this.id, Digital.BACK.value);
//         }
//     }
//     private static class DualType implements Constants.Controls {
//         private static enum Digital {
//             TRI(1), TB(2), TT(3), TL(4), TR(5),             // ~ trigger, top-bottom, top-top, top-left, top-right
//             B1(6), B2(7), B3(8), B4(9), B5(10), B6(11);     // ~ buttons on the base of the joystick (labeled)

//             public final int value;
//             private Digital(int value) {
//                 this.value = value;
//             }
//         }
//         private static enum Analog {
//             X(0), Y(1), S(2);   // ~ X-Axis, Y-Axis, Slider thing on the bottom

//             public final int value;
//             private Analog(int value) {
//                 this.value = value;
//             }
//         }

//         private final int l_id, r_id;

//         public DualType(int l_id, int r_id) {
//             this.l_id = l_id;
//             this.r_id = r_id;
//         }

//         public double getAnalogLX() {
//             return source.getStickAxis(this.l_id, Analog.X.value);
//         }
//         public double getAnalogLY() {
//             return source.getStickAxis(this.l_id, Analog.Y.value);
//         }
//         public double getAnalogRX() {
//             return source.getStickAxis(this.r_id, Analog.X.value);
//         }
//         public double getAnalogRY() {
//             return source.getStickAxis(this.r_id, Analog.Y.value);
//         }

//         public Input.XButton getSpeedToggle() {
//             return new XButton(this.r_id, Digital.TRI.value);
//         }

//         public Input.XButton getDumpActuate() {
//             return new XButton(this.r_id, Digital.TT.value);
//         }
//         public Input.XButton getDumpReset() {
//             return new XButton(this.r_id, Digital.TB.value);
//         }

//         public Input.XButton getEStop() {
//             return new XButton(this.l_id, Digital.TRI.value);
//         }
//     }
//     // private static class ErrorType implements Constants.Controls {
//     //     public double getAnalogLX() {
//     //         return 0;
//     //     }
//     //     public double getAnalogLY() {
//     //         return 0;
//     //     }
//     //     public double getAnalogRX() {
//     //         return 0;
//     //     }
//     //     public double getAnalogRY() {
//     //         return 0;
//     //     }

//     //     public Input.XButton getSpeedToggle() {
//     //         return null;
//     //     }

//     //     public Input.XButton getDumpActuate() {
//     //         return null;
//     //     }
//     //     public Input.XButton getDumpReset() {
//     //         return null;
//     //     }

//     //     public Input.XButton getEStop() {
//     //         return null;
//     //     }
//     // }

//     private boolean checkValidJoysticks(String name) {
//         String[] valid = {"Attack3"};                   // MAKE SURE THESE NAMEs ARE CORRECT AND THE COMPARISSON ACTUALLY WORKS
//         for (int i = 0; i < valid.length; i++) {
//             if (name.equals(valid[i])) {
//                 return true;
//             }
//         }
//         return false;
//     }

//     public Input() {    // automatically search for patterns to determine input type
//         int detected = 0;
//         int[] jsticks = {-1, -1};
//         for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
//             if (source.isJoystickConnected(i)) {
//                 detected += 1;
//                 if (source.getJoystickIsXbox(i)) {      // prioritze this because detection methods are most concrete
//                     this.controller = new XboxType(i);
//                     System.out.println("Xbox controller detected and initialized on port " + i); 
//                     return;                   
//                 }
//                 else if (checkValidJoysticks(source.getJoystickName(i))) {
//                     jsticks[1] = jsticks[0];
//                     jsticks[0] = i;
//                     if((jsticks[0] >= 0) && (jsticks[1] >= 0)) {
//                         this.controller = new DualType(jsticks[0], jsticks[1]);
//                         System.out.println("Joysticks on ports " + jsticks[0] + " and " + jsticks[1] + " detected and intialized");
//                         return;
//                     }
//                 }
//             }
//         }
//         this.controller = null;
//         if (detected > 0) {
//             System.out.println(detected + " devices detected but none match a template type - add types or manual port assignments");
//         }
//         else {
//             System.out.println("No devices detected - plug in devices and restart the program");
//         }
//         // pause the program safely
//     }
//     public Input(int p) {
//         this.controller = new XboxType(p);
//     }
//     public Input(int p_left, int p_right) {
//         this.controller = new DualType(p_left, p_right);
//     }
//     // 3-Joystick control board?

//     public double getAnalogLX() {
//         return this.controller.getAnalogLX();
//     }
//     public double getAnalogLY() {
//         return this.controller.getAnalogLY();
//     }
//     public double getAnalogRX() {
//         return this.controller.getAnalogRX();
//     }
//     public double getAnalogRY() {
//         return this.controller.getAnalogRY();
//     }

//     public Input.XButton getSpeedToggle() {
//         return this.controller.getSpeedToggle();
//     }

//     public Input.XButton getDumpActuate() {
//         return this.controller.getDumpActuate();
//     }
//     public Input.XButton getDumpReset() {
//         return this.controller.getDumpReset();
//     }

//     public Input.XButton getEStop() {
//         return this.controller.getEStop();
//     }
// }