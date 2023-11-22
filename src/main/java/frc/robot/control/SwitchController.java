package frc.robot.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class SwitchController extends GenericHID {
    public enum Button {
        kA(3),
        kB(2),
        kX(4),
        kY(1),
        kL(5),
        kR(6),
        kZL(7),
        kZR(8),
        kMinus(9),
        kPlus(10),
        kLeftStick(11),
        kRightStick(12),
        kHome(13),
        kScreenshot(14);

        public final int value;

        Button(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            return name + "Button";
        }
    }

    public enum Axis {
        kLeftX(0),
        kRightX(2),
        kLeftY(1),
        kRightY(3);

        public final int value;

        Axis(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            return name;
        }
    }

    public SwitchController(final int port) {
        super(port);
    }

    public double getLeftX() {
        return getRawAxis(Axis.kLeftX.value);
    }

    public double getRightX() {
        return getRawAxis(Axis.kRightX.value);
    }

    public double getLeftY() {
        return getRawAxis(Axis.kLeftY.value);
    }

    public double getRightY() {
        return getRawAxis(Axis.kRightY.value);
    }

    public boolean getL() {
        return getRawButton(Button.kL.value);
    }

    public boolean getR() {
        return getRawButton(Button.kR.value);
    }

    public boolean getLPressed() {
        return getRawButtonPressed(Button.kL.value);
    }

    public boolean getRPressed() {
        return getRawButtonPressed(Button.kR.value);
    }

    public boolean getLReleased() {
        return getRawButtonReleased(Button.kL.value);
    }

    public boolean getRReleased() {
        return getRawButtonReleased(Button.kR.value);
    }

    public BooleanEvent l(EventLoop loop) {
        return new BooleanEvent(loop, this::getL);
    }

    public BooleanEvent r(EventLoop loop) {
        return new BooleanEvent(loop, this::getR);
    }

    public boolean getLeftStickButton() {
        return getRawButton(Button.kLeftStick.value);
    }

    public boolean getRightStickButton() {
        return getRawButton(Button.kRightStick.value);
    }

    public boolean getLeftStickButtonPressed() {
        return getRawButtonPressed(Button.kLeftStick.value);
    }

    public boolean getRightStickButtonPressed() {
        return getRawButtonPressed(Button.kRightStick.value);
    }

    public boolean getLeftStickButtonReleased() {
        return getRawButtonReleased(Button.kLeftStick.value);
    }

    public boolean getRightStickButtonReleased() {
        return getRawButtonReleased(Button.kRightStick.value);
    }

    public BooleanEvent leftStick(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftStickButton);
    }

    public BooleanEvent rightStick(EventLoop loop) {
        return new BooleanEvent(loop, this::getRightStickButton);
    }

    public boolean getAButton() {
        return getRawButton(Button.kA.value);
    }

    public boolean getAButtonPressed() {
        return getRawButtonPressed(Button.kA.value);
    }

    public boolean getAButtonReleased() {
        return getRawButtonReleased(Button.kA.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent a(EventLoop loop) {
        return new BooleanEvent(loop, this::getAButton);
    }

    public boolean getBButton() {
        return getRawButton(Button.kB.value);
    }

    public boolean getBButtonPressed() {
        return getRawButtonPressed(Button.kB.value);
    }

    public boolean getBButtonReleased() {
        return getRawButtonReleased(Button.kB.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent b(EventLoop loop) {
        return new BooleanEvent(loop, this::getBButton);
    }

    public boolean getXButton() {
        return getRawButton(Button.kX.value);
    }

    public boolean getXButtonPressed() {
        return getRawButtonPressed(Button.kX.value);
    }

    public boolean getXButtonReleased() {
        return getRawButtonReleased(Button.kX.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent x(EventLoop loop) {
        return new BooleanEvent(loop, this::getXButton);
    }

    public boolean getYButton() {
        return getRawButton(Button.kY.value);
    }

    public boolean getYButtonPressed() {
        return getRawButtonPressed(Button.kY.value);
    }

    public boolean getYButtonReleased() {
        return getRawButtonReleased(Button.kY.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent y(EventLoop loop) {
        return new BooleanEvent(loop, this::getYButton);
    }

    public boolean getZLButton() {
        return getRawButton(Button.kZL.value);
    }

    public boolean getZLButtonPressed() {
        return getRawButtonPressed(Button.kZL.value);
    }

    public boolean getZLButtonReleased() {
        return getRawButtonReleased(Button.kZL.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent zl(EventLoop loop) {
        return new BooleanEvent(loop, this::getZLButton);
    }

    public boolean getZRButton() {
        return getRawButton(Button.kZR.value);
    }

    public boolean getZRButtonPressed() {
        return getRawButtonPressed(Button.kZR.value);
    }

    public boolean getZRButtonReleased() {
        return getRawButtonReleased(Button.kZR.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent zr(EventLoop loop) {
        return new BooleanEvent(loop, this::getZRButton);
    }

    public boolean getMinusButton() {
        return getRawButton(Button.kMinus.value);
    }

    public boolean getMinusButtonPressed() {
        return getRawButtonPressed(Button.kMinus.value);
    }

    public boolean getMinusButtonReleased() {
        return getRawButtonReleased(Button.kMinus.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent minus(EventLoop loop) {
        return new BooleanEvent(loop, this::getMinusButton);
    }

    public boolean getPlusButton() {
        return getRawButton(Button.kPlus.value);
    }

    public boolean getPlusButtonPressed() {
        return getRawButtonPressed(Button.kPlus.value);
    }

    public boolean getPlusButtonReleased() {
        return getRawButtonReleased(Button.kPlus.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent plus(EventLoop loop) {
        return new BooleanEvent(loop, this::getPlusButton);
    }

    public boolean getHomeButton() {
        return getRawButton(Button.kHome.value);
    }

    public boolean getHomeButtonPressed() {
        return getRawButtonPressed(Button.kHome.value);
    }

    public boolean getHomeButtonReleased() {
        return getRawButtonReleased(Button.kHome.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent home(EventLoop loop) {
        return new BooleanEvent(loop, this::getHomeButton);
    }

    public boolean getScreenshotButton() {
        return getRawButton(Button.kScreenshot.value);
    }

    public boolean getScreenshotButtonPressed() {
        return getRawButtonPressed(Button.kScreenshot.value);
    }

    public boolean getScreenshotButtonReleased() {
        return getRawButtonReleased(Button.kScreenshot.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent screenshot(EventLoop loop) {
        return new BooleanEvent(loop, this::getScreenshotButton);
    }
}
