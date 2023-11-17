package frc.robot.control;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;


@SuppressWarnings("MethodName")
public class CommandSwitchController extends CommandGenericHID {
    private final SwitchController m_hid;

    public CommandSwitchController(int port) {
        super(port);
        m_hid = new SwitchController(port);
    }

    @Override
    public SwitchController getHID() {
        return m_hid;
    }

    public Trigger leftStick() {
        return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftStick(EventLoop loop) {
        return m_hid.leftStick(loop).castTo(Trigger::new);
    }

    public Trigger rightStick() {
        return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightStick(EventLoop loop) {
        return m_hid.rightStick(loop).castTo(Trigger::new);
    }

    public Trigger a() {
        return a(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger a(EventLoop loop) {
        return m_hid.a(loop).castTo(Trigger::new);
    }

    public Trigger b() {
        return b(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger b(EventLoop loop) {
        return m_hid.b(loop).castTo(Trigger::new);
    }

    public Trigger x() {
        return x(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger x(EventLoop loop) {
        return m_hid.x(loop).castTo(Trigger::new);
    }

    public Trigger y() {
        return y(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger y(EventLoop loop) {
        return m_hid.y(loop).castTo(Trigger::new);
    }

    public Trigger l() {
        return l(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger l(EventLoop loop) {
        return m_hid.l(loop).castTo(Trigger::new);
    }

    public Trigger r() {
        return r(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger r(EventLoop loop) {
        return m_hid.r(loop).castTo(Trigger::new);
    }

    public Trigger zl() {
        return zl(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger zl(EventLoop loop) {
        return m_hid.zl(loop).castTo(Trigger::new);
    }

    public Trigger zr() {
        return zr(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger zr(EventLoop loop) {
        return m_hid.zr(loop).castTo(Trigger::new);
    }

    public Trigger plus() {
        return plus(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger plus(EventLoop loop) {
        return m_hid.plus(loop).castTo(Trigger::new);
    }

    public Trigger minus() {
        return minus(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger minus(EventLoop loop) {
        return m_hid.minus(loop).castTo(Trigger::new);
    }

    public double getLeftX() {
        double value = m_hid.getLeftX();
        if (Math.abs(value) <= Constants.deadZoneDefault) {
            value = 0.0;
        }
        return value;
    }

    public double getRightX() {
        double value = m_hid.getRightX();
        if (Math.abs(value) <= Constants.deadZoneDefault) {
            value = 0.0;
        }
        return value;
    }

    public double getLeftY() {
        double value = m_hid.getLeftY();
        if (Math.abs(value) <= Constants.deadZoneDefault) {
            value = 0.0;
        }
        return value;
    }

    public double getRightY() {
        double value = m_hid.getRightY();
        if (Math.abs(value) <= Constants.deadZoneDefault) {
            value = 0.0;
        }
        return value;
    }
}
