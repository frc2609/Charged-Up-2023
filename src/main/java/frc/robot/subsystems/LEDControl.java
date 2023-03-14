package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDControl extends SubsystemBase{
// led controller pretends to be a PWM motor controller
    private final Spark LEDController = new Spark(Constants.LED.PWM_PORT);

    public void setViolet() {
        LEDController.set(Constants.LED.VIOLET);
    }

    public void setGreen() {
        LEDController.set(Constants.LED.GREEN);
    }

    public void setRed() {
        LEDController.set(Constants.LED.RED);
    }

    public void setYellow() {
        LEDController.set(Constants.LED.YELLOW);
    }

    public void setBlue() {
        LEDController.set(Constants.LED.BLUE);
    }
}
