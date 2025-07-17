package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LEDs extends SubsystemBase {

    CANdle leds;
    private final int LedCount = 120;

    public enum LEDStates {
        Default,
        Intaking,
        NotInScoringPosition,
        InScoringPosition,
        DisabledRed,
        DisabledBlue,
        Endgame
    }

    public LEDs() {
        leds = new CANdle(Constants.ledsID);
        leds.configLEDType(LEDStripType.GRB);
    }

    public void setLEDS(LEDStates state) {
        switch (state) {
            case Default:
                leds.animate(new StrobeAnimation(0, 255, 255));
                break;
            case Intaking:
                leds.animate(new StrobeAnimation(255, 0, 255, 0, 0.5, LedCount));
                break;
            case NotInScoringPosition:
                leds.animate(new StrobeAnimation(0, 255, 255, 0, 0.8, LedCount));
                break;
            case InScoringPosition:
                leds.animate(new StrobeAnimation(0, 255, 0));
                break;
            case DisabledRed:
                leds.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.7, LedCount));
                break;
            case DisabledBlue:
                leds.animate(new SingleFadeAnimation(0, 0, 255, 0, 0.7, LedCount));
                break;
            case Endgame:
                leds.animate(new RainbowAnimation(1, 1, LedCount));
                break;
            default:
                break;
        }
    }

}
