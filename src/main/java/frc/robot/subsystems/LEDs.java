package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class LEDs extends SubsystemBase {

    CANdle leds;
    private final int LedCount = 120;

    public enum LEDStates {
        Default,
        Intaking,
        L1,
        L2,
        L3,
        L4,
        NotInScoringPosition,
        InScoringPosition,
        DisabledRed,
        DisabledBlue,
        BallIntook
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
            case L1:
                leds.animate(new FireAnimation(1, 0.7, LedCount / 4, 0.7, 0.7));
                break;
            case L2:
                leds.animate(new FireAnimation(1, 0.7, LedCount / 2, 0.7, 0.7));
                break;
            case L3:
                leds.animate(new FireAnimation(1, 0.7, (LedCount * 3) / 2, 0.7, 0.7));
                break;
            case L4:
                leds.animate(new FireAnimation(1, 0.7, LedCount, 0.7, 0.7));
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
            case BallIntook:
                leds.animate(new StrobeAnimation(255, 255, 0));
                break;
            default:
                break;
        }
    }

}
