// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import java.net.CacheRequest;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Superstructure.SuperstructureCurrentState;

public class LED extends SubsystemBase {

    // CANdles
    private final CANdle candle1;
    //private final CANdle candle2;
    private Intake intake;

    // LED counts
    private static final int CANDLE1_LED_COUNT = 30;
    private static final int CANDLE2_LED_COUNT = 74;

    // Animations for candle
    private final LarsonAnimation larsonAnimation1 =
            new LarsonAnimation(0, CANDLE1_LED_COUNT);

    private final RgbFadeAnimation rgbFadeAnimation1 =
            new RgbFadeAnimation(0, CANDLE1_LED_COUNT);

    private final RainbowAnimation rainbowAnimation1 =
            new RainbowAnimation(0, CANDLE1_LED_COUNT);

    private final FireAnimation fireAnimation1 =
            new FireAnimation(0, CANDLE1_LED_COUNT);

    private final TwinkleAnimation twinkleAnimation1 =
            new TwinkleAnimation(0, CANDLE1_LED_COUNT);

    private final ColorFlowAnimation colorFlowAnimation1 =
            new ColorFlowAnimation(0, CANDLE1_LED_COUNT);

    public LED(Intake intake) {

        this.intake = intake;

        candle1 = new CANdle(35, CANBus.roboRIO());
        applyConfigs(candle1);


        //candle2 = new CANdle(1, CANBus.roboRIO());
        //applyConfigs(candle2);
    }

    private void applyConfigs(CANdle candle) {
        var cfg = new CANdleConfiguration();

        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;

        cfg.CANdleFeatures.StatusLedWhenActive =
                StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(cfg);

        // Clear animation slots
        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
    }

    private void animate(ControlRequest anim1) {
        candle1.setControl(anim1);
    }

    @Override
    public void periodic() {

        if (!DriverStation.isEnabled()) {

            animate(rgbFadeAnimation1);

        } else {
            Intake.CurrentState intakecurrentState = intake.getIntakeCurrentState();
            SuperstructureCurrentState currentState = Superstructure.getCurrentState();

            if (currentState == SuperstructureCurrentState.SHOOTING 
                || currentState == SuperstructureCurrentState.PRE_SHOOT
                || currentState == SuperstructureCurrentState.REVUP) {

                animate(rainbowAnimation1);

            } else if (intakecurrentState == Intake.CurrentState.INTAKE) {

                animate(fireAnimation1);

            } else {

                animate(colorFlowAnimation1);
            }
        }
    }
}