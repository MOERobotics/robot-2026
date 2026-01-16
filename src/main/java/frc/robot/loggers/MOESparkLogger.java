package frc.robot.loggers;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Current;
import lombok.Builder;
import lombok.Value;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.*;

public class MOESparkLogger implements LoggableInputs {
    private final SparkMax spark;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkLimitSwitch forwardLimit, reverseLimit;
    private final RelativeEncoder relativeEncoder;
    private final @Value @Builder static class ConfigCache {
        public boolean
            forwardLimitEnabled,
            reverseLimitEnabled,
            motorInverted,
            absoluteEncoderInverted,
            relativeEncoderInverted,
            followerModeInverted;
        public IdleMode idleMode;
        public int
            deviceID,
            followerLeaderID;
        public double
            motorRampRate,
            relativeEncoderCountPerRev,
            absoluteEncoderZeroOffset;
        public Current currentLimit;
        public String
            motorType,
            serialNumber,
            firmware,
            forwardLimitType,
            reverseLimitType;
    };
    private final ConfigCache config;

    public MOESparkLogger(SparkMax spark) {
        this.spark = spark;
        this.absoluteEncoder = spark.getAbsoluteEncoder();
        this.forwardLimit = spark.getForwardLimitSwitch();
        this.reverseLimit = spark.getReverseLimitSwitch();
        this.relativeEncoder = spark.getEncoder();


        //region Config cache
        var _config = spark.configAccessor;
        this.config = ConfigCache
            .builder()
            .deviceID(spark.getDeviceId())
            .serialNumber(sprintByteArray(spark.getSerialNumber()))
            .firmware(spark.getFirmwareString())
            .idleMode(_config.getIdleMode())
            .motorType(spark.getMotorType().name())
            .currentLimit(Amps.of(_config.getSmartCurrentLimit()))
            .motorInverted(_config.getInverted())
            .motorRampRate(_config.getOpenLoopRampRate())

            .forwardLimitEnabled(_config.limitSwitch.getForwardLimitSwitchEnabled())
            .forwardLimitType(_config.limitSwitch.getForwardSwitchType().name())
            .reverseLimitEnabled(_config.limitSwitch.getReverseLimitSwitchEnabled())
            .reverseLimitType(_config.limitSwitch.getReverseSwitchType().name())

            .relativeEncoderInverted(_config.encoder.getInverted())
            .relativeEncoderCountPerRev(_config.encoder.getCountsPerRevolution())
            .absoluteEncoderInverted(_config.absoluteEncoder.getInverted())
            .absoluteEncoderZeroOffset(_config.absoluteEncoder.getZeroOffset())
            .followerLeaderID(_config.getFollowerModeLeaderId())
            .followerModeInverted(_config.getFollowerModeInverted())
            .build();

        //endregion
    }

    private static String sprintByteArray(byte[] buffer) {
        StringBuilder out = new StringBuilder(buffer.length * 2);
        for (byte b : buffer) out.append(String.format("%02X", b));
        return out.toString();
    }

    @Override
    public void toLog(LogTable logTable) {

        logTable.put("deviceID", config.deviceID);
        logTable.put("type", config.motorType);
        logTable.put("heartbeat", System.currentTimeMillis());

        logTable.put("status/setpoint", spark.get());
        logTable.put("status/output%", spark.getAppliedOutput());
        logTable.put("status/outVoltage", Volts.of(spark.getBusVoltage() * spark.getAppliedOutput()));
        logTable.put("status/busVoltage", Volts.of(spark.getBusVoltage()));
        logTable.put("status/temperature", Celsius.of(spark.getMotorTemperature()));
        logTable.put("status/current", Amps.of(spark.getOutputCurrent()));

        logTable.put("forwardLimit/isPressed", forwardLimit.isPressed());
        logTable.put("forwardLimit/isEnabled", config.forwardLimitEnabled);
        logTable.put("forwardLimit/type", config.forwardLimitType);

        logTable.put("reverseLimit/isPressed", reverseLimit.isPressed());
        logTable.put("reverseLimit/isEnabled", config.reverseLimitEnabled);
        logTable.put("reverseLimit/type", config.reverseLimitType);

        logTable.put("absoluteEncoder/position", absoluteEncoder.getPosition());
        logTable.put("absoluteEncoder/velocity", absoluteEncoder.getVelocity());
        logTable.put("absoluteEncoder/inverted", config.absoluteEncoderInverted);
        logTable.put("absoluteEncoder/offset", config.absoluteEncoderZeroOffset);

        logTable.put("relativeEncoder/position", relativeEncoder.getPosition());
        logTable.put("relativeEncoder/velocity", relativeEncoder.getVelocity());
        logTable.put("relativeEncoder/inverted", config.relativeEncoderInverted);
        logTable.put("relativeEncoder/countPerRev", config.relativeEncoderCountPerRev);

        logTable.put("config/serial", config.serialNumber);
        logTable.put("config/firmware", config.firmware);
        logTable.put("config/idleMode", config.idleMode);
        logTable.put("config/inverted", config.motorInverted);
        logTable.put("config/leader", config.followerLeaderID);
        logTable.put("config/leaderInverted", config.followerModeInverted);
        logTable.put("config/rampRate", config.motorRampRate);
        logTable.put("config/currentLimit", config.currentLimit);
    }

    @Override
    public void fromLog(LogTable logTable) {
        //lmao
    }
}
