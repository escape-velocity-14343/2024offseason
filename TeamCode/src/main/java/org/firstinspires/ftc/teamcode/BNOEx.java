package org.firstinspires.ftc.teamcode;

import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@I2cDeviceType
@DeviceProperties(name = "FastBNOEx", xmlTag = "fastbnoex")
public class BNOEx extends LynxEmbeddedBNO055IMUNew implements IMU {

    private static final int EUL_DATA_X_LSB = 0x1A;

    /**
     * This constructor is called internally by the FTC SDK.
     *
     * @param deviceClient
     * @param deviceClientIsOwned
     */
    public BNOEx(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
    }

    @Override
    protected boolean internalInitialize(@NonNull IMU.Parameters genericParameters) {
        RobotLog.ii("FastBNO", "Default i2caddr: " + deviceClient.getI2cAddress().get7Bit());
        return super.internalInitialize(genericParameters);
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {

        TimestampedData ts = deviceClient.readTimeStamped(EUL_DATA_X_LSB, 2);

        int lower = ts.data[0];
        int upper = ts.data[1];
        int sign = 1;
        if ((upper & 0b10000000) == 0b10000000) {
            sign = -1;
            upper -= 0b10000000;
        }
        double yaw = (double) (lower + upper*256) * sign/16;
        return new YawPitchRollAngles(AngleUnit.DEGREES, yaw, 0, 0, ts.nanoTime);
    }
}
