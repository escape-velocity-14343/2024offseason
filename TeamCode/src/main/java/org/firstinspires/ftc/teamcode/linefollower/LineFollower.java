package org.firstinspires.ftc.teamcode.linefollower;

import android.util.Log;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "Line Follower", xmlTag = "linefollower")

public class LineFollower extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    byte lastBarRawValue = 0;
    boolean invertBits = true;

    public LineFollower(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x3E));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();

    }
    public byte scan() {
        deviceClient.write8(Registry.DATAB.bVal,0x00);
        lastBarRawValue = deviceClient.read8(Registry.DATAA.bVal);
        if(invertBits == true) //Invert the bits if needed
        {
            lastBarRawValue ^= 0xFF;
        }
        return lastBarRawValue;
    }
    public void reset() {
        deviceClient.write8(Registry.RESET.bVal,0x12);
        deviceClient.write8(Registry.RESET.bVal,0x34);
    }
    public void init() {
        deviceClient.write8(Registry.REG_DIR_A.bVal, 0xFF);
        deviceClient.write8(Registry.REG_DIR_B.bVal, 0xFC);
        deviceClient.write8(Registry.DATAB.bVal, 0x01);
    }
    /**
     * @return the offset of the line from the center
     * */
    public double getPosition() {

        int accumulator = 0;
        byte bitsCounted = 0;
        double lastBarPositionValue = 0;
        int i = 0;
        for ( i = 0; i < 8; i++ )
        {
            if ( ((lastBarRawValue >> i) & 0x01) == 1 )
            {
                bitsCounted++;
            }
        }

        //Find the vector value of each positive bit and sum
        for ( i = 7; i > 3; i-- ) //iterate negative side bits
        {
            if ( ((lastBarRawValue >> i) & 0x01) == 1 )
            {
                accumulator += ((-32 * (i - 3)) + 1);
            }
        }
        for ( i = 0; i < 4; i++ ) //iterate positive side bits
        {
            if ( ((lastBarRawValue >> i) & 0x01) == 1 )
            {
                accumulator += ((32 * (4 - i)) - 1);
            }
        }

        if ( bitsCounted > 0 )
        {
            lastBarPositionValue = accumulator / (bitsCounted);
        }
        else
        {
            lastBarPositionValue = 0;
        }
        return lastBarPositionValue;
    }
    public void setInvertBits(boolean invert) {
        invertBits=invert;
    }

    /**
     * @return the offset of the line from the center in inches
     * */
    public double getPositionInches() {
        return getPosition()/72.73;
    }

    @Override
    protected boolean doInitialize() {
        byte[] byes =  deviceClient.read(Registry.INTERRUPT_MASK_A.bVal);
        Log.println(Log.ASSERT,"line follower","initializing: " + byes);
        reset();
        init();
        return byes[0] == 0xFF00;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Line Follower";
    }
    enum Registry {
        DATAB(0x10),
        DATAA(0x11),
        RESET(0x7D),
        REG_DIR_A(0x0F),
        REG_DIR_B(0x0E),
        INTERRUPT_MASK_A(0x13)
        ;
        public int bVal;
        Registry(int bVal) {
            this.bVal = bVal;
        }
    }

}



