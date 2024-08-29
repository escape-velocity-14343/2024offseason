package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.bosch.BNO055Util.getRawQuaternion;
import static com.qualcomm.hardware.bosch.BNO055Util.write8;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.bosch.BNO055Util;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.I2cWarningManager;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public class FastBNO055 extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, FastBNO055.Parameters> implements IMU {

    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    private static final String TAG = "BNO055IMU (new)";
    private static final BNO055IMU.AngleUnit INTERNAL_ANGLE_UNIT = BNO055IMU.AngleUnit.DEGREES;

    // 4.3.27 and 4.3.28 in the BNO055 datasheet
    private static final int EUL_DATA_X_LSB = 0x1A;
    private static final int EUL_DATA_X_MSB = 0x1B;

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    private final QuaternionBasedImuHelper helper;
    private float yawOffset = 0;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public FastBNO055(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned, new BNO055IMUNew.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        this.deviceClient.setI2cAddress(BNO055IMU.I2CADDR_DEFAULT);

        helper = new QuaternionBasedImuHelper(parameters.imuOrientationOnRobot);

        if (BNO055Util.imuIsPresent(deviceClient, false)) {
            // Reset the yaw to ensure predictable behavior on app launch and Robot Restart, which
            // is when hardware device objects get (re) created. On boot, it's clearer if yaw 0 is
            // set at the time when the rest of the system finishes booting, instead of within the
            // first couple of seconds of power being applied. On Robot Restart, it's _much_ clearer
            // if the yaw gets reset to 0, instead of being set to the current rotation relative to
            // power on.
            if (initialize(parameters)) {
                // We call helper.resetYaw() directly instead of this.resetYaw() so that we can
                // specify a nice long timeout
                helper.resetYaw(TAG, () -> getRawQuaternion(deviceClient), 500);
            }
        }

    }


    /**
     * Actually attempts to carry out initialization with the indicated parameter block.
     * If successful, said parameter block should be stored in the {@link #parameters}
     * member variable.
     *
     * @param genericParameters the parameter block with which to initialize
     * @return whether initialization was successful or not
     */
    @Override
    protected boolean internalInitialize(@NonNull FastBNO055.Parameters genericParameters) {
        // This new BNO055 driver does NOT perform a reset, so that we don't wipe out the yaw offset
        // until the user requests that we do so.
        genericParameters = genericParameters.copy();

        FastBNO055.Parameters parameters;
        if (genericParameters instanceof FastBNO055.Parameters) {
            parameters = (FastBNO055.Parameters) genericParameters;
        } else {
            if (!genericParameters.getClass().equals(IMU.Parameters.class)) {
                RobotLog.addGlobalWarningMessage(AppUtil.getDefContext().getString(R.string.parametersForOtherDeviceUsed));
            }
            parameters = new FastBNO055.Parameters(genericParameters.imuOrientationOnRobot);
        }
        this.parameters = parameters;

        helper.setImuOrientationOnRobot(parameters.imuOrientationOnRobot);

        deviceClient.setI2cAddress(parameters.i2cAddr);

        try {
            // Make sure we have the right device
            if (!BNO055Util.imuIsPresent(deviceClient, true)) {
                throw new BNO055Util.InitException("IMU appears to not be present");
            }

            // Get us into config mode, for sure
            BNO055Util.setSensorMode(deviceClient, BNO055IMU.SensorMode.CONFIG);

            BNO055Util.sharedInit(deviceClient, parameters.toOldParameters());
        } catch (BNO055Util.InitException e) {
            RobotLog.ee(TAG, e, "Failed to initialize BNO055 IMU");
            return false;
        }

        // Make sure the status is correct
        BNO055IMU.SystemStatus status = BNO055Util.getSystemStatus(deviceClient, TAG);
        return status == BNO055IMU.SystemStatus.RUNNING_FUSION;
    }

    //----------------------------------------------------------------------------------------------
    // IMU interface
    //----------------------------------------------------------------------------------------------

    @Override public void resetYaw() {
        // After initializing the BNO055 IMU, it can take about 50ms before it starts returning
        // valid data, so we want to leave some wiggle room.
        helper.resetYaw(TAG, () -> getRawQuaternion(deviceClient), 100);
    }

    public double getHeading() {
        int lower = deviceClient.read8(EUL_DATA_X_LSB);
        int upper = deviceClient.read8(EUL_DATA_X_MSB);
        int sign = 1;
        if ((upper & 0b10000000) == 0b10000000) {
            sign = -1;
            upper -= 0b10000000;
        }
        return (double) (lower + upper*256) * sign/16 + yawOffset;
    }

    /*public void resetYaw(float yawOffset) {
        helper.setYawOffsetQuaternion(yawOffset);
        this.yawOffset = yawOffset;
    }*/

    @Override public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return helper.getRobotYawPitchRollAngles(TAG, () -> getRawQuaternion(deviceClient));
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return helper.getRobotOrientation(TAG, () -> getRawQuaternion(deviceClient), reference, order, angleUnit);
    }

    @Override public Quaternion getRobotOrientationAsQuaternion() {
        return helper.getRobotOrientationAsQuaternion(TAG, () -> getRawQuaternion(deviceClient), true);
    }

    @Override public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return helper.getRobotAngularVelocity(
                BNO055Util.getRawAngularVelocity(deviceClient, INTERNAL_ANGLE_UNIT, INTERNAL_ANGLE_UNIT.toAngleUnit()),
                angleUnit);
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return AppUtil.getDefContext().getString(com.qualcomm.robotcore.R.string.lynx_embedded_bno055_imu_name);
    }

    @Override public HardwareDevice.Manufacturer getManufacturer() {
        return HardwareDevice.Manufacturer.Lynx;
    }

    public static class Parameters extends IMU.Parameters {

        /** The I2C address of the BNO055 */
        public I2cAddr i2cAddr = BNO055IMU.I2CADDR_DEFAULT;

        /** Calibration data with which the BNO055 should be initialized.*/
        public BNO055IMU.CalibrationData calibrationData = null;

        /**
         * The path of a file containing calibration data with which the BNO055 should be
         * initialized. If {@link #calibrationData} is not null, that will be used instead.
         */
        public String calibrationDataFile = null;

        /**
         * @param imuOrientationOnRobot The orientation of the IMU relative to the robot. If the IMU
         *                              is in a REV Control or Expansion Hub, create an instance of
         *                              com.qualcomm.hardware.rev.RevHubOrientationOnRobot
         *                              (from the Hardware module).
         */
        public Parameters(ImuOrientationOnRobot imuOrientationOnRobot) {
            super(imuOrientationOnRobot);
        }

        private Parameters(IMU.Parameters genericParameters) {
            super(genericParameters.imuOrientationOnRobot);
        }

        @Override public FastBNO055.Parameters copy() {
            FastBNO055.Parameters copy = new FastBNO055.Parameters(this.imuOrientationOnRobot);
            copy.i2cAddr = this.i2cAddr;
            copy.calibrationData = this.calibrationData;
            copy.calibrationDataFile = this.calibrationDataFile;
            return copy;
        }

        /**
         * @return A {@link BNO055IMU.Parameters} instance that represents the settings needed for
         *         this driver to work correctly, and respects the parameters set in this object by
         *         the user.
         */
        private BNO055IMU.Parameters toOldParameters() {
            BNO055IMU.Parameters result = new BNO055IMU.Parameters();
            result.angleUnit = INTERNAL_ANGLE_UNIT;
            result.calibrationData = this.calibrationData;
            result.calibrationDataFile = this.calibrationDataFile;
            return result;
        }
    }
}
