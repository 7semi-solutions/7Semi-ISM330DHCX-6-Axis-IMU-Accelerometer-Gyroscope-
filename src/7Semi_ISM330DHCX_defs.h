#pragma once

/**
 * ISM330DHCX_WHO_AM_I / ISM330DHCX_ID
 * - WHO_AM_I register is used to verify the sensor is responding.
 * - ISM330DHCX_ID is the expected value returned by WHO_AM_I for this device.
 */
#define ISM330DHCX_WHO_AM_I     0x0F  // Device ID register
#define ISM330DHCX_ID           0x6B  // Expected WHO_AM_I value

/**
 * Main control registers
 * - CTRL1_XL: accel ODR/FS/filter enable bits
 * - CTRL2_G : gyro ODR/FS bits
 * - CTRL3_C : common control (BDU, reset, auto-increment, interface config)
 */
#define ISM330DHCX_CTRL1_XL     0x10
#define ISM330DHCX_CTRL2_G      0x11
#define ISM330DHCX_CTRL3_C      0x12

/**
 * Additional control registers
 * - These control bandwidth, power modes, high-pass filters, axis enables, etc.
 */
#define CTRL4_C                 0x13
#define CTRL5_C                 0x14
#define CTRL6_C                 0x15
#define CTRL7_G                 0x16
#define CTRL8_XL                0x17
#define CTRL9_XL                0x18
#define CTRL10_C                0x19

/**
 * FUNC_CFG_ACCESS / PIN_CTRL
 * - FUNC_CFG_ACCESS gates access to embedded function + sensor hub register banks.
 * - PIN_CTRL configures interrupt pin electrical behavior and pullups.
 */
#define FUNC_CFG_ACCESS         0x01
#define PIN_CTRL                0x02

/**
 * FIFO_CTRLx
 * - FIFO_CTRL1/2: watermark threshold (LSB/MSB)
 * - FIFO_CTRL3  : decimation settings for FIFO batching
 * - FIFO_CTRL4  : FIFO mode selection and enable bits
 *
 * COUNTER_BDR_REGx
 * - Batch data rate counter and threshold configuration.
 */
#define FIFO_CTRL1              0x07
#define FIFO_CTRL2              0x08
#define FIFO_CTRL3              0x09
#define FIFO_CTRL4              0x0A
#define COUNTER_BDR_REG1        0x0B
#define COUNTER_BDR_REG2        0x0C

/**
 * INT1_CTRL / INT2_CTRL
 * - Route data-ready, FIFO events, counter events, etc. to the INT pins.
 */
#define INT1_CTRL               0x0D
#define INT2_CTRL               0x0E

/**
 * Aliases (single source of truth)
 * - Keep exactly one definition per register to avoid silent conflicts.
 * - These aliases let the .cpp use short names without duplicating addresses.
 */
#define WHO_AM_I                ISM330DHCX_WHO_AM_I
#define CTRL1_XL                ISM330DHCX_CTRL1_XL
#define CTRL2_G                 ISM330DHCX_CTRL2_G
#define CTRL3_C                 ISM330DHCX_CTRL3_C

/**
 * Status / interrupt source registers
 * - Used to identify which event fired (tap, wake-up, 6D, etc.).
 */
#define ALL_INT_SRC             0x1A
#define WAKE_UP_SRC             0x1B
#define TAP_SRC                 0x1C
#define DRD_SRC                 0x1D
#define STATUS_REG              0x1E

/**
 * Temperature output registers
 * - OUT_TEMP_L/H contain raw temperature.
 * - Conversion is typically: T(°C) = 25 + (raw / 256)
 */
#define OUT_TEMP_L              0x20
#define OUT_TEMP_H              0x21

/**
 * Gyro output registers (little-endian per axis)
 * - Read OUTX_L_G..OUTZ_H_G as a 6-byte burst for coherent XYZ.
 */
#define OUTX_L_G                0x22
#define OUTX_H_G                0x23
#define OUTY_L_G                0x24
#define OUTY_H_G                0x25
#define OUTZ_L_G                0x26
#define OUTZ_H_G                0x27

/**
 * Accel output registers (little-endian per axis)
 * - Read OUTX_L_A..OUTZ_H_A as a 6-byte burst for coherent XYZ.
 */
#define OUTX_L_A                0x28
#define OUTX_H_A                0x29
#define OUTY_L_A                0x2A
#define OUTY_H_A                0x2B
#define OUTZ_L_A                0x2C
#define OUTZ_H_A                0x2D


/**
 * Embedded function status (main page)
 * - FSM/MLC status registers live on the main page for quick polling.
 */
#define EMB_FUNC_STATUS_MAINPAGE 0x35
#define FSM_STATUS_A_MAINPAGE    0x36
#define FSM_STATUS_B_MAINPAGE    0x37
#define MLC_STATUS_MAINPAGE      0x38
#define STATUS_MASTER_MAINPAGE   0x39


/**
 * FIFO_STATUS1/2
 * - FIFO_STATUS1: unread level LSB
 * - FIFO_STATUS2: unread level MSB + flags (overrun, watermark, etc.)
 *
 * FIFO_DATA_OUT_TAG + XYZ
 * - Tagged FIFO output frames are 7 bytes:
 *   - byte0 (0x78): TAG (tag is in bits [7:3])
 *   - byte1..6 (0x79..0x7E): X/Y/Z int16 little-endian payload
 *
 * IMPORTANT:
 * - Even if IF_INC is enabled, read FIFO in 7-byte chunks starting at 0x78.
 * - A long burst read from 0x78 can auto-increment past 0x7E and corrupt parsing.
 */
#define FIFO_STATUS1            0x3A
#define FIFO_STATUS2            0x3B

#define FIFO_DATA_OUT_TAG       0x78
#define FIFO_DATA_OUT_X_L       0x79
#define FIFO_DATA_OUT_X_H       0x7A
#define FIFO_DATA_OUT_Y_L       0x7B
#define FIFO_DATA_OUT_Y_H       0x7C
#define FIFO_DATA_OUT_Z_L       0x7D
#define FIFO_DATA_OUT_Z_H       0x7E


/**
 * Timestamp registers
 * - 32-bit timestamp split across TIMESTAMP0..3 (LSB..MSB).
 * - Timestamp must be enabled via CTRL10_C TIMESTAMP_EN (bit depends on device).
 */
#define TIMESTAMP0              0x40
#define TIMESTAMP1              0x41
#define TIMESTAMP2              0x42
#define TIMESTAMP3              0x43

/**
 * Tap / wake-up / free-fall configuration
 * - Used for embedded detection features (tap, wake-up, 6D, etc.).
 * - Requires proper routing via INTx_CTRL and configuration registers.
 */
#define TAP_CFG0                0x56
#define TAP_CFG1                0x57
#define TAP_CFG2                0x58
#define TAP_THS_6D              0x59
#define INT_DUR2                0x5A
#define WAKE_UP_THS             0x5B
#define WAKE_UP_DUR             0x5C
#define FREE_FALL               0x5D

/**
 * OIS + user offset registers
 * - OIS block has its own CTRLx_OIS controls.
 * - X/Y/Z_OFS_USR are user offsets for accel output correction.
 */
#define INTERNAL_FREQ_FINE      0x63
#define INT_OIS                 0x6F
#define CTRL1_OIS               0x70
#define CTRL2_OIS               0x71
#define CTRL3_OIS               0x72
#define X_OFS_USR               0x73
#define Y_OFS_USR               0x74
#define Z_OFS_USR               0x75

/**
 * CTRL1_XL fields (0x10)
 * - XL_ODR_MASK: ODR field mask (bits [7:4])
 * - XL_FS_MASK : FS field mask (bits [3:2])
 * - XL_LPF2_EN : enables LPF2 path (bit1)
 */
#define XL_ODR_MASK        0xF0
#define XL_FS_MASK         0x0C
#define XL_LPF2_EN         (1 << 1)

/**
 * CTRL2_G fields (0x11)
 * - G_ODR_MASK: ODR field mask (bits [7:4])
 * - G_FS_MASK : FS field mask (bits [3:2])
 *
 * Note:
 * - 125 dps and 4000 dps are special encodings on ISM330 family.
 * - Keep those encodings in one place (preferably your enum + setter logic).
 */
#define G_ODR_MASK         0xF0
#define G_FS_MASK          0x0C

/**
 * CTRL3_C bits (0x12)
 * - BOOT     : reboot memory content
 * - BDU      : block data update
 * - H_LACTIVE: interrupt polarity (active-low when set)
 * - PP_OD    : push-pull vs open-drain
 * - SIM      : SPI 3-wire enable
 * - IF_INC   : address auto-increment
 * - SW_RESET : software reset
 */
#define CTRL3_BOOT         (1 << 7)
#define CTRL3_BDU          (1 << 6)
#define CTRL3_H_LACTIVE    (1 << 5)
#define CTRL3_PP_OD        (1 << 4)
#define CTRL3_SIM          (1 << 3)
#define CTRL3_IF_INC       (1 << 2)
#define CTRL3_SW_RESET     (1 << 0)

/**
 * CTRL4_C bits (0x13)
 * - SLEEP_G      : gyro sleep enable
 * - INT2_ON_INT1 : route INT2 signals internally to INT1
 * - DRDY_MASK    : masks DRDY signals until read
 * - I2C_DISABLE  : disable I2C interface (SPI only)
 * - LPF1_SEL_G   : selects gyro LPF1 path
 */
#define CTRL4_SLEEP_G      (1 << 6)
#define CTRL4_INT2_ON_INT1 (1 << 5)
#define CTRL4_DRDY_MASK    (1 << 4)
#define CTRL4_I2C_DISABLE  (1 << 3)
#define CTRL4_LPF1_SEL_G   (1 << 1)

/**
 * CTRL5_C fields (0x14)
 * - ROUNDING_MASK: output rounding field
 * - ST_G_MASK    : gyro self-test field
 * - ST_XL_MASK   : accel self-test field
 */
#define CTRL5_ROUNDING_MASK  0x60
#define CTRL5_ST_G_MASK      0x18
#define CTRL5_ST_XL_MASK     0x06

/**
 * CTRL6_C bits/fields (0x15)
 * - TRIG_EN/LVL1/LVL2: trigger configuration bits
 * - XL_HM_MODE       : accel high-performance mode control
 * - USR_OFF_W        : user offset weight selection
 * - FTYPE_MASK       : gyro LPF1 bandwidth selection bits (FTYPE)
 */
#define CTRL6_TRIG_EN      (1 << 7)
#define CTRL6_LVL1_EN      (1 << 6)
#define CTRL6_LVL2_EN      (1 << 5)
#define CTRL6_XL_HM_MODE   (1 << 4)
#define CTRL6_USR_OFF_W    (1 << 3)
#define CTRL6_FTYPE_MASK   0x07

/**
 * CTRL7_G bits/fields (0x16)
 * - G_HM_MODE       : gyro high-performance mode control
 * - HP_EN_G         : gyro high-pass enable
 * - HPM_G_MASK      : gyro HPF cutoff field
 * - OIS_ON_EN/OIS_ON: OIS controls (if used)
 * - USR_OFF_ON_OUT  : accel user offset output enable (naming per your .cpp)
 */
#define CTRL7_G_HM_MODE      (1 << 7)
#define CTRL7_HP_EN_G        (1 << 6)
#define CTRL7_HPM_G_MASK     0x30
#define CTRL7_OIS_ON_EN      (1 << 3)
#define CTRL7_USR_OFF_ON_OUT (1 << 2)
#define CTRL7_OIS_ON         (1 << 1)

/**
 * CTRL8_XL bits/fields (0x17)
 * - HPCF_MASK        : cutoff selection field
 * - HP_REF_MODE      : high-pass reference mode
 * - FAST_SETTL       : fast settling enable
 * - HP_SLOPE_EN      : high-pass/slope enable
 * - LOW_PASS_ON_6D   : apply LPF on 6D orientation detection path
 */
#define CTRL8_HPCF_MASK      0xE0
#define CTRL8_HP_REF_MODE    (1 << 4)
#define CTRL8_FAST_SETTL     (1 << 3)
#define CTRL8_HP_SLOPE_EN    (1 << 2)
#define CTRL8_LOW_PASS_ON_6D (1 << 1)

/**
 * COUNTER_BDR_REG1 bits
 * - DATAREADY_PULSED : DRDY pulsed behavior
 * - RST_COUNTER_BDR  : reset counter
 * - TRIG_COUNTER_BDR : select trigger source
 *
 * Notes:
 * - CNT_BDR_TH_x bit mapping can vary by device/revision.
 * - Only keep these if your .cpp actually uses them.
 */
#define DATAREADY_PULSED  (1 << 7)
#define RST_COUNTER_BDR   (1 << 6)
#define TRIG_COUNTER_BDR  (1 << 5)

#define CNT_BDR_TH_10     (1 << 4)
#define CNT_BDR_TH_9      (1 << 3)
#define CNT_BDR_TH_8      (1 << 2)

/**
 * INT1_CTRL bits
 * - Enable routing of various events to INT1 pin.
 */
#define INT1_DEN_DRDY  (1 << 7)
#define INT1_CNT_BDR   (1 << 6)
#define INT1_FIFO_FULL (1 << 5)
#define INT1_FIFO_OVR  (1 << 4)
#define INT1_FIFO_TH   (1 << 3)
#define INT1_BOOT      (1 << 2)
#define INT1_DRDY_G    (1 << 1)
#define INT1_DRDY_XL   (1 << 0)

/**
 * INT2_CTRL bits
 * - Enable routing of various events to INT2 pin.
 */
#define INT2_CNT_BDR   (1 << 6)
#define INT2_FIFO_FULL (1 << 5)
#define INT2_FIFO_OVR  (1 << 4)
#define INT2_FIFO_TH   (1 << 3)
#define INT2_DRDY_TEMP (1 << 2)
#define INT2_DRDY_G    (1 << 1)
#define INT2_DRDY_XL   (1 << 0)
