#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// Helper function to write to the I2C bus directly, replacing system("i2cset ...")
static int axp2101_write_reg(unsigned char reg, unsigned char val)
{
    int fd;
    const char *device = "/dev/i2c-0";
    int addr = 0x34; // AXP2101 I2C address

    if ((fd = open(device, O_RDWR)) < 0)
    {
        perror("Failed to open /dev/i2c-0");
        return -1;
    }

    // I2C_SLAVE_FORCE is the equivalent of the '-f' flag in i2cset.
    // It forces access even if a kernel driver is currently bound to the AXP2101.
    if (ioctl(fd, I2C_SLAVE_FORCE, addr) < 0)
    {
        perror("Failed to acquire bus access");
        close(fd);
        return -1;
    }

    unsigned char buffer[2];
    buffer[0] = reg;
    buffer[1] = val;

    if (write(fd, buffer, 2) != 2)
    {
        perror("Failed to write to the i2c bus");
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

void set_charge_limit_voltage(int value)
{
    int reg_val;

    /*
     * AXP2101 REG 64H (CV charger voltage setting)
     * 0x01 = 4.00V
     * 0x02 = 4.10V
     * 0x03 = 4.20V
     * 0x04 = 4.35V
     * 0x05 = 4.40V (OF)
     */
    switch (value)
    {
    case 1:
        reg_val = 0x05; // 4.40V
        break;
    case 2:
        reg_val = 0x04; // 4.35V
        break;
    case 3:
        reg_val = 0x03; // 4.20V
        break;
    case 4:
        reg_val = 0x02; // 4.10V
        break;
    case 5:
        reg_val = 0x01; // 4.00V
        break;
    default:
        reg_val = 0;
        break;
    }
    if (reg_val == 0)
        return;

    axp2101_write_reg(0x64, reg_val);
}

void set_charge_current(int value)
{
    int reg_val = 0;
    
    /*
     * AXP2101 REG 62H (ICC charger setting)
     * N <= 8: 25 * N mA | N > 8: 200 + 100 * (N - 8) mA
     * 
     * MAPPED VALUES:
     * 0  (0x00) = 0mA        11 (0x0B) = 500mA
     * 1  (0x01) = 25mA       12 (0x0C) = 600mA
     * 2  (0x02) = 50mA       13 (0x0D) = 700mA (OF)
     * 3  (0x03) = 75mA       14 (0x0E) = 800mA
     * 4  (0x04) = 100mA      15 (0x0F) = 900mA
     * 5  (0x05) = 125mA      16 (0x10) = 1000mA
     * 6  (0x06) = 150mA      17 (0x11) = 1100mA
     * 7  (0x07) = 175mA      18 (0x12) = 1200mA
     * 8  (0x08) = 200mA      19 (0x13) = 1300mA
     * 9  (0x09) = 300mA      20 (0x14) = 1400mA
     * 10 (0x0A) = 400mA      21 (0x15) = 1500mA (Max)
     */
    int values[] = {0, 25, 50, 75, 100, 125, 150, 175, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500};

    for (int i = 0; i < sizeof(values) / sizeof(values[0]); i++)
    {
        if (value == values[i])
        {
            reg_val = i;
            break;
        }
    }
    
    // Validate bounds. Forbid 0 (0mA) for now. 
    if (reg_val == 0)
        return;

    axp2101_write_reg(0x62, reg_val);
}