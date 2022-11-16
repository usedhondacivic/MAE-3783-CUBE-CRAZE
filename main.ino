/*
 * MAE 3783 - CUBE CRAZE
 *
 * Created (Date): 11/16/22
 * Author (Name, netID): Michael Crum (mmc323)
 */

///////////////////////////////////////////////////////////////////
//////////////////////// UTILITY FUNCTIONS ////////////////////////
///////////////////////////////////////////////////////////////////

// Set a bit in reg
void setBit(volatile unsigned char *reg, unsigned int bit)
{
    *reg |= 1 << bit;
}

// Unset a bit in reg
void unsetBit(volatile unsigned char *reg, unsigned int bit)
{
    *reg &= ~(1 << bit);
}

// Get the value of a bit in reg
int getBit(volatile unsigned char *reg, unsigned int bit)
{
    return (*reg >> bit) & 0b1;
}

void setPinDirection(int pin, int dir)
{
}

void writePin(int pin, int output)
{
}

int readPin(int pin)
{
}

///////////////////////////////////////////////////////////////////
//////////////////// COMMUNICATION PROTOCOLS //////////////////////
///////////////////////////////////////////////////////////////////

class PWMChannel
{
private:
public:
};

class I2CDriver
{
private:
    int SDA, SCL;

public:
    I2CDriver(int _SDA, int _SCL)
    {
        SDA = _SDA;
        SCL = _SCL;
    }
};

///////////////////////////////////////////////////////////////////
/////////////////////// PERIPHERAL DRIVERS ////////////////////////
///////////////////////////////////////////////////////////////////

class HBridgeDriver
{
private:
    int PWMOne, PWMTwo;
    double speed;

public:
    HBridgeDriver(int _PWMOne, int _PWMTwo)
    {
        PWMOne = _PWMOne;
        PWMTwo = _PWMTwo;
    }

    void setSpeed(double speed)
    {
    }
};

class ColorSensorDriver
{
private:
    int outputPin, s0, s1, s2, s3;
    int sensorReading;

public:
    ColorSensorDriver(int _out, int _s0, int _s1, int _s2, int _s3)
    {
        outputPin = _out;
        s0 = _s0;
        s1 = _s1;
        s2 = _s2;
        s3 = _s3;
    }

    int getReading()
    {
    }
};

class IMUDriver
{
private:
    int SDA, SCL;

public:
    IMUDriver(int _SDA, int _SCL)
    {
        SDA = _SDA;
        SCL = _SCL;
    }

    int getReading()
    {
    }
};

///////////////////////////////////////////////////////////////////
////////////////////////// ROBOT SETUP ////////////////////////////
///////////////////////////////////////////////////////////////////

HBridgeDriver leftWheel(6, 5);
HBridgeDriver rightWheel(11, 10);

///////////////////////////////////////////////////////////////////
//////////////////////// CONTROL ROUTINES /////////////////////////
///////////////////////////////////////////////////////////////////

void turn_90_left()
{
}

void turn_90_right()
{
}

void drive_6_inches()
{
}

///////////////////////////////////////////////////////////////////
/////////////////////// MAIN CONTROL LOOPS ////////////////////////
///////////////////////////////////////////////////////////////////

int main(void)
{
    drive_6_inches();
    drive_6_inches();
    turn_90_right();
    drive_6_inches();
    drive_6_inches();
    turn_90_left();
    drive_6_inches();
    turn_90_right();
    turn_90_right();
    drive_6_inches();
    drive_6_inches();
    drive_6_inches();
    turn_90_right();
    drive_6_inches();
    drive_6_inches();
}
