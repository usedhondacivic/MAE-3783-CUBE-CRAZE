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

// Set a pins direction
void setPinDirection(int pin, int dir)
{
    if (pin <= 7)
    {
        dir == 1 ? setBit(&DDRD, pin) : unsetBit(&DDRD, pin); // boooooo ternary ew gross icky
    }
    else if (pin <= 13)
    {
        dir == 1 ? setBit(&DDRB, pin - 8) : unsetBit(&DDRB, pin - 8);
    }
    else if (pin < 18)
    {
        Serial.println("Set dir: Pin not found idiot"); // We don't do analog here
    }
    else if (pin <= 19)
    {
        dir == 1 ? setBit(&DDRC, pin - 23) : unsetBit(&DDRC, pin - 23);
    }
}

void writePin(int pin, int output)
{
    if (pin <= 7)
    {
        output == 1 ? setBit(&PORTD, pin) : unsetBit(&PORTD, pin); // boooooo ternary ew gross icky
    }
    else if (pin <= 13)
    {
        output == 1 ? setBit(&PORTB, pin - 8) : unsetBit(&PORTB, pin - 8);
    }
    else if (pin < 18)
    {
        Serial.println("Write pin: Pin not found idiot"); // We don't do analog here
    }
    else if (pin <= 19)
    {
        output == 1 ? setBit(&PORTC, pin - 23) : unsetBit(&PORTC, pin - 23);
    }
}

int readPin(int pin)
{
    if (pin <= 7)
    {
        return getBit(&PIND, pin);
    }
    else if (pin <= 13)
    {
        return getBit(&PINB, pin - 8);
    }
    else if (pin < 18)
    {
        Serial.println("Read pin: Pin not found idiot"); // We don't do analog here
    }
    else if (pin <= 19)
    {
        return getBit(&PINC, pin - 23);
    }
}

///////////////////////////////////////////////////////////////////
//////////////////// COMMUNICATION PROTOCOLS //////////////////////
///////////////////////////////////////////////////////////////////

class PWMChannel
{
private:
    int pin;
    int dutyCycle = 0;

public:
    PWMChannel() {}

    PWMChannel(int _pin)
    {
        pin = _pin;
        setPinDirection(_pin, OUTPUT);
    }

    void setDutyCycle(double percent)
    {
        if (percent > 0.5)
        {
            writePin(pin, 1);
        }
        else
        {
            writePin(pin, 0);
        }
    }
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
    PWMChannel PWMOne, PWMTwo;
    double speed;

public:
    HBridgeDriver() {}

    HBridgeDriver(int _pinOne, int _pinTwo)
    {
        PWMOne = PWMChannel(_pinOne);
        PWMTwo = PWMChannel(_pinTwo);
    }

    void setSpeed(double speed)
    {
        if (speed > 0)
        {
            PWMOne.setDutyCycle(speed);
            PWMTwo.setDutyCycle(0);
        }
        else
        {
            PWMOne.setDutyCycle(0);
            PWMTwo.setDutyCycle(-speed);
        }
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

HBridgeDriver rightWheel(11, 10);
HBridgeDriver leftWheel(6, 5);

///////////////////////////////////////////////////////////////////
//////////////////////// CONTROL ROUTINES /////////////////////////
///////////////////////////////////////////////////////////////////

void turn_90_left()
{
    leftWheel.setSpeed(-1);
    rightWheel.setSpeed(1);
    _delay_ms(1000);
    leftWheel.setSpeed(0);
    rightWheel.setSpeed(0);
}

void turn_90_right()
{
    leftWheel.setSpeed(1);
    rightWheel.setSpeed(-1);
    _delay_ms(1000);
    leftWheel.setSpeed(0);
    rightWheel.setSpeed(0);
}

void drive_6_inches()
{
    leftWheel.setSpeed(1);
    rightWheel.setSpeed(1);
    _delay_ms(1000);
    leftWheel.setSpeed(0);
    rightWheel.setSpeed(0);
}

///////////////////////////////////////////////////////////////////
/////////////////////// MAIN CONTROL LOOPS ////////////////////////
///////////////////////////////////////////////////////////////////

int main(void)
{
    init(); // Needed for efficient serial

    Serial.begin(115200);
    Serial.println("Hello world\n");

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

    for (;;)
    {
    }

    return 0;
}