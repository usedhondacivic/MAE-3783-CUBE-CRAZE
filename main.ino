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
///////////////////////////// ISR's ///////////////////////////////
///////////////////////////////////////////////////////////////////

int half_period = 0; // half_period in clock ticks

ISR(PCINT2_vect)
{
    // Serial.println("hit");
    if (getBit(&PIND, 2) != 0) // Pin is now high -> rising edge
    {
        TCNT1 = 0; // Set timer to 0
    }
    else
    {
        half_period = TCNT1; // Read timer into variable
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
        else if (speed == 0)
        {
            PWMOne.setDutyCycle(0);
            PWMTwo.setDutyCycle(0);
        }
        else
        {
            PWMOne.setDutyCycle(0);
            PWMTwo.setDutyCycle(-speed);
        }
    }
};

enum COLOR
{
    BLUE,
    YELLOW,
    BLACK
};

class ColorSensorDriver
{
private:
    int outputPin = 2;
    int yellowCutoff = 50;
    int blueCutoff = 350;

public:
    ColorSensorDriver()
    {
        unsetBit(&DDRD, outputPin); // Set sensor pin to input

        setBit(&PCICR, 2);          // Enable pin change interrupts for port D
        setBit(&PCMSK2, outputPin); // Enable pin change interrupts for output pin

        sei();

        setBit(&TCCR1B, 0); // Enable timer 1, with no prescaling
        // TCCR1A has correct vals for normal operation by default
    }

    int getReadingRaw()
    {
        setBit(&PCMSK2, outputPin); // Activate interrupts on the sensor pin
        _delay_ms(5);
        unsetBit(&PCMSK2, outputPin); // Disable interrupts on the sensor pin

        return ((double)2 * (double)half_period * (double)0.0625); // 0.0625 microseconds / clock tick
    }

    COLOR getReading()
    {
        int color = getReadingRaw();
        if (color < yellowCutoff)
        {
            return YELLOW;
        }
        else if (color < blueCutoff)
        {
            return BLUE;
        }
        else
        {
            return BLACK;
        }
    }

    COLOR getAverageReading(int delay)
    {
        int one = getReading();
        delayMicroseconds(delay * 1000);
        int two = getReading();
        delayMicroseconds(delay * 1000);
        int three = getReading();
        return (COLOR)round((double)(one + two + three) / 3);
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

class Robot
{
private:
    HBridgeDriver rightWheel;
    HBridgeDriver leftWheel;

public:
    ColorSensorDriver colorSense;

    Robot()
    {
        rightWheel = HBridgeDriver(10, 11);
        leftWheel = HBridgeDriver(6, 5);
        colorSense = ColorSensorDriver();
    }

    void forward()
    {
        leftWheel.setSpeed(1);
        rightWheel.setSpeed(1);
        // delayMicroseconds(time * 1000);
        // delay(time);
        // leftWheel.setSpeed(0);
        // rightWheel.setSpeed(0);
    }
    void backward()
    {
        leftWheel.setSpeed(-1);
        rightWheel.setSpeed(-1);
        // delayMicroseconds(time * 1000);
        // leftWheel.setSpeed(0);
        // rightWheel.setSpeed(0);
    }
    void turnLeft()
    {
        leftWheel.setSpeed(-1);
        rightWheel.setSpeed(1);
        // delayMicroseconds(time * 1000);
        // leftWheel.setSpeed(0);
        // rightWheel.setSpeed(0);
    }
    void turnRight()
    {
        leftWheel.setSpeed(1);
        rightWheel.setSpeed(-1);
        // delayMicroseconds(time * 1000);
        // leftWheel.setSpeed(0);
        // rightWheel.setSpeed(0);
    }

    void disableMotors()
    {
        leftWheel.setSpeed(0);
        rightWheel.setSpeed(0);
    }

    void mileStone3()
    {
        COLOR initialColor = colorSense.getReading();
        while (1)
        {
            COLOR measuredColor = colorSense.getReading();
            Serial.println(measuredColor);
            if (measuredColor == initialColor)
            {
                Serial.println("Same color");
                forward();
                _delay_ms(200);
                disableMotors();
            }
            else if (measuredColor == BLACK)
            {
                // Serial.println("Hit black");
                backward();
                _delay_ms(1000);
                turnRight();
                _delay_ms(300);
                disableMotors();
            }
            else // Other side
            {
                Serial.println("Hit other color?");
                forward();
                _delay_ms(100);
                turnLeft();
                _delay_ms(100);
                disableMotors();
                measuredColor = colorSense.getReading();
                int measuredColorRaw = colorSense.getReadingRaw();
                if (measuredColor == BLACK)
                {
                    Serial.println("Actually hit black");

                    continue;
                }
                Serial.println("Definitely hit other color");
                Serial.println(measuredColorRaw);
                turnRight(); // 180
                _delay_ms(1200);
                forward();
                _delay_ms(1000);
                disableMotors();
                _delay_ms(100);
                break;
            }
        }
    }
};

Robot meowBot;

///////////////////////////////////////////////////////////////////
/////////////////////// MAIN CONTROL LOOPS ////////////////////////
///////////////////////////////////////////////////////////////////

int main(void)
{
    // init(); // Needed for efficient serial

    Serial.begin(115200);
    Serial.println("Hello world\n");

    meowBot.mileStone3();

    for (;;)
    {
        Serial.println("Done:");
        // Serial.println(meowBot.colorSense.getReading());
        _delay_ms(1000);
    }

    return 0;
}