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

// Write the given output to a pin
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

// Read the value of a pin
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

// Half period (in )
int half_period = 0;

///////////////////////////////////////////////////////////////////
//////////////////// COMMUNICATION PROTOCOLS //////////////////////
///////////////////////////////////////////////////////////////////

// TODO: Make this actually PWM and not binary
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
        // Fake PWM, turns out it was all that I needed for the competition
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

///////////////////////////////////////////////////////////////////
/////////////////////// PERIPHERAL DRIVERS ////////////////////////
///////////////////////////////////////////////////////////////////

#define MIN_PULSE 6
#define MAX_PULSE 36

// Class for interfacing with a continuous rotation servo over PWM
class ContinuousRotationServo
{
private:
    int pin;

public:
    ContinuousRotationServo() {}
    ContinuousRotationServo(int _pin)
    {
        pin = _pin;
        if (_pin != 5 && _pin != 6)
        {
            Serial.println("Only pins 5 or 6 supported");
        }
        setPinDirection(_pin, OUTPUT);
        // Generate ~50kHz PWM on pins 5 and 6 using fast PWM mode
        TCCR0A = 0b10100011;
        TCCR0B = 0b00000101;
        if (_pin == 6)
        {
            // Pulse lengths experimentally determined, 21 = stopped
            OCR0A = 21;
        }
        if (_pin == 5)
        {
            OCR0B = 21;
        }
    }

    void setSpeed(double speed)
    {
        int pulse = MIN_PULSE + (MAX_PULSE - MIN_PULSE) * speed;
        if (pin == 6)
        {
            OCR0A = pulse;
        }
        if (pin == 5)
        {
            OCR0B = pulse;
        }
    }
};

// Class for interfacing with an H Bridge over PWM
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

// Represents each color within an enum. Makes the code more intuitive to read
enum COLOR
{
    BLUE,
    YELLOW,
    BLACK
};

// Class for interfacing with the Color Sensor
class ColorSensorDriver
{
private:
    int outputPin = 2;
    // Cutoffs for detecting the color, in units of us per pulse.
    // Represents the upper bound for the given color. Experimentally determined.
    int yellowCutoff = 80;
    int blueCutoff = 380;

public:
    ColorSensorDriver()
    {
        unsetBit(&DDRD, outputPin); // Set sensor pin to input

        setBit(&PCICR, 2);          // Enable pin change interrupts for port D
        setBit(&PCMSK2, outputPin); // Enable pin change interrupts for output pin

        sei(); // Enable interupts

        setBit(&TCCR1B, 0); // Enable timer 1, with no prescaling
        // TCCR1A has correct vals for normal operation by default
    }

    // Returns the period of the sensor in us
    int getReadingRaw()
    {
        setBit(&PCMSK2, outputPin);   // Activate interrupts on the sensor pin
        _delay_ms(5);                 // Wait for a reading
        unsetBit(&PCMSK2, outputPin); // Disable interrupts on the sensor pin

        return ((double)2 * (double)half_period * (double)0.0625); // 0.0625 microseconds / clock tick
    }

    // Get the color sensor reading as a value of the enum
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
};

// RIP IMU, you would have been great
class IMUDriver
{

public:
    IMUDriver()
    {
    }

    int getReading()
    {
    }
};

///////////////////////////////////////////////////////////////////
////////////////////////// ROBOT SETUP ////////////////////////////
///////////////////////////////////////////////////////////////////

// This class represents the whole robot, and owns the sensors / actuators belonging to it.
class Robot
{
private:
    HBridgeDriver rightWheel;
    HBridgeDriver leftWheel;
    ContinuousRotationServo rotor;
    COLOR initialColor;
    COLOR oppositeColor;

public:
    ColorSensorDriver colorSense;

    double gameTime = 0; // The game time in (approximately) seconds. (Check the notes in the ISR for more details)

    Robot()
    {
        // Initialize all actuators
        rightWheel = HBridgeDriver(10, 11);
        leftWheel = HBridgeDriver(9, 8);
        rotor = ContinuousRotationServo(5);
        // Initialize sensors, take initial readings
        colorSense = ColorSensorDriver();
        initialColor = colorSense.getReading();
        while (initialColor == BLACK) // We will never start on black, so this must be an error. Retry and wait for a valid reading.
        {
            Serial.println(colorSense.getReadingRaw());
            initialColor = colorSense.getReading();
        }
        // Store the other color for convience
        if (initialColor == BLUE)
        {
            oppositeColor = YELLOW;
        }
        else
        {
            oppositeColor = BLUE;
        }

        // Start game timer
        // TCCR2A is by default 0
        TCCR2B = 0;
        TCCR2B |= 0x08;       // setting bit 3
        TCCR2B |= 0b00000101; // setting bits 0,2 - prescalar of 1024

        OCR2A = 155; // 1/100.8064516 seconds

        TIMSK2 |= (1 << 1); // enable CTC interrupt

        // Enable interrupts
        sei();
    }

    // Convience functions -> Do some commonly used actions
    void startRotor()
    {
        rotor.setSpeed(1);
    }

    void stopRotor()
    {
        rotor.setSpeed(0);
    }

    void forward()
    {
        leftWheel.setSpeed(1);
        rightWheel.setSpeed(1);
    }
    void backward()
    {
        leftWheel.setSpeed(-1);
        rightWheel.setSpeed(-1);
    }
    void turnLeft()
    {
        leftWheel.setSpeed(-1);
        rightWheel.setSpeed(1);
    }
    void turnRight()
    {
        leftWheel.setSpeed(1);
        rightWheel.setSpeed(-1);
    }

    void disableMotors()
    {
        leftWheel.setSpeed(0);
        rightWheel.setSpeed(0);
    }

    COLOR checkColor()
    {
        COLOR measuredColor = colorSense.getReading();
        if (measuredColor == initialColor)
        {
            return initialColor;
        }
        else if (measuredColor == BLACK)
        {
            return BLACK;
        }
        else
        {
            // Edge case: When the sensor is positioned over both a color and black, it will read the average color of the two.
            // Fix: If we detect a change in color, move forward slightly and take a second reading. The second reading can be trusted
            forward();
            _delay_ms(100);
            disableMotors();
            measuredColor = colorSense.getReading();
            if (measuredColor == BLACK)
            {
                return BLACK; // We actually hit black, even though we previously thought otherwise
            }
            return measuredColor; // The first reading was right
        }
    }

    // Routine used for milestone 3
    void milestone3()
    {
        while (1)
        {
            COLOR measuredColor = colorSense.getReading();
            Serial.println(colorSense.getReadingRaw());
            if (measuredColor == initialColor)
            {
                Serial.println("Same color");
                forward();
                _delay_ms(200);
                disableMotors();
            }
            else if (measuredColor == BLACK)
            {
                Serial.println("Hit black");
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

    // Routine used for milestone 4
    void milestone4()
    {
        turnRight();
        _delay_ms(300);
        forward();
        _delay_ms(3700);
        turnLeft();
        _delay_ms(400);
        forward();
        _delay_ms(700);
        turnLeft();
        _delay_ms(1000);
        forward();
        _delay_ms(3000);
        turnLeft();
        _delay_ms(1000);
        forward();
        _delay_ms(1000);
        disableMotors();
        // Ganar
    }

    // Routine used for competition
    void comp()
    {
        // Start the collection rotor
        startRotor();
        // Angle right, move until we have reached the other color
        turnRight();
        _delay_ms(200);
        forward();
        while (checkColor() == initialColor)
        {
            _delay_ms(50);
        }

        // Turn left, we are now on the other side of the field
        _delay_ms(300);
        turnLeft();
        _delay_ms(900);

        while (1)
        {
            // Go forward while we are within the opponents color
            forward();
            while (checkColor() == oppositeColor)
            {
                forward();
                _delay_ms(50);
            }
            // If we are here, the color sensor must have detected either BLACK or initialColor
            if (checkColor() == initialColor && gameTime > 40)
            {
                // We hit our start color, and the game is about to end.
                // We should return to our side so our points count.
                // Therefore, break out of the loop and continue straight.
                break;
            }
            // We want to stay on the opponents side, so back up and turn to try a new direction.
            backward();
            _delay_ms(1500);
            turnLeft();
            _delay_ms(600);
            // Start the loop over again
        }
        // If we are out of the loop, we must have hit our color with < 20 seconds remaining.
        // Get safely on our side, then turn everything off.
        forward();
        _delay_ms(1000);
        disableMotors();
    }
};

// The instance of the robot. Go team Meow!
Robot meowBot;

///////////////////////////////////////////////////////////////////
///////////////////////////// ISR's ///////////////////////////////
///////////////////////////////////////////////////////////////////

// Used for the color sensor, detect a pulse on pin 2
ISR(PCINT2_vect)
{
    if (getBit(&PIND, 2) != 0) // Pin is now high -> rising edge
    {
        TCNT1 = 0; // Set timer to 0
    }
    else
    {
        half_period = TCNT1; // Read timer into variable
    }
}

// Used for integrating game time
ISR(TIMER2_COMPA_vect)
{
    // Note: I'm not sure why the / 4 is needed here.
    // By my math, each interrupt should be ~ 0.01 second, but that was incrementing much too fast.
    // / 4 makes the count close to accurate, but still isn't quite right.
    // Regardless, the timing is consistent, even if the units are not in seconds as I had hoped.
    // This means we can trust it to measure game time so long as we find the corresponding game time value for the realtime equivalent.
    meowBot.gameTime += (1 / 100.8064516) / 4;
}

///////////////////////////////////////////////////////////////////
/////////////////////// MAIN CONTROL LOOPS ////////////////////////
///////////////////////////////////////////////////////////////////

int main(void)
{
    Serial.begin(115200);
    Serial.println("Hello world\n"); // Let everyone know we're here

    meowBot.comp(); // Run our game logic

    for (;;)
    {
        // If we're here, meowBot.comp() returned and therefore we are done.
        // Busy wait cause exiting is for chumps
        Serial.println("Done:");
        _delay_ms(1000);
    }

    return 0;
}