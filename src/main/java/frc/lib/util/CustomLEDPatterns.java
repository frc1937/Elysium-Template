package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.Arrays;

public class CustomLEDPatterns {
    private static final int LEDS_COUNT = 40;

    private static int counter;
    private static int previousColour = 0;

    private static final Timer timer = new Timer();

    private static Color8Bit[] buffer = new Color8Bit[LEDS_COUNT];

    private static int rainbowFirstPixel;

    static {
        timer.start();
    }

    /**
     * Generates a buffer with a single colour
     *
     * @param colour - the colour
     * @return - The filled buffer.
     */
    public static Color8Bit[] generateSingleColourBuffer(Color8Bit colour) {
        Arrays.fill(buffer, colour);
        return buffer;
    }

    /**
     * Fill the buffer with RAINBOW colours. This needs to be called periodically
     * in order for the rainbow to not be static.
     *
     * @return The filled buffer.
     */
    public static Color8Bit[] generateRainbowBuffer() {
        int hue;

        for (var i = 0; i < LEDS_COUNT; i++) {
            hue = (rainbowFirstPixel + (i * 180 / LEDS_COUNT)) % 180;

            buffer[i] = new Color8Bit(Color.fromHSV(hue, 255, 128));
        }

        rainbowFirstPixel += 3;
        rainbowFirstPixel %= 180;

        return buffer;
    }


    /**
     * Set the buffer to any amount of colours and quickly iterate between them.
     * This needs to be called periodic in order for the buffer to flash between colours.
     *
     * @param colours - The colours to switch between
     * @return - The filled buffer
     */
    public static Color8Bit[] generateFlashingBuffer(Color8Bit... colours) {
        if (counter % 25 == 0) //Make sure there's a delay between colour switching
            buffer = generateSingleColourBuffer(colours[previousColour++]);

        previousColour %= colours.length;
        counter++;

        return buffer;
    }


    /**
     * Slowly switch between two colours, creating a breathing effect
     *
     * @param firstColour  - The first colour
     * @param secondColour - The second colour
     * @return - The filled buffer.
     */
    public static Color8Bit[] generateBreathingBuffer(Color8Bit firstColour, Color8Bit secondColour) {
        double x = ((timer.get()) * 2.0 * Math.PI);
        return generateSingleColourBuffer(getRGBFromXAndColours(x, firstColour, secondColour));
    }

    public static Color8Bit[] generateNewCirclingBuffer(Color8Bit... colours) {
        int colorsLength = colours.length;
        int timerPosition = (int) (timer.get() * 46 % LEDS_COUNT);

        for (int i = 0; i < LEDS_COUNT; i++) {
            int index = wrapIndex(i);
            int colorIndex = (timerPosition + index) % colorsLength;
            buffer[index] = colours[colorIndex];
        }

        return buffer;
    }

    @Deprecated
    public static Color8Bit[] generateCirclingBuffer(Color8Bit c1, Color8Bit c2) {
        int halfLength = LEDS_COUNT / 2;
        int timerPosition = (int) (timer.get() * 46 % LEDS_COUNT);

        for (int i = 0; i < LEDS_COUNT; i++) {
            int index = wrapIndex(i);

            if (index >= timerPosition && index < timerPosition + halfLength) {
                buffer[index] = c1;
            } else {
                buffer[index] = c2;
            }
        }

        return buffer;
    }
//
//    private void setBufferToOutwardy(Color c1) {
//        generateSingleColourBuffer(new Color8Bit(Color.kBlack));
//
//        int quarter = LEDS_COUNT / 4;
//
//        final double time = timer.get();
//
//        int x = time == (int) time ? ((int) (time) % 11) : ((int) (time * 16 % 11));
//
//        SmartDashboard.putNumber("x", x);
//
//        for (int i = quarter - 1 - x; i < quarter + 1 + x; i++) {
//            buffer.setLED(i, new Color(c1.red, c1.green, c1.blue));
//        }
//
//        for (int i = quarter * 3 - x; i < 2 + quarter * 3 + x; i++) {
//            buffer.setLED(i, new Color(c1.red, c1.green, c1.blue));
//        }
//    }


    private static int wrapIndex(int i) {
        while (i >= LEDS_COUNT)
            i -= LEDS_COUNT;

        while (i < 0) {
            i += LEDS_COUNT;
        }

        return i;
    }


    private static Color8Bit getRGBFromXAndColours(double x, Color8Bit c1, Color8Bit c2) {
        double ratio = (Math.sin(x) + 1.0) / 2.0;

        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);

        return new Color8Bit((int) red, (int) green, (int) blue);
    }
}
