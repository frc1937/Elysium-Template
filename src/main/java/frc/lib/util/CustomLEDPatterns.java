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
        double x = timer.get();
        return generateSingleColourBuffer(interpolateColours(firstColour, secondColour, cosInterpolate(x)));
    }


    /**
     * Circle N colours across the whole LEDStrip. Utilizes a smooth effect. Needs to be called periodic.
     * @param colours - The colours
     * @return - The filled buffer
     */
    public static Color8Bit[] generateNewCirclingBuffer(Color8Bit... colours) {
        int colorsLength = colours.length;
        float timerValue = (float) timer.get(); // Get current timer value
        float timerPosition = timerValue * 13f % LEDS_COUNT; // Adjust the multiplier to control the speed

        int index, colorIndex1, colorIndex2;
        float colorIndexFloat, fraction;

        for (int i = 0; i < LEDS_COUNT; i++) {
            index = wrapIndex(i);

            // Calculate the floating point color index for smooth transitions
            colorIndexFloat = (timerPosition + i) % LEDS_COUNT / LEDS_COUNT * colorsLength;

            // Determine the previous and next color indices for interpolation
            colorIndex1 = (int) colorIndexFloat;
            colorIndex2 = (colorIndex1 + 1) % colorsLength;

            // Fractional part for interpolation (cosine interpolation for smooth transitions)
            fraction = (float) cosInterpolate(colorIndexFloat - colorIndex1);

            // Interpolate between the two colors
            Color8Bit color1 = colours[colorIndex1];
            Color8Bit color2 = colours[colorIndex2];

            Color8Bit interpolatedColor = interpolateColours(color1, color2, fraction);

            buffer[index] = interpolatedColor;
        }

        return buffer;
    }


    /**
     * Clears the buffer, then moves a colour from the middle outwards.
     * Creating a nice loading effect. Should be used periodically.
     * @param colour - the colour to use
     * @return - The current state of the buffer
     */
    public static Color8Bit[] generateOutwardsPointsBuffer(Color8Bit colour) {
        buffer = generateSingleColourBuffer(new Color8Bit(Color.kBlack));

        final int quarter = LEDS_COUNT / 4;

        final double time = timer.get();

        int x = time == (int) time ? ((int) (time) % 11) : ((int) (time * 16 % 11));

        for (int i = quarter - 1 - x; i < quarter + 1 + x; i++) {
            buffer[i] = new Color8Bit(new Color(colour.red, colour.green, colour.blue));
        }

        for (int i = quarter * 3 - x; i < 2 + quarter * 3 + x; i++) {
            buffer[i] = new Color8Bit(new Color(colour.red, colour.green, colour.blue));
        }

        return buffer;
    }


    private static int wrapIndex(int i) {
        while (i >= LEDS_COUNT)
            i -= LEDS_COUNT;

        while (i < 0) {
            i += LEDS_COUNT;
        }

        return i;
    }


    private static Color8Bit interpolateColours(Color8Bit color1, Color8Bit color2, double fraction) {
        int red = (int) (color1.red * (1 - fraction) + color2.red * fraction);
        int green = (int) (color1.green * (1 - fraction) + color2.green * fraction);
        int blue = (int) (color1.blue * (1 - fraction) + color2.blue * fraction);

        return new Color8Bit(red, green, blue);
    }

    private static double cosInterpolate(double x) {
        return ((1 - Math.cos(x * Math.PI)) * 0.5);
    }
}
