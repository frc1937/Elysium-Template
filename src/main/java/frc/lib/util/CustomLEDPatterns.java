package frc.lib.util;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.Arrays;

public class CustomLEDPatterns {
    public static final int LEDS_COUNT = 46;

    private static int counter;
    private static int previousColour = 0;

    private static final Timer timer = new Timer();

    private static Color8Bit[] buffer = new Color8Bit[LEDS_COUNT];

    private static int rainbowFirstPixel;

    static {
        timer.start();
    }

    public static Color8Bit[] reduceBrightness(Color8Bit[] colours, int brightnessPercentage) {
        double brightnessFactor = brightnessPercentage / 100.0;

        final Color8Bit[] adjustedColours = new Color8Bit[colours.length];

        for (int i = 0; i < colours.length; i++) {
            Color8Bit originalColor = colours[i];

            int newRed = (int) (originalColor.red * brightnessFactor);
            int newGreen = (int) (originalColor.green * brightnessFactor);
            int newBlue = (int) (originalColor.blue * brightnessFactor);

            newRed = Math.min(255, Math.max(0, newRed));
            newGreen = Math.min(255, Math.max(0, newGreen));
            newBlue = Math.min(255, Math.max(0, newBlue));

            adjustedColours[i] = new Color8Bit(newRed, newGreen, newBlue);
        }

        return adjustedColours;
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
     * Set the buffer from the colour
     * @param ledBuffer - the ledbuffer to set
     */
    public static AddressableLEDBuffer getBufferFromColours(AddressableLEDBuffer ledBuffer, Color8Bit[] buffer) {
        for (int i = 0; i < buffer.length; i++) {
            ledBuffer.setLED(i, buffer[i]);
        }

        return ledBuffer;
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
     * Generates a loading animation with two moving directions.
     * The animation moves from the center of the LED strip outwards in both directions.
     * This should be called periodically to update the state of the animation.
     *
     * @param color1 - The colour for the first direction.
     * @param color2 - The colour for the second direction.
     * @return - The filled buffer.
     */
    public static Color8Bit[] generateLoadingAnimationBuffer(Color8Bit color1, Color8Bit color2) {
        buffer = generateSingleColourBuffer(new Color8Bit(Color.kBlack)); // Clear the buffer

        int midPoint = LEDS_COUNT / 2; // Find the middle of the LED strip
        double time = timer.get() * 5; // Control the speed of the animation
        int progress = (int) time % (LEDS_COUNT / 2); // Get the current progress

        // Fill from the middle outwards with color1
        for (int i = midPoint - progress; i <= midPoint; i++) {
            if (i >= 0) {
                buffer[i] = color1;
            }
        }

        // Fill from the middle outwards in the opposite direction with color2
        for (int i = midPoint + progress; i >= midPoint; i--) {
            if (i < LEDS_COUNT) {
                buffer[i] = color2;
            }
        }

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
    public static Color8Bit[] generateCirclingBuffer(Color8Bit... colours) {
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
