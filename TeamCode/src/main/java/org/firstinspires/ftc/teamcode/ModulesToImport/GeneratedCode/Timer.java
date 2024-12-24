package org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode;

public class Timer {
    private static long startTime;

    // Start the timer
    public static void start()   {
        startTime = System.currentTimeMillis();
    }

    // Get the elapsed time in milliseconds
    public static long getElapsedTimeMillis() {
        return (System.currentTimeMillis() - startTime);
    }

}
