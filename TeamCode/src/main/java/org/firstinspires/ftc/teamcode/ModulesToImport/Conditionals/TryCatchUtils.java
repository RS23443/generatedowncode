package org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals;
public class TryCatchUtils {

    /**
     * Executes a block of code wrapped in a try-catch block.
     *
     * @param runnable The code block to execute.
     * @param exceptionHandler The code to run if an exception occurs.
     */
    public static void executeWithTryCatch(Runnable runnable, Runnable exceptionHandler) {
        try {
            runnable.run();
        } catch (Exception e) {
            // Execute the exception handler
            exceptionHandler.run();

            // Optionally, log the exception (customize this as needed)
            System.err.println("Exception caught: " + e.getMessage());
            e.printStackTrace();
        }
    }
}
