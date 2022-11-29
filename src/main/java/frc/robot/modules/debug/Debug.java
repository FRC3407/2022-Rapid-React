package frc.robot.modules.debug;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.format.DateTimeFormatter;
import java.time.LocalDateTime;    
public class Debug {
    static File debugFile = null;
    static File consoleFile = null;
    static FileOutputStream outStream = null;
    static ExtPrintStream stream = null;
    static ExtPrintStream errStream = null;
    public static enum Level {
        INFO (0),
        WARNING (1),
        ERROR (2);
    
        public final int val;
        private Level(int v) 
        { 
            this.val = v; 
        }
    }
    
    /**
     * @return File: debugFile currently in use
     */ 
    public static File getFile()
    {
        return debugFile;
    }

    /**
     * @return boolean: true if there is a debugFile present
     */
    public static boolean isFile()
    {
        if(debugFile != null)
        {
            return true;
        }
        return false;
    }

    /**
     * Sets the system stream to both the console and the output log file
     * @throws Exception handled locally
     */
    static private void setStream()
    {
        try 
        {
            //sets System.out and File as output streams
            outStream = new FileOutputStream(consoleFile);
            stream = new ExtPrintStream(outStream, System.out, false);
            errStream = new ExtPrintStream(outStream, System.err, true);
            System.setOut(stream);
            System.setErr(errStream);
        } 
        catch (Exception e) 
        {
            Debug.log("An error ocoured", 2);
            e.printStackTrace();
        }
    }

    /**
     * Creates a new log file based on the system time that can store the console output and manual Debug.log() commands
     * @return boolean: true if no issues prevent the file from being created
     * @throws IOExcepetion handled locally
     */
    public static boolean newLog()
    {
        DateTimeFormatter dtfFile = DateTimeFormatter.ofPattern("[yyyy-MM-dd_HH-mm-ss]");
        LocalDateTime now = LocalDateTime.now();  

        try

        {
            debugFile = new File(System.getProperty("user.dir")+"\\resources\\logs\\"+dtfFile.format(now)+".txt");
            consoleFile = new File(System.getProperty("user.dir")+"\\resources\\logs\\"+dtfFile.format(now)+".txt");
            setStream();
            debugFile.createNewFile();
        }
        catch (IOException e)
        {
            System.err.println("Debug log or console stream creation failed");
            e.printStackTrace();
            return false;
        }
        Debug.log("Created console stream");
        System.out.println("Created debug log");
        return true;
    }

    /**
     * Logs to the current debugLog with a timestamp and marked as [Info]
     * @return boolean: true if file is successfully logged to
     * @throws IOException handled locally
     */
    public static boolean log()
    {
        return log("", 0);
    }

     /**
     * Logs to the current debugLog with a timestamp and marked as [Info]
     * @param msg - String message sent to the log
     * @return boolean: true if file is successfully logged to
     * @throws IOException handled locally
     */
    public static boolean log(String msg)
    {
        return log(msg, 0);
    }

    /**
     * Logs to the current debugLog with a timestamp and marked with a info stamp
     * @param msg - String message sent to the log
     * @param errorLevel - int that applies an info stamp, 0-info 1-warning 2-error, default 0-info
     * @return boolean: true if file is successfully logged to
     * @throws IOException handled locally
     */
    public static boolean log(String msg, int errorLevel)
    {
        if(!isFile())
        {
            return false;
        }
        DateTimeFormatter dtfFile = DateTimeFormatter.ofPattern("[HH:mm:ss]");
        LocalDateTime now = LocalDateTime.now();  
        StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
        FileWriter writer = null;
        BufferedWriter bufferedWriter = null;
        PrintWriter printWriter = null;
        String msgLevel = "";
        try
        {
            try
            {
                writer = new FileWriter(debugFile, true);
            }
            catch(IOException e)
            {
                System.err.println("File writer creation failed");
                e.printStackTrace();
                return false;
            }
            bufferedWriter = new BufferedWriter(writer);
            printWriter = new PrintWriter(bufferedWriter);
            Level level = Level.values()[errorLevel];
            switch(level) {
                case INFO:
                    msgLevel = "[Info]";
                    break;
                case WARNING:
                    msgLevel = "[Warning]";
                    break;
                case ERROR:
                    msgLevel = "[Error]";
                    break;
            }
            printWriter.println("["+dtfFile.format(now)+"] "+msgLevel+" ("+stackTraceElements[2].getClassName()+"."+stackTraceElements[2].getMethodName()+") "+msg);
            printWriter.flush();
        }
        finally
        {
            try
            {
                printWriter.close();
                bufferedWriter.close();
                writer.close();
            }
            catch (IOException e)
            {
                System.err.println("printWriter, bufferedWriter, or writer failed to close");
                e.printStackTrace();
                return false;
            }
        }
        return true;
    }

    /**
     * Not implemented
     * @return
     */
    public static float inputDelay()
    {
        //takes time input stamps from controls and calculates the difference, printing it to the log
        return 0;
    }
}
