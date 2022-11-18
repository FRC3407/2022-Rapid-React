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
    public static final int LEVEL_INFO = 0;
    public static final int LEVEL_WARNING = 1;
    public static final int LEVEL_ERROR = 2;
    
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
            stream = new ExtPrintStream(outStream, System.out);
            System.setOut(stream);
            System.setErr(stream);
            
            
        } catch (Exception e) {
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
            System.out.println("Debug log or console stream creation failed");
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
        try
        {
            try
            {
                writer = new FileWriter(debugFile, true);
            }
            catch(IOException e)
            {
                Debug.log("An error ocoured", 2);
                e.printStackTrace();
                return false;
            }
            bufferedWriter = new BufferedWriter(writer);
            printWriter = new PrintWriter(bufferedWriter);
            printWriter.println("["+dtfFile.format(now)+"] [Info] ("+stackTraceElements[2].getClassName()+"."+stackTraceElements[2].getMethodName()+")");
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
                Debug.log("An error ocoured", 2);
                e.printStackTrace();
                return false;
            }
        }
        return true;
    }

     /**
     * Logs to the current debugLog with a timestamp and marked as [Info]
     * @param msg - String message sent to the log
     * @return boolean: true if file is successfully logged to
     * @throws IOException handled locally
     */
    public static boolean log(String msg)
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
        
        try
        {
            try
            {
                writer = new FileWriter(debugFile, true);
            }
            catch(IOException e)
            {
                Debug.log("An error ocoured", 2);
                e.printStackTrace();
                return false;
            }
            bufferedWriter = new BufferedWriter(writer);
            printWriter = new PrintWriter(bufferedWriter);
            
            printWriter.println("["+dtfFile.format(now)+"] [Info] ("+stackTraceElements[2].getClassName()+"."+stackTraceElements[2].getMethodName()+") "+msg);
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
                Debug.log("An error ocoured", 2);
                e.printStackTrace();
                return false;
            }
        }
        return true;
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
                Debug.log("An error ocoured", 2);
                e.printStackTrace();
                return false;
            }
            bufferedWriter = new BufferedWriter(writer);
            printWriter = new PrintWriter(bufferedWriter);
            switch(errorLevel) {
                case LEVEL_INFO:
                    msgLevel = "[Info]";
                    break;
                case LEVEL_WARNING:
                    msgLevel = "[Warning]";
                    break;
                case LEVEL_ERROR:
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
                Debug.log("An error ocoured", 2);
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
