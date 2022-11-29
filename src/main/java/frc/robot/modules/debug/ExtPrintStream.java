package frc.robot.modules.debug;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;


/** 
 * En extension of PrintStream that splits the output between two streams
*/
public class ExtPrintStream extends PrintStream {
    private final PrintStream second;
    private final DateTimeFormatter dtf = DateTimeFormatter.ofPattern("[HH:mm:ss]");
    private boolean isErrStream;
    public ExtPrintStream(OutputStream outputStream, PrintStream printStream, boolean isErrStream)
    {
        super(outputStream);
        this.second = printStream;
        this.isErrStream = isErrStream;
    }

    @Override
    public void close() 
    {
        super.close();
        second.close();
    }

    @Override
    public void flush() 
    {
        super.flush();
        second.flush();
    }

    @Override
    public void write(byte[] buf, int off, int len) 
    {
        super.write(buf, off, len);
        second.write(buf, off, len);
    }

    @Override
    public void write(int b) 
    {
        super.write(b);
        second.write(b);
    }

    @Override
    public void write(byte[] b) throws IOException 
    {
        super.write(b);
        second.write(b);
    }

    /** 
    * Prints as normal to the second stream, then prints to the first stream with the applied time and info stamp
    * @param x - String printed to both streams
    * @throws IOException handled locally
    */
    @Override
    public void println(String x)
    {
        second.println(x);
        if (Debug.isFile())
        {
            LocalDateTime now = LocalDateTime.now();  
            StackTraceElement[] stackTraceElements = Thread.currentThread().getStackTrace();
            FileWriter writer = null;
            BufferedWriter bufferedWriter = null;
            PrintWriter printWriter = null;
            try
            {
                try
                {
                    writer = new FileWriter(Debug.getFile(), true);
                }
                catch(IOException e)
                {
                    
                }
                    bufferedWriter = new BufferedWriter(writer);
                    printWriter = new PrintWriter(bufferedWriter);
                    if(isErrStream)
                    {
                        printWriter.println("["+dtf.format(now)+"] [Error] (on line "+stackTraceElements[2].getLineNumber()+" of "+stackTraceElements[2].getClassName()+"."+stackTraceElements[2].getMethodName()+") "+x);
                    }
                    else
                    {
                        printWriter.println("["+dtf.format(now)+"] [Info] ("+stackTraceElements[2].getClassName()+"."+stackTraceElements[2].getMethodName()+") "+x);
                    }
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
                }
            }
        }
    }
}