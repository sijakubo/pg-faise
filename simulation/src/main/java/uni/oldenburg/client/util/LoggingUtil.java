package uni.oldenburg.client.util;

import com.google.gwt.core.client.GWT;
import uni.oldenburg.client.service.LoggingService;
import uni.oldenburg.client.service.LoggingServiceAsync;

public class LoggingUtil {

   /**
    * @author sijakubo
    */
   public static void logMessageToServer(String message) {
      LoggingServiceAsync loggingService = GWT.create(LoggingService.class);
      loggingService.logOnServer(message, new EmptyAsyncCallback());
   }

   /**
    * @author sijakubo
    */
   public static void logMessageToServer(String message, Throwable e) {
      LoggingServiceAsync loggingService = GWT.create(LoggingService.class);
      loggingService.logOnServer(message, e, new EmptyAsyncCallback());
   }

   /**
    * @author sijakubo
    */
   public static void logInfoMessageToServer(String message, Throwable e) {
      LoggingServiceAsync loggingService = GWT.create(LoggingService.class);
      loggingService.logInfoOnServer(message, new EmptyAsyncCallback());
   }
}
