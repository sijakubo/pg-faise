package uni.oldenburg.client.util;

import com.google.gwt.core.client.GWT;
import com.google.gwt.user.client.rpc.AsyncCallback;
import uni.oldenburg.client.service.LoggingService;
import uni.oldenburg.client.service.LoggingServiceAsync;

public class LoggingUtil {

   /**
    * @author sijakubo
    */
   public static void logMessageToServer(String message) {
      LoggingServiceAsync loggingService = GWT.create(LoggingService.class);
      loggingService.logOnServer(message, createEmptyAsyncCallback());
   }

   /**
    * @author sijakubo
    */
   public static void logMessageToServer(String message, Throwable e) {
      LoggingServiceAsync loggingService = GWT.create(LoggingService.class);
      loggingService.logOnServer(message, e, createEmptyAsyncCallback());
   }

   /**
    * @author sijakubo
    */
   public static void logInfoMessageToServer(String message, Throwable e) {
      LoggingServiceAsync loggingService = GWT.create(LoggingService.class);
      loggingService.logInfoOnServer(message, createEmptyAsyncCallback());
   }

   /**
    * @author sijakubo
    */
   private static AsyncCallback<Void> createEmptyAsyncCallback() {
      return new AsyncCallback<Void>() {
         public void onFailure(Throwable caught) {
            //doNothing.
         }

         public void onSuccess(Void result) {
            //doNothing.
         }
      };
   }
}
