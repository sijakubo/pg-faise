package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.AsyncCallback;
import org.apache.log4j.Level;

public interface LoggingServiceAsync {
   void logOnServer(String message, AsyncCallback<Void> async);

   void logOnServer(String message, Throwable e, AsyncCallback<Void> async);

   void logInfoOnServer(String message, AsyncCallback<Void> async);
}
