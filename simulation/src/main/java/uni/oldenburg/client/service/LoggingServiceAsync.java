package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.AsyncCallback;

public interface LoggingServiceAsync {
   void logOnServer(String message, AsyncCallback<Void> async);

   void logOnServer(String message, Throwable e, AsyncCallback<Void> async);

   void logInfoOnServer(String message, AsyncCallback<Void> async);
}
