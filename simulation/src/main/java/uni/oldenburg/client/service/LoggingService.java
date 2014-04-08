package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;

@RemoteServiceRelativePath("loggingService")
public interface LoggingService extends RemoteService {
   void logOnServer(String message, Throwable e);

   void logOnServer(String message);

   void logInfoOnServer(String message);
}
