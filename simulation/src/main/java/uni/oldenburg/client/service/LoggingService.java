package uni.oldenburg.client.service;

import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;
import org.apache.log4j.Level;

@RemoteServiceRelativePath("loggingService")
public interface LoggingService extends RemoteService {
   void logOnServer(String message, Throwable e);

   void logOnServer(String message);

   void logInfoOnServer(String message);
}
