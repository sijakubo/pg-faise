package uni.oldenburg.server.service;

import com.google.gwt.user.server.rpc.RemoteServiceServlet;
import uni.oldenburg.client.service.SessionInformationService;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpSession;

/**
 * @author sijakubo
 */
public class SessionInformationServiceImpl extends RemoteServiceServlet implements SessionInformationService {

   public boolean isUserLoggedInInSession() {
      HttpServletRequest httpServletRequest = this.getThreadLocalRequest();
      HttpSession session = httpServletRequest.getSession();
      return session.getAttribute("user") != null;
   }
}
