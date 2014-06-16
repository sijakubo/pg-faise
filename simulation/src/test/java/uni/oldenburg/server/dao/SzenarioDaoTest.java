package uni.oldenburg.server.dao;

import junit.framework.Assert;
import org.junit.Ignore;
import org.junit.Test;
import uni.oldenburg.shared.model.Szenario;

import java.sql.SQLException;

public class SzenarioDaoTest {
   
   @Test
   public void testLoadSzenario() {
      SzenarioDao szenarioDao = new SzenarioDao();

      try {
         Szenario testSzenario = szenarioDao.loadSzenario("TestSzenario");
         Assert.assertNotNull(testSzenario);
         Assert.assertTrue(testSzenario.getTitle().equals("TestSzenario"));
      } catch (SQLException e) {
         e.printStackTrace();
         Assert.fail();
      }
   }
   
   @Test
   public void testPersistSzenario() {
      SzenarioDao szenarioDao = new SzenarioDao();

      try {
    	 //Dummy Szenario is created and persisted
    	 Szenario szenario=new Szenario();
         szenarioDao.persistSzenario(szenario);
         //Then load it to check if it was persistes
         Szenario testSzenario=szenarioDao.loadSzenario("new Szenario");
         Assert.assertNotNull(testSzenario);
         Assert.assertTrue(testSzenario.getTitle().equals("new Szenario"));
         //Remove the szenario, because the Databse should not be changed by a test
         szenarioDao.deleteSzenario("new Szenario");
      } catch (SQLException e) {
         e.printStackTrace();
         Assert.fail();
      }
   }
   
   @Test
   public void testCheckIfTitleExists() {
      SzenarioDao szenarioDao = new SzenarioDao();

      try {       
         //Assert that Method returns true for an existing Szenario and false for a not existing Szenario
         Assert.assertTrue(szenarioDao.checkIfTitleExists("TestSzenario"));
         Assert.assertFalse(szenarioDao.checkIfTitleExists("Testing456789"));
      } catch (SQLException e) {
         e.printStackTrace();
         Assert.fail();
      }
   }
}
