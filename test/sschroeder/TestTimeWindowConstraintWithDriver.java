/*******************************************************************************
 * Copyright (C) 2013  Stefan Schroeder
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either 
 * version 3.0 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/
package sschroeder;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;
import static org.junit.Assert.assertEquals;

import jsprit.core.algorithm.state.StateManager;
import jsprit.core.problem.constraint.HardActivityStateLevelConstraint.ConstraintsStatus;
import jsprit.core.problem.cost.VehicleRoutingTransportCosts;
import jsprit.core.problem.driver.Driver;
import jsprit.core.problem.job.Job;
import jsprit.core.problem.misc.JobInsertionContext;
import jsprit.core.problem.solution.route.VehicleRoute;
import jsprit.core.problem.solution.route.activity.TourActivity;
import jsprit.core.problem.vehicle.Vehicle;
import jsprit.core.util.Coordinate;
import jsprit.core.util.CrowFlyCosts;
import jsprit.core.util.Locations;

import org.junit.Before;
import org.junit.Test;

public class TestTimeWindowConstraintWithDriver {
	
	VehicleRoute route;
	
	Job job;
	
	Driver newDriver;
	
	VehicleRoutingTransportCosts routingCosts;
	
	TourActivity prevAct;
	
	TourActivity nextAct;
	
	StateManager states;
	
	@Before
	public void doBefore(){
		
		job = mock(Job.class);
		newDriver = mock(Driver.class);
		routingCosts = new CrowFlyCosts(new Locations(){

			@Override
			public Coordinate getCoord(String id) {
				return Coordinate.newInstance(Double.valueOf(id.split(",")[0].trim()), Double.valueOf(id.split(",")[1].trim()));
			}
			
		});
		prevAct = getMockedActivity("0,10",10.,20.);
		nextAct = getMockedActivity("0,20",0.,20.);
		states = new StateManager(routingCosts);
		states.putTypedActivityState(prevAct, StateIds.LATEST_ACTIVITY_START, Double.class, 20.);
		states.putTypedActivityState(nextAct, StateIds.LATEST_ACTIVITY_START, Double.class, 20.);
		
	}
	
	@Test
	public void whenNewActNeedsToBeServedBeforePrevAct_returnNotFulfilledBreak(){
		Vehicle newVehicle = getMockedVehicle("0,0","0,0",0.,100.);
		JobInsertionContext iContext = getInsertionContext(newVehicle,0.);
		double maxDriverTime = 100;
		TimeWindowConstraintWithDriverTime c = new TimeWindowConstraintWithDriverTime(states, routingCosts, maxDriverTime);
		TourActivity newAct = getMockedActivity("0,10",0.,9.);
		
		assertEquals(ConstraintsStatus.NOT_FULFILLED_BREAK,c.fulfilled(iContext, prevAct, newAct, nextAct, 10.));
	}
	
	@Test
	public void whenNewActNeedsToBeServedAfterNextAct_returnNotFulfilled(){
		Vehicle newVehicle = getMockedVehicle("0,0","0,0",0.,100.);
		JobInsertionContext iContext = getInsertionContext(newVehicle,0.);
		double maxDriverTime = 100;
		TimeWindowConstraintWithDriverTime c = new TimeWindowConstraintWithDriverTime(states, routingCosts, maxDriverTime);
		TourActivity newAct = getMockedActivity("0,10",25.,30.);
		
		assertEquals(ConstraintsStatus.NOT_FULFILLED,c.fulfilled(iContext, prevAct, newAct, nextAct, 10.));
	}
	
	@Test
	public void whenNewActNeedsToBeServedAfterLatestNextAct_returnNotFulfilled(){
		Vehicle newVehicle = getMockedVehicle("0,0","0,0",0.,100.);
		JobInsertionContext iContext = getInsertionContext(newVehicle,0.);
		states.putTypedActivityState(nextAct, StateIds.LATEST_ACTIVITY_START, Double.class, 15.);
		double maxDriverTime = 100;
		TimeWindowConstraintWithDriverTime c = new TimeWindowConstraintWithDriverTime(states, routingCosts, maxDriverTime);
		TourActivity newAct = getMockedActivity("0,10",10.,15.);
		/*
		 * arrTime@Next=20 > latestArrTime@Next=15
		 */
		assertEquals(ConstraintsStatus.NOT_FULFILLED,c.fulfilled(iContext, prevAct, newAct, nextAct, 10.));
	}
	
	@Test
	public void whenVehicleArrTimeIsNotEnough_returnNotFulfilledBreak(){
		Vehicle newVehicle = getMockedVehicle("0,0","0,0",0.,19.);
		JobInsertionContext iContext = getInsertionContext(newVehicle,0.);
		
		states.putTypedActivityState(prevAct, StateIds.LATEST_ACTIVITY_START, Double.class, 9.);
		states.putTypedActivityState(nextAct, StateIds.LATEST_ACTIVITY_START, Double.class, 19.);
		
		double maxDriverTime = 100;
		TimeWindowConstraintWithDriverTime c = new TimeWindowConstraintWithDriverTime(states, routingCosts, maxDriverTime);
		TourActivity newAct = getMockedActivity("0,10",10.,15.);
		
		assertEquals(ConstraintsStatus.NOT_FULFILLED_BREAK,c.fulfilled(iContext, prevAct, newAct, nextAct, 10.));
	}
	
	@Test
	public void whenMaxDriverTimeIsNotEnough_returnNotFulfilledBreak(){
		Vehicle newVehicle = getMockedVehicle("0,0","0,0",0.,100.);
		JobInsertionContext iContext = getInsertionContext(newVehicle,0.);
		
		states.putTypedActivityState(prevAct, StateIds.LATEST_ACTIVITY_START, Double.class, 9.);
		states.putTypedActivityState(nextAct, StateIds.LATEST_ACTIVITY_START, Double.class, 19.);
		
		double maxDriverTime = 19;
		TimeWindowConstraintWithDriverTime c = new TimeWindowConstraintWithDriverTime(states, routingCosts, maxDriverTime);
		TourActivity newAct = getMockedActivity("0,10",10.,15.);
		
		assertEquals(ConstraintsStatus.NOT_FULFILLED_BREAK,c.fulfilled(iContext, prevAct, newAct, nextAct, 10.));
	}
	
	@Test
	public void whenMaxDriverTimeIsNotEnoughv2_returnNotFulfilledBreak(){
		Vehicle newVehicle = getMockedVehicle("0,0","0,0",0.,100.);
		JobInsertionContext iContext = getInsertionContext(newVehicle,0.);
		
		states.putTypedActivityState(prevAct, StateIds.LATEST_ACTIVITY_START, Double.class, 10.);
		states.putTypedActivityState(nextAct, StateIds.LATEST_ACTIVITY_START, Double.class, 20.);
		
		double maxDriverTime = 30;
		TimeWindowConstraintWithDriverTime c = new TimeWindowConstraintWithDriverTime(states, routingCosts, maxDriverTime);
		TourActivity newAct = getMockedActivity("0,10",10.,15.);
		
		assertEquals(ConstraintsStatus.NOT_FULFILLED_BREAK,c.fulfilled(iContext, prevAct, newAct, nextAct, 10.));
	}
	
	
	

	private JobInsertionContext getInsertionContext(Vehicle newVehicle, double newDepartureTime) {
		return new JobInsertionContext(route, job, newVehicle, newDriver, newDepartureTime);
	}

	private Vehicle getMockedVehicle(String startLoc, String endLoc, double earliestStart, double latestArr) {
		Vehicle vehicle = mock(Vehicle.class);
		when(vehicle.getStartLocationId()).thenReturn("0,0");
		when(vehicle.getEndLocationId()).thenReturn("0,0");
		when(vehicle.getEarliestDeparture()).thenReturn(earliestStart);
		when(vehicle.getLatestArrival()).thenReturn(latestArr);
		return vehicle;
	}
	
	public TourActivity getMockedActivity(String locationId, double earliestStart, double latestStart){
		TourActivity act = mock(TourActivity.class);
		when(act.getLocationId()).thenReturn(locationId);
		when(act.getTheoreticalEarliestOperationStartTime()).thenReturn(earliestStart);
		when(act.getTheoreticalLatestOperationStartTime()).thenReturn(latestStart);
		return act;
	}

}
