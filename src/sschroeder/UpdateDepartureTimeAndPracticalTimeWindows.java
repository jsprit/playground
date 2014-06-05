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

import jsprit.core.algorithm.state.StateManager;
import jsprit.core.algorithm.state.StateUpdater;
import jsprit.core.problem.cost.VehicleRoutingTransportCosts;
import jsprit.core.problem.solution.route.VehicleRoute;
import jsprit.core.problem.solution.route.activity.ReverseActivityVisitor;
import jsprit.core.problem.solution.route.activity.TourActivity;

/**
 * Updates and memorizes latest operation start times at activities.
 * 
 * @author schroeder
 *
 */
class UpdateDepartureTimeAndPracticalTimeWindows implements ReverseActivityVisitor, StateUpdater{

	private StateManager states;
	
	private VehicleRoute route;
	
	private VehicleRoutingTransportCosts transportCosts;
	
	private double latestArrTimeAtPrevAct;
	
	private TourActivity prevAct;
	
	private double maxOperationTimeOfDriver;
	
	public UpdateDepartureTimeAndPracticalTimeWindows(StateManager states, VehicleRoutingTransportCosts transportCosts, double maxDriverTime) {
		super();
		this.states = states;
		this.transportCosts = transportCosts;
		maxOperationTimeOfDriver = maxDriverTime;
	}

	@Override
	public void begin(VehicleRoute route) {
		this.route = route;
		double newDepartureTime = getNewDepartureTime(route);
		double latestArrAtDepot = Math.min(route.getEnd().getTheoreticalLatestOperationStartTime(),newDepartureTime+maxOperationTimeOfDriver);
		states.putTypedRouteState(route, StateIds.LATEST_ARR_AT_DEPOT, Double.class, latestArrAtDepot);
		latestArrTimeAtPrevAct = latestArrAtDepot;
		prevAct = route.getEnd();
	}

	@Override
	public void visit(TourActivity activity) {
		double potentialLatestArrivalTimeAtCurrAct = latestArrTimeAtPrevAct - transportCosts.getBackwardTransportTime(activity.getLocationId(), prevAct.getLocationId(), latestArrTimeAtPrevAct, route.getDriver(),route.getVehicle()) - activity.getOperationTime();
		double latestArrivalTime = Math.min(activity.getTheoreticalLatestOperationStartTime(), potentialLatestArrivalTimeAtCurrAct);
		states.putTypedActivityState(activity, StateIds.LATEST_ACTIVITY_START, Double.class, latestArrivalTime);
		latestArrTimeAtPrevAct = latestArrivalTime;
		prevAct = activity;
	}

	@Override
	public void finish() {}
	
	
	public double getNewDepartureTime(VehicleRoute route){
		double earliestDepartureTime = route.getDepartureTime();
		TourActivity firstActivity = route.getActivities().get(0);
		double tpTime_startToFirst = transportCosts.getTransportTime(route.getStart().getLocationId(), firstActivity.getLocationId(), 
				earliestDepartureTime, null, route.getVehicle());
		double newDepartureTime = Math.max(earliestDepartureTime, firstActivity.getTheoreticalEarliestOperationStartTime()-tpTime_startToFirst);
		return newDepartureTime;
	}

}