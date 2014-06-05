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

import java.util.Collection;

import jsprit.core.algorithm.listener.AlgorithmEndsListener;
import jsprit.core.algorithm.state.UpdateActivityTimes;
import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.cost.TransportTime;
import jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import jsprit.core.problem.solution.route.RouteActivityVisitor;
import jsprit.core.problem.solution.route.VehicleRoute;
import jsprit.core.problem.solution.route.activity.TourActivity;
import jsprit.core.util.Solutions;

public class DepartureTimeReScheduler implements AlgorithmEndsListener {

	@Override
	public void informAlgorithmEnds(VehicleRoutingProblem problem,Collection<VehicleRoutingProblemSolution> solutions) {
		VehicleRoutingProblemSolution solution = Solutions.bestOf(solutions);
		for(VehicleRoute route : solution.getRoutes()){
			if(!route.isEmpty()){
				double earliestDepartureTime = route.getDepartureTime();
				TourActivity firstActivity = route.getActivities().get(0);
				double tpTime_startToFirst = problem.getTransportCosts().getTransportTime(route.getStart().getLocationId(), firstActivity.getLocationId(), 
						earliestDepartureTime, null, route.getVehicle());
				double newDepartureTime = Math.max(earliestDepartureTime, firstActivity.getTheoreticalEarliestOperationStartTime()-tpTime_startToFirst);
				route.setVehicleAndDepartureTime(route.getVehicle(), newDepartureTime);
			}
		}
	}
	
	public void updateActivityTimes(VehicleRoute route, TransportTime transportTimes){
		RouteActivityVisitor routeVisitor = new RouteActivityVisitor();
		routeVisitor.addActivityVisitor(new UpdateActivityTimes(transportTimes));
		routeVisitor.visit(route);
	}
	
}