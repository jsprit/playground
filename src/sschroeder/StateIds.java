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

import jsprit.core.problem.solution.route.state.StateFactory;
import jsprit.core.problem.solution.route.state.StateFactory.StateId;

public class StateIds {
	
	public static final StateId LATEST_ACTIVITY_START = StateFactory.createId("latest_start");
	
	public static final StateId DEPARTURE_AT_DEPOT = StateFactory.createId("departure");
	
	public static final StateId LATEST_ARR_AT_DEPOT = StateFactory.createId("latestArrTimeAtDepot");

	public static final StateId ARRIVAL_AT_DEPOT = StateFactory.createId("arrTimeAtDepot");;

}
