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

import jsprit.core.problem.constraint.HardActivityStateLevelConstraint;
import jsprit.core.problem.cost.VehicleRoutingTransportCosts;
import jsprit.core.problem.misc.JobInsertionContext;
import jsprit.core.problem.solution.route.activity.End;
import jsprit.core.problem.solution.route.activity.Start;
import jsprit.core.problem.solution.route.activity.TourActivity;
import jsprit.core.problem.solution.route.state.RouteAndActivityStateGetter;
import jsprit.core.util.CalculationUtils;


/**
	 * 
	 * @author stefan
	 *
	 */
	class TimeWindowConstraintWithDriverTime implements HardActivityStateLevelConstraint {

		private RouteAndActivityStateGetter states;
		
		private VehicleRoutingTransportCosts routingCosts;
		
		private double maxDriverTime;
		
		public TimeWindowConstraintWithDriverTime(RouteAndActivityStateGetter states, VehicleRoutingTransportCosts routingCosts, double maxDriverTime) {
			super();
			this.states = states;
			this.routingCosts = routingCosts;
			this.maxDriverTime=maxDriverTime;
		}

		@Override
		public ConstraintsStatus fulfilled(JobInsertionContext iFacts, TourActivity prevAct, TourActivity newAct, TourActivity nextAct, double prevActDepTime) {
			
//			double actualDeparture = getActualDeparture(iFacts,newAct);
			double latest_arr_at_depot = getLatestArrAtDepot(iFacts,newAct);
			
			/*
			 * if actualDeparture changes --> latestArr changes too
			 * 
			 * 
			 * 
			 */
			double actualEnd = getCurrentArrTimeAtEnd(iFacts);
			if(prevAct instanceof Start){
				double newDepartureTime = getVehicleDeparture(iFacts, newAct);
				if(actualEnd - newDepartureTime > maxDriverTime){
					return ConstraintsStatus.NOT_FULFILLED;
				}
			}
			
			
			double latestVehicleArrival = Math.min(iFacts.getNewVehicle().getLatestArrival(),latest_arr_at_depot);
			
			/*
			 * if latest arrival of vehicle (at its end) is smaller than earliest operation start times of activities,
			 * then vehicle can never conduct activities.
			 * 
			 *     |--- vehicle's operation time ---|
			 *                        					|--- prevAct or newAct or nextAct ---|
			 */
			if(latestVehicleArrival < prevAct.getTheoreticalEarliestOperationStartTime() || 
					latestVehicleArrival < newAct.getTheoreticalEarliestOperationStartTime() ||
						latestVehicleArrival < nextAct.getTheoreticalEarliestOperationStartTime()){
				return ConstraintsStatus.NOT_FULFILLED_BREAK;
			}
			/*
			 * if the latest operation start-time of new activity is smaller than the earliest start of prev. activity,
			 * then 
			 * 
			 *                    |--- prevAct ---|
			 *  |--- newAct ---|
			 */
			if(newAct.getTheoreticalLatestOperationStartTime() < prevAct.getTheoreticalEarliestOperationStartTime()){
				return ConstraintsStatus.NOT_FULFILLED_BREAK;
			}
			
			/*
			 *  |--- prevAct ---|
			 *                                          |- earliest arrival of vehicle
			 *                       |--- nextAct ---|
			 */
			double arrTimeAtNextOnDirectRouteWithNewVehicle = prevActDepTime + routingCosts.getTransportTime(prevAct.getLocationId(), nextAct.getLocationId(), prevActDepTime, iFacts.getNewDriver(), iFacts.getNewVehicle());
			if(arrTimeAtNextOnDirectRouteWithNewVehicle > nextAct.getTheoreticalLatestOperationStartTime()){
				return ConstraintsStatus.NOT_FULFILLED_BREAK;
			}
			/*
			 *                     |--- newAct ---|
			 *  |--- nextAct ---|
			 */
			if(newAct.getTheoreticalEarliestOperationStartTime() > nextAct.getTheoreticalLatestOperationStartTime()){
				return ConstraintsStatus.NOT_FULFILLED;
			}
			//			log.info("check insertion of " + newAct + " between " + prevAct + " and " + nextAct + ". prevActDepTime=" + prevActDepTime);
			double arrTimeAtNewAct = prevActDepTime + routingCosts.getTransportTime(prevAct.getLocationId(), newAct.getLocationId(), prevActDepTime, iFacts.getNewDriver(), iFacts.getNewVehicle());
			double endTimeAtNewAct = CalculationUtils.getActivityEndTime(arrTimeAtNewAct, newAct);
			double latestArrTimeAtNewAct = getLatestNewActStart(newAct,nextAct,iFacts,latestVehicleArrival,endTimeAtNewAct);
			
//			log.info(newAct + " arrTime=" + arrTimeAtNewAct);
			
			double arrTimeAtNextAct = endTimeAtNewAct + routingCosts.getTransportTime(newAct.getLocationId(), nextAct.getLocationId(), endTimeAtNewAct, iFacts.getNewDriver(), iFacts.getNewVehicle());
			double latestArrTimeAtNextAct = getLatestActStartOfNext(nextAct,iFacts,latestVehicleArrival);
			/*
			 *  |--- newAct ---|
			 *                       		                 |--- vehicle's arrival @nextAct
			 *        latest arrival of vehicle @nextAct ---|                     
			 */
			if(arrTimeAtNextAct > latestVehicleArrival){
				return ConstraintsStatus.NOT_FULFILLED_BREAK;
			}
			if(arrTimeAtNextAct > latestArrTimeAtNextAct){
				return ConstraintsStatus.NOT_FULFILLED;
			}
			/*
			 *  |--- prevAct ---|
			 *                       		                 |--- vehicle's arrival @newAct
			 *        latest arrival of vehicle @newAct ---|                     
			 */
			if(arrTimeAtNewAct > latestVehicleArrival){
				return ConstraintsStatus.NOT_FULFILLED_BREAK;
			}
			if(arrTimeAtNewAct > latestArrTimeAtNewAct){
				return ConstraintsStatus.NOT_FULFILLED;
			}
			
			
//			if vehicle cannot even manage direct-route - break
			if(arrTimeAtNextOnDirectRouteWithNewVehicle > latestArrTimeAtNextAct){
				return ConstraintsStatus.NOT_FULFILLED_BREAK;
			}
			return ConstraintsStatus.FULFILLED;
		}

		private double getCurrentArrTimeAtEnd(JobInsertionContext iFacts) {
			return iFacts.getRoute().getEnd().getArrTime();
		}

		private double getLatestNewActStart(TourActivity newAct, TourActivity nextAct, JobInsertionContext iFacts, double latestVehicleArrival, double departureAtNewAct) {
			double latestNextActStart = getLatestActStartOfNext(nextAct, iFacts, latestVehicleArrival);
			double latestNewAct = latestNextActStart - routingCosts.getTransportTime(newAct.getLocationId(), nextAct.getLocationId(), 
					departureAtNewAct, iFacts.getNewDriver(), iFacts.getNewVehicle());
			return Math.min(newAct.getTheoreticalLatestOperationStartTime(), latestNewAct);
		}

		private double getLatestActStartOfNext(TourActivity nextAct,JobInsertionContext iFacts, double latestVehicleArrival) {
			if(nextAct instanceof End){
				return latestVehicleArrival;
			}
			if(states.getActivityState(nextAct, StateIds.LATEST_ACTIVITY_START, Double.class) != null){
				return states.getActivityState(nextAct, StateIds.LATEST_ACTIVITY_START, Double.class);
			}
			else{
				return nextAct.getTheoreticalLatestOperationStartTime();
			}
		}

		private double getLatestArrAtDepot(JobInsertionContext iFacts, TourActivity newActivity) {
			if(states.getRouteState(iFacts.getRoute(), StateIds.LATEST_ARR_AT_DEPOT, Double.class) != null){
				return states.getRouteState(iFacts.getRoute(), StateIds.LATEST_ARR_AT_DEPOT, Double.class);
			}
			else return Math.min(iFacts.getNewVehicle().getLatestArrival(),getVehicleDeparture(iFacts,newActivity)+maxDriverTime);
		}

		private double getVehicleDeparture(JobInsertionContext iFacts, TourActivity newActivity) {
			double tpTime = routingCosts.getTransportTime(iFacts.getNewVehicle().getStartLocationId(), newActivity.getLocationId(), iFacts.getNewDepTime(), iFacts.getNewDriver(), iFacts.getNewVehicle());
			double newDepTime = newActivity.getTheoreticalEarliestOperationStartTime() - tpTime;
			return Math.max(iFacts.getNewDepTime(),newDepTime);
		}
	}