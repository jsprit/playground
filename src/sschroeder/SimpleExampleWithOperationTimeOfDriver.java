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

import jsprit.core.algorithm.VehicleRoutingAlgorithm;
import jsprit.core.algorithm.VehicleRoutingAlgorithmBuilder;
import jsprit.core.algorithm.state.StateManager;
import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.constraint.ConstraintManager;
import jsprit.core.problem.constraint.ConstraintManager.Priority;
import jsprit.core.problem.job.Service;
import jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import jsprit.core.problem.solution.route.activity.TimeWindow;
import jsprit.core.problem.vehicle.Vehicle;
import jsprit.core.problem.vehicle.VehicleImpl;
import jsprit.core.problem.vehicle.VehicleImpl.Builder;
import jsprit.core.problem.vehicle.VehicleType;
import jsprit.core.problem.vehicle.VehicleTypeImpl;
import jsprit.core.reporting.SolutionPrinter;
import jsprit.core.reporting.SolutionPrinter.Print;
import jsprit.core.util.Coordinate;
import jsprit.core.util.Solutions;

public class SimpleExampleWithOperationTimeOfDriver {
	
	public static void main(String[] args) {
		
		/*
		 * get a vehicle type-builder and build a type with the typeId "vehicleType" and one capacity dimension, i.e. weight, and capacity dimension value of 2
		 */
		final int WEIGHT_INDEX = 0;
		VehicleTypeImpl.Builder vehicleTypeBuilder = VehicleTypeImpl.Builder.newInstance("vehicleType").addCapacityDimension(WEIGHT_INDEX, 2);
		VehicleType vehicleType = vehicleTypeBuilder.build();
		
		/*
		 * get a vehicle-builder and build a vehicle located at (10,10) with type "vehicleType"
		 */
		Builder vehicleBuilder = VehicleImpl.Builder.newInstance("vehicle");
		vehicleBuilder.setStartLocationCoordinate(Coordinate.newInstance(0, 0));
		vehicleBuilder.setType(vehicleType);
		vehicleBuilder.setLatestArrival(35.);
		Vehicle vehicle = vehicleBuilder.build();
		
		/*
		 * build services at the required locations, each with a capacity-demand of 1.
		 */
		
		Service service1 = Service.Builder.newInstance("1").addSizeDimension(WEIGHT_INDEX, 1).setCoord(Coordinate.newInstance(10, 0))
				.setTimeWindow(TimeWindow.newInstance(15., 30.)).setServiceTime(1.).build();
		Service service2 = Service.Builder.newInstance("2").setServiceTime(1.).addSizeDimension(WEIGHT_INDEX, 1).setCoord(Coordinate.newInstance(10, 0))
				.setTimeWindow(TimeWindow.newInstance(15.,30.)).build();
		
		
		VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
		vrpBuilder.addVehicle(vehicle);
		vrpBuilder.addJob(service1).addJob(service2);
		
		VehicleRoutingProblem problem = vrpBuilder.build();
		
		/*
		 * get the algorithm out-of-the-box. 
		 */
		VehicleRoutingAlgorithmBuilder vraBuilder = new VehicleRoutingAlgorithmBuilder(problem, "input/sschroeder/rr_ta.xml");
		vraBuilder.addDefaultCostCalculators();
		
		double maxDriverTime = 21.;
		StateManager stateManager = new StateManager(problem.getTransportCosts());
		stateManager.updateLoadStates();
		stateManager.addStateUpdater(new UpdateDepartureTimeAndPracticalTimeWindows(stateManager, problem.getTransportCosts(), maxDriverTime));
			
		ConstraintManager constraintManager = new ConstraintManager(problem, stateManager);
		constraintManager.addLoadConstraint();
		constraintManager.addConstraint(new TimeWindowConstraintWithDriverTime(stateManager, problem.getTransportCosts(), maxDriverTime),Priority.CRITICAL);
		
		vraBuilder.setStateAndConstraintManager(stateManager, constraintManager);
		
		VehicleRoutingAlgorithm algorithm = vraBuilder.build();
		algorithm.addListener(new DepartureTimeReScheduler());
		
		/*
		 * and search a solution
		 */
		Collection<VehicleRoutingProblemSolution> solutions = algorithm.searchSolutions();
		
		/*
		 * get the best 
		 */
		VehicleRoutingProblemSolution bestSolution = Solutions.bestOf(solutions);
		
//		new VrpXMLWriter(problem, solutions).write("output/problem-with-solution.xml");
		
		SolutionPrinter.print(problem,bestSolution,Print.VERBOSE);
		
		
	}

}
