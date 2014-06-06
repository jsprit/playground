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

import jsprit.analysis.toolbox.GraphStreamViewer;
import jsprit.analysis.toolbox.GraphStreamViewer.Label;
import jsprit.analysis.toolbox.Plotter;
import jsprit.analysis.toolbox.SolutionPrinter;
import jsprit.analysis.toolbox.SolutionPrinter.Print;
import jsprit.core.algorithm.VehicleRoutingAlgorithm;
import jsprit.core.algorithm.VehicleRoutingAlgorithmBuilder;
import jsprit.core.algorithm.selector.SelectBest;
import jsprit.core.algorithm.state.StateManager;
import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.constraint.ConstraintManager;
import jsprit.core.problem.constraint.ConstraintManager.Priority;
import jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import jsprit.instance.reader.SolomonReader;


public class SolomonExample {
	
	public static void main(String[] args) {
		/*
		 * some preparation - create output folder
		 */
	
		
		/*
		 * Build the problem.
		 * 
		 * But define a problem-builder first.
		 */
		VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
		
		/*
		 * A solomonReader reads solomon-instance files, and stores the required information in the builder.
		 */
		new SolomonReader(vrpBuilder).read("input/sschroeder/C101_solomon.txt");
		
		/*
		 * Finally, the problem can be built. By default, transportCosts are crowFlyDistances (as usually used for vrp-instances).
		 */
		VehicleRoutingProblem problem = vrpBuilder.build();
		
//		new Plotter(vrp).plot("output/solomon_C101.png", "C101");
		
		/*
		 * Define the required vehicle-routing algorithms to solve the above problem.
		 * 
		 * The algorithm can be defined and configured in an xml-file.
		 */
		VehicleRoutingAlgorithmBuilder vraBuilder = new VehicleRoutingAlgorithmBuilder(problem, "input/sschroeder/rr_ta.xml");
		vraBuilder.addDefaultCostCalculators();
		
		/*
		 * The next lines of code are to consider a maximum working time of drivers of 600. time units. Additionally, the algorithm
		 * optimizes departure times at the depot such that waiting times at the first activity of a route are avoided, i.e. the
		 * vehicle starts at the depot such that it just arrives at the earliestOperationStartTime of the first activity, i.e. 
		 * actualDepartureTime = max(earliestStartOfVehicle, firstActivity.earliestOperationStart - tpTime(start,firstActivity)).
		 * If the vehicle departs later, it can also arrive later as long as actualArrival <= latestArrivalOfVehicle holds. 
		 * 
		 * For example:  
		 * 
		 *    |----- vehicle's operation time -------| (i.e. [earliestDeparture,latestArrival]
		 *    
		 *    actDeparture                     actArrival 
		 *        |---- driver's operation time ---|
		 *    
		 */
		
		//set max driver time
		double maxDriverTime = 600.;
		StateManager stateManager = new StateManager(problem.getTransportCosts());
		//adds core load/capacity constraints
		stateManager.updateLoadStates();
		//replaces the default twUpdater and updates departureTime and practical time windows
		stateManager.addStateUpdater(new UpdateDepartureTimeAndPracticalTimeWindows(stateManager, problem.getTransportCosts(), maxDriverTime));
			
		ConstraintManager constraintManager = new ConstraintManager(problem, stateManager);
		constraintManager.addLoadConstraint();
		//replaces the default twConstraint to consider departureTime and driver's working hour as well
		constraintManager.addConstraint(new TimeWindowConstraintWithDriverTime(stateManager, problem.getTransportCosts(), maxDriverTime),Priority.CRITICAL);
		
		vraBuilder.setStateAndConstraintManager(stateManager, constraintManager);
		
		VehicleRoutingAlgorithm algorithm = vraBuilder.build();
		//finally re-schedules the departure time of vehicles to avoid waiting times at first activity
		algorithm.addListener(new DepartureTimeReScheduler());

		
		/*
		 * Solve the problem.
		 * 
		 *
		 */
		Collection<VehicleRoutingProblemSolution> solutions = algorithm.searchSolutions();
		
		/*
		 * Retrieve best solution.
		 */
		VehicleRoutingProblemSolution solution = new SelectBest().selectSolution(solutions);
		
		/*
		 * print solution
		 */
		SolutionPrinter.print(problem,solution,Print.VERBOSE);
		
		/*
		 * Plot solution. 
		 */
		Plotter plotter = new Plotter(problem,solution);
//		plotter.setBoundingBox(30, 0, 50, 20);
		plotter.plot("output/sschroeder/solomon_C101_solution.png", "C101");
//		SolutionPlotter.plotSolutionAsPNG(vrp, solution, "output/solomon_C101_solution.png","C101");
		
//		GraphStream.display(vrp,100);
		
		new GraphStreamViewer(problem,solution).labelWith(Label.ID).setRenderDelay(100).display();
		
	}

}
