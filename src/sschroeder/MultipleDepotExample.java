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

import jsprit.analysis.toolbox.AlgorithmSearchProgressChartListener;
import jsprit.analysis.toolbox.Plotter;
import jsprit.analysis.toolbox.SolutionPrinter;
import jsprit.analysis.toolbox.SolutionPrinter.Print;
import jsprit.core.algorithm.VehicleRoutingAlgorithm;
import jsprit.core.algorithm.VehicleRoutingAlgorithmBuilder;
import jsprit.core.algorithm.state.StateManager;
import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.constraint.ConstraintManager;
import jsprit.core.problem.constraint.ConstraintManager.Priority;
import jsprit.core.problem.io.VrpXMLReader;
import jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import jsprit.core.util.Solutions;


public class MultipleDepotExample {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		/*
		 * some preparation - create output folder
		 */

		
		VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
		/*
		 * Read cordeau-instance p01, BUT only its services without any vehicles 
		 */
		new VrpXMLReader(vrpBuilder).read("input/sschroeder/cordeau_p01.xml");
		
		/*
		 * build the problem
		 */
		VehicleRoutingProblem vrp = vrpBuilder.build();
		
		/*
		 * plot to see how the problem looks like
		 */
//		SolutionPlotter.plotVrpAsPNG(vrp, "output/problem01.png", "p01");

		/*
		 * solve the problem
		 */
		VehicleRoutingAlgorithmBuilder vraBuilder = new VehicleRoutingAlgorithmBuilder(vrp, "input/sschroeder/rr_ta.xml");
		vraBuilder.addDefaultCostCalculators();
		
		double maxDriverTime = 70.;
		StateManager stateManager = new StateManager(vrp.getTransportCosts());
		stateManager.updateLoadStates();
//		stateManager.updateTimeWindowStates();
		stateManager.addStateUpdater(new UpdateDepartureTimeAndPracticalTimeWindows(stateManager, vrp.getTransportCosts(), maxDriverTime));
			
		ConstraintManager constraintManager = new ConstraintManager(vrp, stateManager);
		constraintManager.addLoadConstraint();
//		constraintManager.addTimeWindowConstraint();
		constraintManager.addConstraint(new TimeWindowConstraintWithDriverTime(stateManager, vrp.getTransportCosts(), maxDriverTime),Priority.CRITICAL);
		
		vraBuilder.setStateAndConstraintManager(stateManager, constraintManager);
		
		VehicleRoutingAlgorithm algorithm = vraBuilder.build();
		algorithm.setNuOfIterations(2000);
		algorithm.addListener(new DepartureTimeReScheduler());
		
		algorithm.getAlgorithmListeners().addListener(new AlgorithmSearchProgressChartListener("output/sschroeder/progress.png"));
		Collection<VehicleRoutingProblemSolution> solutions = algorithm.searchSolutions();
		
		SolutionPrinter.print(vrp,Solutions.bestOf(solutions),Print.VERBOSE);
		
		new Plotter(vrp, Solutions.bestOf(solutions)).plot("output/sschroeder/cordeau_p01_comp.png", "p01");
		
//		new GraphStreamViewer(vrp, Solutions.bestOf(solutions)).setRenderDelay(100).display();
		
	}

}
