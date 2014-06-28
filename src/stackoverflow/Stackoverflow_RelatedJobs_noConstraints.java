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

package stackoverflow;

import java.util.Collection;

import jsprit.analysis.toolbox.AlgorithmSearchProgressChartListener;
import jsprit.analysis.toolbox.Plotter;
import jsprit.analysis.toolbox.Plotter.Label;
import jsprit.core.algorithm.VehicleRoutingAlgorithm;
import jsprit.core.algorithm.VehicleRoutingAlgorithmBuilder;
import jsprit.core.algorithm.state.StateManager;
import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.constraint.ConstraintManager;
import jsprit.core.problem.io.VrpXMLReader;
import jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import jsprit.core.reporting.SolutionPrinter;
import jsprit.core.reporting.SolutionPrinter.Print;
import jsprit.core.util.Solutions;

/**
 * http://stackoverflow.com/questions/24447451/related-jobs-in-jsprit
 * 
 * @author schroeder
 *
 */
public class Stackoverflow_RelatedJobs_noConstraints {
	
	
	public static void main(String[] args) {
		 
			VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
			
			/*
			 * A solomonReader reads solomon-instance files, and stores the required information in the builder.
			 */
			new VrpXMLReader(vrpBuilder).read("input/stackoverflow/vrpnc1-jsprit.xml");
			
			/*
			 * Finally, the problem can be built. By default, transportCosts are crowFlyDistances (as usually used for vrp-instances).
			 */
			VehicleRoutingProblem vrp = vrpBuilder.build();
			
			final StateManager stateManager = new StateManager(vrp.getTransportCosts());
			
			ConstraintManager constraintManager = new ConstraintManager(vrp,stateManager);
			
			VehicleRoutingAlgorithmBuilder vraBuilder = new VehicleRoutingAlgorithmBuilder(vrp, "input/stackoverflow/rr_ta.xml");
			vraBuilder.setStateAndConstraintManager(stateManager, constraintManager);
			vraBuilder.addDefaultCostCalculators();
			VehicleRoutingAlgorithm vra = vraBuilder.build();
			
			vra.addListener(new AlgorithmSearchProgressChartListener("output/search"));
			
//			
			Collection<VehicleRoutingProblemSolution> solutions = vra.searchSolutions();
			
			SolutionPrinter.print(vrp, Solutions.bestOf(solutions), Print.VERBOSE);
			
			new Plotter(vrp,Solutions.bestOf(solutions)).setLabel(Label.ID).plot("output/jsprit_vrpnc1_noConstraints", "jsprit: noConstraints");
	}

}
