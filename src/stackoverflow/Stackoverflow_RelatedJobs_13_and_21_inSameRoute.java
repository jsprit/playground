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
import jsprit.core.algorithm.state.StateUpdater;
import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.constraint.ConstraintManager;
import jsprit.core.problem.constraint.SoftRouteConstraint;
import jsprit.core.problem.io.VrpXMLReader;
import jsprit.core.problem.job.Job;
import jsprit.core.problem.misc.JobInsertionContext;
import jsprit.core.problem.solution.SolutionCostCalculator;
import jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import jsprit.core.problem.solution.route.VehicleRoute;
import jsprit.core.problem.solution.route.activity.ActivityVisitor;
import jsprit.core.problem.solution.route.activity.TourActivity;
import jsprit.core.problem.solution.route.activity.TourActivity.JobActivity;
import jsprit.core.problem.solution.route.state.StateFactory;
import jsprit.core.reporting.SolutionPrinter;
import jsprit.core.reporting.SolutionPrinter.Print;
import jsprit.core.util.Solutions;

/**
 * http://stackoverflow.com/questions/24447451/related-jobs-in-jsprit
 * 
 * @author schroeder
 *
 */
public class Stackoverflow_RelatedJobs_13_and_21_inSameRoute {
	
	static class JobsInRouteMemorizer implements StateUpdater, ActivityVisitor {

		private StateManager stateManager;
		private VehicleRoute route;
		
		
		public JobsInRouteMemorizer(StateManager stateManager) {
			super();
			this.stateManager = stateManager;
		}

		@Override
		public void begin(VehicleRoute route) {
			this.route=route;
		}

		@Override
		public void visit(TourActivity activity) {
			if(activity instanceof JobActivity){
				String jobId = ((JobActivity) activity).getJob().getId();
				stateManager.putProblemState(StateFactory.createId(jobId), VehicleRoute.class, this.route);
			}
			
		}

		@Override
		public void finish() {}
		
	}
	
	static class TwentyOneAndThirteenInSameRouteConstraint implements SoftRouteConstraint {

		private StateManager stateManager;
		
		public TwentyOneAndThirteenInSameRouteConstraint(StateManager stateManager) {
			super();
			this.stateManager = stateManager;
		}

		@Override
		public double getCosts(JobInsertionContext insertionContext) {
			if(insertionContext.getJob().getId().equals("13")){
				VehicleRoute route = stateManager.getProblemState(StateFactory.createId("21"), VehicleRoute.class);
				if(route==null){
					return 0.;
				}
				if(route!=null){
					if(route==insertionContext.getRoute()){
						return -100.;
					}
					else return 0.;
				}
			}
			if(insertionContext.getJob().getId().equals("21")){
				VehicleRoute route = stateManager.getProblemState(StateFactory.createId("13"), VehicleRoute.class);
				if(route==null){
					return 0.;
				}
				if(route!=null){
					if(route==insertionContext.getRoute()){
						return -100.;
					}
					else return 0.;
				}
			}
			return 0;
		}
	}
	
	
	
	static class RewardAndPenaltiesThroughSoftConstraints {
		
		private VehicleRoutingProblem vrp;

		public RewardAndPenaltiesThroughSoftConstraints(VehicleRoutingProblem vrp) {
			super();
			this.vrp = vrp;
		}
		
		public double getCosts(VehicleRoute route) {
			boolean serves13 = route.getTourActivities().servesJob(getJob("13"));
			boolean serves21 = route.getTourActivities().servesJob(getJob("21"));
			if(serves13 && serves21) { //then reward
				return -100;
			}
			return 0;
		}
		
		private Job getJob(String string) {
			return vrp.getJobs().get(string);
		}

	}
	
	public static void main(String[] args) {
		 
			VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
			
			/*
			 * A solomonReader reads solomon-instance files, and stores the required information in the builder.
			 */
			new VrpXMLReader(vrpBuilder).read("input/vrpnc1-jsprit.xml");
			
			/*
			 * Finally, the problem can be built. By default, transportCosts are crowFlyDistances (as usually used for vrp-instances).
			 */
			VehicleRoutingProblem vrp = vrpBuilder.build();
			
			final StateManager stateManager = new StateManager(vrp.getTransportCosts());
			stateManager.addStateUpdater(new JobsInRouteMemorizer(stateManager));
			
			ConstraintManager constraintManager = new ConstraintManager(vrp,stateManager);
			constraintManager.addConstraint(new TwentyOneAndThirteenInSameRouteConstraint(stateManager));
			
			final RewardAndPenaltiesThroughSoftConstraints softConstraintContributionToOverallObjective = new RewardAndPenaltiesThroughSoftConstraints(vrp);
			SolutionCostCalculator costCalculator = new SolutionCostCalculator() {
				
				@Override
				public double getCosts(VehicleRoutingProblemSolution solution) {
					double costs = 0.;
					for(VehicleRoute route : solution.getRoutes()){
						costs+=route.getVehicle().getType().getVehicleCostParams().fix;
						costs+=stateManager.getRouteState(route, StateFactory.COSTS, Double.class);
						costs+=softConstraintContributionToOverallObjective.getCosts(route);
					}
					return costs;
				}
				
			};
			
			VehicleRoutingAlgorithmBuilder vraBuilder = new VehicleRoutingAlgorithmBuilder(vrp, "input/rr_ta.xml");
			vraBuilder.addCoreConstraints();
			vraBuilder.setStateAndConstraintManager(stateManager, constraintManager);
			vraBuilder.addDefaultCostCalculators();
			vraBuilder.setObjectiveFunction(costCalculator);
			VehicleRoutingAlgorithm vra = vraBuilder.build();
			
			vra.addListener(new AlgorithmSearchProgressChartListener("output/search"));
			
//			
			Collection<VehicleRoutingProblemSolution> solutions = vra.searchSolutions();
			
			SolutionPrinter.print(vrp, Solutions.bestOf(solutions), Print.VERBOSE);
			
			new Plotter(vrp,Solutions.bestOf(solutions)).setLabel(Label.ID).plot("output/jsprit_vrpnc1_13_21_inSameRoute", "jsprit: 13 and 21 in same route");
	}

}
