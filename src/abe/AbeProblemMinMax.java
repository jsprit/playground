package abe;

import java.io.IOException;
import java.util.Collection;

import jsprit.analysis.toolbox.AlgorithmSearchProgressChartListener;
import jsprit.analysis.toolbox.Plotter;
import jsprit.core.algorithm.VehicleRoutingAlgorithm;
import jsprit.core.algorithm.VehicleRoutingAlgorithmBuilder;
import jsprit.core.algorithm.recreate.VariableTransportCostCalculator;
import jsprit.core.algorithm.state.StateManager;
import jsprit.core.algorithm.state.StateUpdater;
import jsprit.core.algorithm.termination.VariationCoefficientTermination;
import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.constraint.ConstraintManager;
import jsprit.core.problem.constraint.SoftActivityConstraint;
import jsprit.core.problem.cost.ForwardTransportTime;
import jsprit.core.problem.cost.VehicleRoutingTransportCosts;
import jsprit.core.problem.io.VrpXMLReader;
import jsprit.core.problem.misc.JobInsertionContext;
import jsprit.core.problem.solution.SolutionCostCalculator;
import jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import jsprit.core.problem.solution.route.VehicleRoute;
import jsprit.core.problem.solution.route.activity.ActivityVisitor;
import jsprit.core.problem.solution.route.activity.End;
import jsprit.core.problem.solution.route.activity.TourActivity;
import jsprit.core.problem.solution.route.state.StateFactory;
import jsprit.core.problem.solution.route.state.StateFactory.StateId;
import jsprit.core.reporting.SolutionPrinter;
import jsprit.core.reporting.SolutionPrinter.Print;
import jsprit.core.util.ActivityTimeTracker;
import jsprit.core.util.CalculationUtils;
import jsprit.core.util.Solutions;
import jsprit.core.util.VehicleRoutingTransportCostsMatrix;

public class AbeProblemMinMax {
	
	/*
	 * This updates the state "max-transport-time" which is introduced below. Once either the insertion procedure starts or a job has
	 * been inserted, UpdateMaxTransportTime is called for the route that has been changed.
	 * 
	 * It must not only be an ActivityVisitor which indicates that the update procedure starts at the beginning of route all the way to end 
	 * (in contrary to the ReverseActivityVisitor) but also be a StateUpdater which is just a marker to register it in the StateManager.
	 * 
	 * You do not need to declare this as static inner class. You can just choose your preferred approach. However, be aware
	 * that this requires the stateName "max-transport-time" you define below. If you choose to define this as class in a new file, 
	 * you might define "max-transport-time" as static id in another file, to make sure you do not have type errors etc..
	 */
	static class UpdateMaxTransportTime implements ActivityVisitor, StateUpdater {

		private StateManager stateManager;
		
		private ActivityTimeTracker timeTracker;
		
		public UpdateMaxTransportTime(StateManager stateManager, ForwardTransportTime transportTime) {
			super();
			this.stateManager = stateManager;
			this.timeTracker = new ActivityTimeTracker(transportTime);
		}
		
		@Override
		public void begin(VehicleRoute route) {
			timeTracker.begin(route);
		}

		@Override
		public void visit(TourActivity activity) {
			timeTracker.visit(activity);
		}

		@Override
		public void finish() {
			timeTracker.finish();
			double newRouteEndTime = timeTracker.getActArrTime();
			double currentMaxTransportTime = stateManager.getProblemState(StateFactory.createId("max-transport-time"), Double.class);
			if(newRouteEndTime > currentMaxTransportTime){
				stateManager.putProblemState(StateFactory.createId("max-transport-time"), Double.class, newRouteEndTime);
			}
		}
		
	}
	
	public static void main(String[] args) throws IOException {
		VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
		new VrpXMLReader(vrpBuilder).read("input/abe/abrahamProblem.xml");
		VehicleRoutingTransportCostsMatrix.Builder matrixBuilder = VehicleRoutingTransportCostsMatrix.Builder.newInstance(true);
		final MatrixReader matrixReader = new MatrixReader(matrixBuilder);
		matrixReader.read("input/abe/Matrix.txt");
		VehicleRoutingTransportCostsMatrix matrix = matrixBuilder.build();
		vrpBuilder.setRoutingCost(matrix);

		final VehicleRoutingProblem problem = vrpBuilder.build();

		/*
		 * Your custom objective function that min max transport times. Additionally you can try to consider overall transport times
		 * in your objective as well. Thus you minimize max transport times first, and second, you minimize overall transport time. 
		 * 
		 * If you choose to consider overall transport times, makes sure you scale it appropriately.
		 */
		SolutionCostCalculator objectiveFunction = new SolutionCostCalculator() {
			
			private double scalingParameter = 0.2;
			
			@Override
			public double getCosts(VehicleRoutingProblemSolution solution) {
				double maxTransportTime = 0.;
				double sumTransportTimes = 0.;
				for(VehicleRoute route : solution.getRoutes()){
					double tpTime = route.getEnd().getArrTime() - route.getStart().getEndTime();
					sumTransportTimes+=tpTime;
					if(tpTime > maxTransportTime){
						maxTransportTime = tpTime; 
					}
				}
				return maxTransportTime + scalingParameter*sumTransportTimes;
			}
		};
		
		final StateManager stateManager = new StateManager(problem.getTransportCosts());
		//introduce a new state called "max-transport-time" 
		StateId max_transport_time_state = StateFactory.createId("max-transport-time");
		//add a default-state for "max-transport-time"
		stateManager.addDefaultProblemState(max_transport_time_state, Double.class, 0.);
		//
		stateManager.addStateUpdater(new UpdateMaxTransportTime(stateManager,problem.getTransportCosts()));
		
		/*
		 * The insertion heuristics is controlled with your constraints
		 */
		ConstraintManager constraintManager = new ConstraintManager(problem, stateManager);
		// soft constraint that calculates additional transport costs when inserting a job(activity) at specified position
		constraintManager.addConstraint(new VariableTransportCostCalculator(problem.getTransportCosts()));
		/*
		 *  soft constraint that penalyzes a shift of max-route transport time, i.e. once the insertion heuristic
		 *  tries to insert a jobActivity at position which results in a shift of max-transport-time, it is penalyzed with 
		 *  penaltyForEachTimeUnitAboveCurrentMaxTime
		 *  
		 */
		SoftActivityConstraint penalyzeShiftOfMaxTransportTime = new SoftActivityConstraint() {
			private final VehicleRoutingTransportCosts routingCosts = problem.getTransportCosts();
			
			private final double penaltyForEachTimeUnitAboveCurrentMaxTime = 3.;
			
			@Override
			public double getCosts(JobInsertionContext iFacts, TourActivity prevAct, TourActivity newAct, TourActivity nextAct, double depTimeAtPrevAct) {
				/*
				 * determines maximum of all routes' transport times, which is here basically a state that can be fetched via the stateManager
				 */
				double maxTime = stateManager.getProblemState(StateFactory.createId("max-transport-time"), Double.class);
				/*
				 * determines additional time of route when inserting newAct between prevAct and nextAct
				 * 
				 */
				double tp_time_prevAct_newAct = routingCosts.getTransportTime(prevAct.getLocationId(), newAct.getLocationId(), depTimeAtPrevAct, iFacts.getNewDriver(), iFacts.getNewVehicle());
				double newAct_arrTime = depTimeAtPrevAct + tp_time_prevAct_newAct;
				double newAct_endTime = CalculationUtils.getActivityEndTime(newAct_arrTime, newAct);
				/*
				 * open routes - if route is set to be open, i.e. end is endogeneously determined by the algorithm, then inserting of newAct between prevAct
				 * and end just shifts the route's end time to by tp_time_prevAct_newAct 
				 * 
				 */
				if(nextAct instanceof End){
					if(!iFacts.getNewVehicle().isReturnToDepot()){
						double additionalTime = tp_time_prevAct_newAct;
						double new_routes_transport_time = iFacts.getRoute().getEnd().getArrTime() - iFacts.getRoute().getStart().getEndTime() + additionalTime;
						return penaltyForEachTimeUnitAboveCurrentMaxTime*Math.max(0,new_routes_transport_time-maxTime);
					}
				}
				double tp_time_newAct_nextAct = routingCosts.getTransportTime(newAct.getLocationId(), nextAct.getLocationId(), newAct_endTime, iFacts.getNewDriver(), iFacts.getNewVehicle());
				double nextAct_arrTime = newAct_endTime + tp_time_newAct_nextAct;
				double oldTime;
				if(iFacts.getRoute().isEmpty()){
					oldTime = (nextAct.getArrTime() - depTimeAtPrevAct);
				}
				else{
					oldTime = (nextAct.getArrTime() - iFacts.getRoute().getDepartureTime());
				}
				double additionalTime = (nextAct_arrTime - iFacts.getNewDepTime()) - oldTime;				
				double tpTime = iFacts.getRoute().getEnd().getArrTime() - iFacts.getRoute().getStart().getEndTime() + additionalTime;
				
				return penaltyForEachTimeUnitAboveCurrentMaxTime*Math.max(0,tpTime-maxTime);
				
			}
		};
		constraintManager.addConstraint(penalyzeShiftOfMaxTransportTime);
		
		VehicleRoutingAlgorithmBuilder algorithmBuilder = new VehicleRoutingAlgorithmBuilder(problem, "/Users/schroeder/Documents/jsprit/abraham/algorithmConfig_stefan.xml");
		algorithmBuilder.setObjectiveFunction(objectiveFunction);
		algorithmBuilder.setStateManager(stateManager);
		algorithmBuilder.setStateAndConstraintManager(stateManager, constraintManager);
		algorithmBuilder.addCoreConstraints();

		
		VehicleRoutingAlgorithm algo = algorithmBuilder.build();

		algo.addListener(new AlgorithmSearchProgressChartListener("output/abe/progress.png"));
		VariationCoefficientTermination prematureAlgorithmTermination = new VariationCoefficientTermination(150, 0.001);
		algo.addListener(prematureAlgorithmTermination);
		algo.setPrematureAlgorithmTermination(prematureAlgorithmTermination);
		
		Collection<VehicleRoutingProblemSolution> solutions = algo.searchSolutions();
		
		Plotter plotter2 = new Plotter(problem,Solutions.bestOf(solutions));
		plotter2.setShowFirstActivity(true);
		plotter2.plot("output/abe/abeProblemWithSolution.png", "abe");
		
		SolutionPrinter.print(problem, Solutions.bestOf(solutions), Print.VERBOSE);
		
		System.out.println("total-time: " + getTotalTime(problem, Solutions.bestOf(solutions)));
		System.out.println("total-distance: " + getTotalDistance(matrixReader, Solutions.bestOf(solutions)));
		
	}

	private static double getTotalDistance(MatrixReader matrix,VehicleRoutingProblemSolution bestOf) {
		double dist = 0.0;
		for(VehicleRoute r : bestOf.getRoutes()){
			TourActivity last = r.getStart();
			for(TourActivity act : r.getActivities()){
				dist += matrix.getDistance(last.getLocationId(), act.getLocationId());
				last=act;
			}
			dist+=matrix.getDistance(last.getLocationId(), r.getEnd().getLocationId());
		}
		return dist;
	}

	private static double getTotalTime(VehicleRoutingProblem problem,VehicleRoutingProblemSolution bestOf) {
		double time = 0.0;
		for(VehicleRoute r : bestOf.getRoutes()) time+=r.getEnd().getArrTime();
		return time;
	}

}
