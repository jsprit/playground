package abe;

import java.io.IOException;
import java.util.Collection;

import jsprit.analysis.toolbox.AlgorithmSearchProgressChartListener;
import jsprit.analysis.toolbox.Plotter;
import jsprit.analysis.toolbox.Plotter.Label;
import jsprit.core.algorithm.VehicleRoutingAlgorithm;
import jsprit.core.algorithm.io.VehicleRoutingAlgorithms;
import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.cost.AbstractForwardVehicleRoutingTransportCosts;
import jsprit.core.problem.cost.VehicleRoutingTransportCosts;
import jsprit.core.problem.driver.Driver;
import jsprit.core.problem.io.VrpXMLReader;
import jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import jsprit.core.problem.solution.route.VehicleRoute;
import jsprit.core.problem.solution.route.activity.TourActivity;
import jsprit.core.problem.vehicle.Vehicle;
import jsprit.core.reporting.SolutionPrinter;
import jsprit.core.reporting.SolutionPrinter.Print;
import jsprit.core.util.Solutions;
import jsprit.core.util.VehicleRoutingTransportCostsMatrix;

public class AbeProblemFastRelation {
	
	
	public static void main(String[] args) throws IOException {
		VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
		new VrpXMLReader(vrpBuilder).read("/Users/schroeder/Documents/jsprit/abraham/abrahamProblem.xml");
		VehicleRoutingTransportCostsMatrix.Builder matrixBuilder = VehicleRoutingTransportCostsMatrix.Builder.newInstance(true);
		final MatrixReader matrixReader = new MatrixReader(matrixBuilder);
		matrixReader.read("/Users/schroeder/Documents/jsprit/abraham/Matrix.txt");
//		VehicleRoutingTransportCostsMatrix matrix = matrixBuilder.build();
		
		VehicleRoutingTransportCosts costs = new AbstractForwardVehicleRoutingTransportCosts() {
			
			@Override
			public double getTransportTime(String fromId, String toId, double departureTime, Driver driver, Vehicle vehicle) {
				double defaultVelocity = 1;
				//fast connection on relation 17 --> 16
				if(fromId.equals("17") && toId.equals("16")){
					return getDistance(fromId, toId)/100.;
				}
				return getDistance(fromId, toId)/defaultVelocity;
			}
			
			@Override
			public double getTransportCost(String fromId, String toId, double departureTime, Driver driver, Vehicle vehicle) { 
				if(vehicle != null){
					return vehicle.getType().getVehicleCostParams().perDistanceUnit*getDistance(fromId,toId)
							+ vehicle.getType().getVehicleCostParams().perTimeUnit*getTransportTime(fromId, toId, departureTime, driver, vehicle);
				}
				return getDistance(fromId,toId);
			}

			private double getDistance(String fromId, String toId) {
				return matrixReader.getDistance(fromId, toId);
			}
			
		};
		
		vrpBuilder.setRoutingCost(costs);
		VehicleRoutingProblem problem = vrpBuilder.build();
		
		Plotter plotter = new Plotter(problem);
		plotter.setLabel(Label.ID);
		plotter.plot("output/abeProblem.png", "abe");
		
//		VehicleRoutingAlgorithm algo = new SchrimpfFactory().createAlgorithm(problem); 
		VehicleRoutingAlgorithm algo = VehicleRoutingAlgorithms.readAndCreateAlgorithm(problem, "/Users/schroeder/Documents/jsprit/abraham/algorithmConfig_stefan.xml");
		algo.addListener(new AlgorithmSearchProgressChartListener("output/progress.png"));
		Collection<VehicleRoutingProblemSolution> solutions = algo.searchSolutions();
		
		Plotter plotter2 = new Plotter(problem,Solutions.bestOf(solutions));
		plotter2.plot("output/abeProblemFastRelation.png", "abe");
		
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
