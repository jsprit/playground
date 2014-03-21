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
package abe;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.Random;

import jsprit.core.algorithm.VehicleRoutingAlgorithm;
import jsprit.core.algorithm.io.VehicleRoutingAlgorithms;
import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.VehicleRoutingProblem.FleetSize;
import jsprit.core.problem.io.VrpXMLReader;
import jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import jsprit.core.problem.solution.route.VehicleRoute;
import jsprit.core.problem.solution.route.activity.TourActivity;
import jsprit.core.problem.vehicle.Vehicle;
import jsprit.core.problem.vehicle.VehicleImpl;
import jsprit.core.problem.vehicle.VehicleTypeImpl;
import jsprit.core.util.Coordinate;
import jsprit.core.util.EuclideanDistanceCalculator;
import jsprit.core.util.Solutions;
import jsprit.core.util.VehicleRoutingTransportCostsMatrix;


public class MultipleDepotTimeCostSensitivity {

	/**
	 * @param args
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException {
		
		VehicleRoutingTransportCostsMatrix.Builder matrixBuilder = VehicleRoutingTransportCostsMatrix.Builder.newInstance(false);
		boolean matrixSet = false;
		
		BufferedWriter writer = new BufferedWriter(new FileWriter(new File("output/costOfTimeSensitivity.txt")));
		
//		double costPerTime = 1.0;
		writer.write("costPerTimeUnit;run;costs;time;distance\n");
		
		for(double costPerTime = 0.0; costPerTime <= 10.; costPerTime += 0.1){
			for(int run = 0; run < 3; run++){
				final VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
				/*
				 * Read cordeau-instance p01, BUT only its services without any vehicles 
				 */
				new VrpXMLReader(vrpBuilder).read("input/vrp_cordeau_01.xml");

				/*
				 * add vehicles with its depots
				 * 4 depots:
				 * (20,20)
				 * (30,40)
				 * (50,30)
				 * (60,50)
				 * 
				 * each with 4 vehicles each with a capacity of 80
				 */
				int nuOfVehicles = 4;
				int capacity = 80;
				Coordinate firstDepotCoord = Coordinate.newInstance(20, 20);
				Coordinate second = Coordinate.newInstance(30, 40);
				Coordinate third = Coordinate.newInstance(50, 30);
				Coordinate fourth = Coordinate.newInstance(60, 50);

				double costPerDistance = 1.0;


				VehicleTypeImpl vehicleType = VehicleTypeImpl.Builder.newInstance("type").addCapacityDimension(0, capacity)
						.setCostPerDistance(costPerDistance).setCostPerTime(costPerTime).build();

				int depotCounter = 1;
				for(Coordinate depotCoord : Arrays.asList(firstDepotCoord,second,third,fourth)){
					for(int i=0;i<nuOfVehicles;i++){
						Vehicle vehicle = VehicleImpl.Builder.newInstance(depotCounter + "_" + (i+1) + "_vehicle").
								setStartLocationCoordinate(depotCoord).setStartLocationId(""+(100+depotCounter)).setType(vehicleType).build();
						vrpBuilder.addVehicle(vehicle);
					}
					depotCounter++;
				}

				/*
				 * define problem with finite fleet
				 */
				vrpBuilder.setFleetSize(FleetSize.FINITE);

				if(!matrixSet){
					System.out.println("construct matrix");
					//construct artificial transportTime- and transportCostMatrix
					Random randomNumberGenerator = new Random(Long.MAX_VALUE);
					double probSpeedupRelation = 0.5; 
					double percentSpeedup = 0.5;

					for(String fromLocation : vrpBuilder.getLocationMap().keySet()){
						for(String toLocation : vrpBuilder.getLocationMap().keySet()){
							double distance = getDistance(vrpBuilder, fromLocation, toLocation);
							matrixBuilder.addTransportDistance(fromLocation, toLocation, distance);
							boolean speedupRelation = randomNumberGenerator.nextDouble() < probSpeedupRelation;
							if(speedupRelation) matrixBuilder.addTransportTime(fromLocation, toLocation, (distance-distance*percentSpeedup));
							else matrixBuilder.addTransportTime(fromLocation, toLocation, distance);
						}
					}
					matrixSet = true;
				}
				vrpBuilder.setRoutingCost(matrixBuilder.build());

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
				VehicleRoutingAlgorithm vra = VehicleRoutingAlgorithms.readAndCreateAlgorithm(vrp, "/Users/schroeder/Documents/jsprit/abraham/algorithmConfig_stefan.xml");
				//		vra.getAlgorithmListeners().addListener(new StopWatch(),Priority.HIGH);
				//		vra.getAlgorithmListeners().addListener(new AlgorithmSearchProgressChartListener("output/progress.png"));
//				VariationCoefficientTermination prematureTermination = new VariationCoefficientTermination(100, 0.01);
//				vra.addListener(prematureTermination);
//				vra.setPrematureAlgorithmTermination(prematureTermination);

				Collection<VehicleRoutingProblemSolution> solutions = vra.searchSolutions();

				//		SolutionPrinter.print(Solutions.bestOf(solutions));

				//		new Plotter(vrp, Solutions.bestOf(solutions)).plot("output/p01_solution.png", "p01");

				writer.write(costPerTime + ";" + run + ";" + Solutions.bestOf(solutions).getCost() + ";" + getTotalTime(vrp, Solutions.bestOf(solutions)) + ";" + getTotalDistance(vrpBuilder, Solutions.bestOf(solutions)) + "\n");
				//			System.out.println("total-time: " + getTotalTime(vrp, Solutions.bestOf(solutions)));
				//			System.out.println("total-distance: " + );
			}
		}
		writer.close();
		
	}

	private static double getDistance(final VehicleRoutingProblem.Builder vrpBuilder,String fromLocation, String toLocation) {
		return EuclideanDistanceCalculator.calculateDistance(vrpBuilder.getLocationMap().get(fromLocation), vrpBuilder.getLocationMap().get(toLocation));
	}
	
	private static double getTotalDistance(final VehicleRoutingProblem.Builder vrpBuilder, VehicleRoutingProblemSolution bestOf) {
		double dist = 0.0;
		for(VehicleRoute r : bestOf.getRoutes()){
			TourActivity last = r.getStart();
			for(TourActivity act : r.getActivities()){
				dist += getDistance(vrpBuilder, last.getLocationId(), act.getLocationId());
				last=act;
			}
//			dist+=matrix.getDistance(last.getLocationId(), r.getEnd().getLocationId());
			dist+=getDistance(vrpBuilder, last.getLocationId(), r.getEnd().getLocationId());
		}
		return dist;
	}

	private static double getTotalTime(VehicleRoutingProblem problem,VehicleRoutingProblemSolution bestOf) {
		double time = 0.0;
		for(VehicleRoute r : bestOf.getRoutes()) time+=r.getEnd().getArrTime();
		return time;
	}

}
