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

import jsprit.core.problem.VehicleRoutingProblem;
import jsprit.core.problem.cost.AbstractForwardVehicleRoutingTransportCosts;
import jsprit.core.problem.cost.VehicleRoutingTransportCosts;
import jsprit.core.problem.driver.Driver;
import jsprit.core.problem.vehicle.Vehicle;
import jsprit.core.problem.vehicle.VehicleImpl;
import jsprit.core.problem.vehicle.VehicleType;
import jsprit.core.problem.vehicle.VehicleTypeImpl;
import jsprit.core.problem.vehicle.VehicleTypeImpl.VehicleCostParams;
import jsprit.core.util.VehicleRoutingTransportCostsMatrix;

public class SpeedVariations {
	
	
	
	public static void main(String[] args) {
		
		VehicleRoutingTransportCostsMatrix.Builder matrixBuilder = VehicleRoutingTransportCostsMatrix.Builder.newInstance(true);
		//base time
		matrixBuilder.addTransportTime("from", "to", 100.);
		final VehicleRoutingTransportCostsMatrix matrix = matrixBuilder.build();
		VehicleRoutingTransportCosts modifiedMatrix = new AbstractForwardVehicleRoutingTransportCosts() {
			
			@Override
			public double getTransportTime(String fromId, String toId,double departureTime, Driver driver, Vehicle vehicle) {
				if(vehicle.getType().getTypeId().equals("A")){
					return matrix.getTransportTime(fromId, toId, departureTime, driver, vehicle)*1.1;
				}
				return matrix.getTransportTime(fromId, toId, departureTime, driver, vehicle);
			}
			
			@Override
			public double getTransportCost(String fromId, String toId, double departureTime, Driver driver, Vehicle vehicle) {
				if(vehicle == null) return matrix.getDistance(fromId, toId);
				VehicleCostParams costParams = vehicle.getType().getVehicleCostParams();
				return costParams.perDistanceUnit*matrix.getDistance(fromId, toId) + 
						costParams.perTimeUnit*getTransportTime(fromId, toId, departureTime, driver, vehicle);
			}
		};
		
		VehicleType typeA = VehicleTypeImpl.Builder.newInstance("A").build();
		VehicleType typeB = VehicleTypeImpl.Builder.newInstance("B").build();
		
		Vehicle vehOfTypeA = VehicleImpl.Builder.newInstance("ofTypeA").setType(typeA).setStartLocationId("start").build();
		Vehicle vehOfTypeB = VehicleImpl.Builder.newInstance("ofTypeB").setType(typeB).setStartLocationId("start").build();
		
		System.out.println("tpTime of A: "+ modifiedMatrix.getTransportTime("from", "to", 0., null, vehOfTypeA));
		System.out.println("tpTime of B: "+ modifiedMatrix.getTransportTime("from", "to", 0., null, vehOfTypeB));
		assert modifiedMatrix.getTransportTime("from", "to", 0., null, vehOfTypeA) >= 110.*0.99 && modifiedMatrix.getTransportTime("from", "to", 0., null, vehOfTypeA) <= 110.*1.01 : "110 should be the correct value for vehicle " + vehOfTypeA;
		assert modifiedMatrix.getTransportTime("from", "to", 0., null, vehOfTypeB) == 100. : "100 should be the correct value for vehicle " + vehOfTypeB;
		
		//set modifiedMatrix instead of matrix
		VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
		vrpBuilder.setRoutingCost(modifiedMatrix);
		
		
	}

}
