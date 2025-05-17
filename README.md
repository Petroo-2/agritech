# agritech
import random
import time
from datetime import datetime

class SoilSensor:
    def __init__(self):
        self.moisture = 0
        self.temperature = 0
        self.ph = 0
        
    def read_sensors(self):
        # Simulate sensor readings
        self.moisture = random.uniform(0, 100)  # %
        self.temperature = random.uniform(10, 40)  # °C
        self.ph = random.uniform(3, 9)  # pH
        
        return {
            'timestamp': datetime.now().isoformat(),
            'moisture': round(self.moisture, 2),
            'temperature': round(self.temperature, 2),
            'ph': round(self.ph, 2)
        }

def main():
    sensor = SoilSensor()
    
    while True:
        data = sensor.read_sensors()
        print(f"Soil Data: {data}")
        
        # Add code here to send data to cloud/API
        # upload_to_cloud(data)
        
        time.sleep(300)  # Read every 5 minutes

if __name__ == "__main__":
    main()
    import tensorflow as tf
from tensorflow.keras import layers, models
from tensorflow.keras.preprocessing.image import ImageDataGenerator

def create_model(input_shape, num_classes):
    model = models.Sequential([
        layers.Conv2D(32, (3,3), activation='relu', input_shape=input_shape),
        layers.MaxPooling2D((2,2)),
        layers.Conv2D(64, (3,3), activation='relu'),
        layers.MaxPooling2D((2,2)),
        layers.Conv2D(128, (3,3), activation='relu'),
        layers.MaxPooling2D((2,2)),
        layers.Flatten(),
        layers.Dense(128, activation='relu'),
        layers.Dense(num_classes, activation='softmax')
    ])
    
    model.compile(optimizer='adam',
                 loss='categorical_crossentropy',
                 metrics=['accuracy'])
    
    return model

# Example usage
input_shape = (256, 256, 3)  # RGB images
num_classes = 5  # Number of disease types + healthy
model = create_model(input_shape, num_classes)

# Train with ImageDataGenerator
train_datagen = ImageDataGenerator(rescale=1./255, 
                                  rotation_range=20,
                                  width_shift_range=0.2,
                                  height_shift_range=0.2,
                                  horizontal_flip=True)

train_generator = train_datagen.flow_from_directory(
    'dataset/train',
    target_size=(256, 256),
    batch_size=32,
    class_mode='categorical')

model.fit(train_generator, epochs=10)
#include <DHT.h>
#include <SoilMoistureSensor.h>

#define DHTPIN 2
#define DHTTYPE DHT22
#define MOISTURE_PIN A0
#define RELAY_PIN 3

DHT dht(DHTPIN, DHTTYPE);
SoilMoistureSensor moistureSensor(MOISTURE_PIN);

float moistureThreshold = 40.0; // % below which to irrigate
unsigned long irrigationDuration = 300000; // 5 minutes in ms

void setup() {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Relay off initially
  dht.begin();
}

void loop() {
  float moisture = moistureSensor.read();
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  Serial.print("Moisture: "); Serial.print(moisture); Serial.println("%");
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println("°C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println("%");
  
  if (moisture < moistureThreshold) {
    irrigate();
  }
  
  delay(5000); // Check every 5 seconds
}

void irrigate() {
  Serial.println("Starting irrigation");
  digitalWrite(RELAY_PIN, LOW); // Turn on water pump
  delay(irrigationDuration);
  digitalWrite(RELAY_PIN, HIGH); // Turn off water pump
  Serial.println("Irrigation complete");
}
import React, { useState, useEffect } from 'react';
import { LineChart, Line, BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, Legend } from 'recharts';

const FarmDashboard = () => {
  const [farmData, setFarmData] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Fetch data from API
    const fetchData = async () => {
      try {
        const response = await fetch('/api/farm-data');
        const data = await response.json();
        setFarmData(data);
        setLoading(false);
      } catch (error) {
        console.error('Error fetching data:', error);
      }
    };
    
    fetchData();
    const interval = setInterval(fetchData, 300000); // Refresh every 5 minutes
    
    return () => clearInterval(interval);
  }, []);

  if (loading) return <div>Loading...</div>;

  return (
    <div className="dashboard">
      <h1>Farm Management Dashboard</h1>
      
      <div className="charts">
        <div className="chart-container">
          <h2>Soil Moisture Levels</h2>
          <LineChart width={600} height={300} data={farmData}>
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="timestamp" />
            <YAxis label={{ value: '% Moisture', angle: -90 }} />
            <Tooltip />
            <Legend />
            <Line type="monotone" dataKey="moisture" stroke="#8884d8" />
          </LineChart>
        </div>
        
        <div className="chart-container">
          <h2>Crop Health by Field</h2>
          <BarChart width={600} height={300} data={farmData}>
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis dataKey="field" />
            <YAxis label={{ value: 'Health Score', angle: -90 }} />
            <Tooltip />
            <Legend />
            <Bar dataKey="healthScore" fill="#82ca9d" />
          </BarChart>
        </div>
      </div>
      
      <div className="alerts">
        <h2>Alerts</h2>
        {farmData.filter(item => item.alert).map(alert => (
          <div key={alert.id} className={`alert ${alert.severity}`}>
            {alert.message} - {alert.timestamp}
          </div>
        ))}
      </div>
    </div>
  );
};

export default FarmDashboard;
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt

class FieldMapper:
    def __init__(self, field_boundary):
        self.boundary = np.array(field_boundary)
        self.voronoi = None
        self.path = []
        
    def generate_voronoi(self, points):
        """Generate Voronoi diagram for optimal coverage"""
        self.voronoi = Voronoi(points)
        return self.voronoi
        
    def plan_path(self, resolution=1.0):
        """Generate efficient drone flight path"""
        if not self.voronoi:
            raise ValueError("Generate Voronoi diagram first")
            
        # Simplified path planning - in practice would use more complex algorithm
        min_x, min_y = np.min(self.boundary, axis=0)
        max_x, max_y = np.max(self.boundary, axis=0)
        
        # Generate grid path
        x_coords = np.arange(min_x, max_x, resolution)
        y_coords = np.arange(min_y, max_y, resolution)
        
        path = []
        for i, y in enumerate(y_coords):
            if i % 2 == 0:
                path.extend([(x, y) for x in x_coords])
            else:
                path.extend([(x, y) for x in reversed(x_coords)])
                
        self.path = np.array(path)
        return self.path
        
    def visualize(self):
        """Visualize field, Voronoi diagram, and path"""
        plt.figure(figsize=(10, 10))
        
        # Plot field boundary
        plt.plot(self.boundary[:,0], self.boundary[:,1], 'k-')
        
        # Plot Voronoi diagram if available
        if self.voronoi:
            voronoi_plot_2d(self.voronoi, ax=plt.gca(), show_vertices=False)
            
        # Plot path if available
        if len(self.path) > 0:
            plt.plot(self.path[:,0], self.path[:,1], 'r-', linewidth=2)
            
        plt.title('Field Mapping and Drone Path Planning')
        plt.xlabel('X coordinate (m)')
        plt.ylabel('Y coordinate (m)')
        plt.grid(True)
        plt.axis('equal')
        plt.show()

# Example usage
field_boundary = [(0,0), (100,0), (100,50), (50,100), (0,50)]
points = np.random.rand(10, 2) * 100  # Random points for Voronoi

mapper = FieldMapper(field_boundary)
mapper.generate_voronoi(points)
mapper.plan_path(resolution=5.0)
mapper.visualize()
