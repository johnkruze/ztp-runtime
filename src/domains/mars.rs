// Mars EDL Physics solver: 0.38g, CO2 atmosphere, atmospheric drag, and retro-propulsion thrust.
// CREED: The vehicle is a physical body matching Martian gravity and atmospheric density profiles.

pub struct MarsPhysics {
    pub gravity: f64,
}

impl Default for MarsPhysics {
    fn default() -> Self {
        Self { gravity: 3.721 } // Mars gravity m/s^2
    }
}

impl MarsPhysics {
    /// Density: rho = 0.020 * exp(-0.00009 * altitude_m)
    pub fn atmosphere_density(&self, altitude: f64) -> f64 {
        if altitude < 0.0 { return 0.020; }
        0.020 * (-0.00009 * altitude).exp()
    }

    /// Calculate 3D drag force vector opposite to velocity
    pub fn calculate_drag(
        &self,
        velocity: [f64; 3],
        altitude: f64,
        area: f64,
        cd: f64,
    ) -> [f64; 3] {
        let speed = (velocity[0].powi(2) + velocity[1].powi(2) + velocity[2].powi(2)).sqrt();
        if speed < 1e-6 {
            return [0.0, 0.0, 0.0];
        }
        let rho = self.atmosphere_density(altitude);
        let drag_mag = 0.5 * rho * speed.powi(2) * cd * area;
        [
            -drag_mag * (velocity[0] / speed),
            -drag_mag * (velocity[1] / speed),
            -drag_mag * (velocity[2] / speed),
        ]
    }
}

pub struct EdlVehicle {
    pub dry_mass: f64,
    pub drag_area: f64,
    pub cd: f64,
    pub fuel_mass: f64,
    pub specific_impulse: f64, // Isp in seconds (e.g. 290s)
}

impl Default for EdlVehicle {
    fn default() -> Self {
        Self {
            dry_mass: 1000.0,
            drag_area: 10.0,
            cd: 1.2,
            fuel_mass: 400.0,
            specific_impulse: 290.0,
        }
    }
}

impl EdlVehicle {
    pub fn total_mass(&self) -> f64 {
        self.dry_mass + self.fuel_mass
    }

    /// Integrates the spacecraft position, velocity, and fuel mass over dt under drag, gravity, and thrust.
    /// retro_thrust: thrust applied in Newtons (directed opposite to velocity to slow down).
    pub fn step(
        &mut self,
        position: &mut [f64; 3],
        velocity: &mut [f64; 3],
        retro_thrust: f64,
        dt: f64,
    ) -> (f64, [f64; 3], [f64; 3]) {
        let physics = MarsPhysics::default();
        let total_m = self.total_mass();
        
        // 1. Calculate Drag force
        let drag_force = physics.calculate_drag(*velocity, position[2], self.drag_area, self.cd);
        
        // 2. Calculate Gravity force (acts downwards on Z axis)
        let gravity_force = -total_m * physics.gravity;
        
        let mut net_force = drag_force;
        net_force[2] += gravity_force;

        // 3. Calculate Thruster force (opposite to velocity direction)
        let speed = (velocity[0].powi(2) + velocity[1].powi(2) + velocity[2].powi(2)).sqrt();
        let mut actual_thrust = 0.0;
        
        if retro_thrust > 0.0 && self.fuel_mass > 0.0 {
            // Apply thrust up to fuel limit
            let g0 = 9.80665;
            let m_dot = retro_thrust / (self.specific_impulse * g0);
            let fuel_burned = (m_dot * dt).min(self.fuel_mass);
            
            if fuel_burned > 0.0 {
                actual_thrust = retro_thrust * (fuel_burned / (m_dot * dt));
                self.fuel_mass -= fuel_burned;
                
                if speed > 2.0 {
                    net_force[0] += -actual_thrust * (velocity[0] / speed);
                    net_force[1] += -actual_thrust * (velocity[1] / speed);
                    net_force[2] += -actual_thrust * (velocity[2] / speed);
                } else {
                    // Controlled descent to touchdown: target vz = -1.5 m/s
                    let target_vz = -1.5;
                    let vz_error = velocity[2] - target_vz;
                    let vert_thrust = (total_m * physics.gravity + vz_error * total_m * 5.0)
                        .clamp(0.0, actual_thrust);
                    net_force[2] += vert_thrust;

                    // Reduce horizontal velocity to zero
                    let h_speed = (velocity[0] * velocity[0] + velocity[1] * velocity[1]).sqrt();
                    if h_speed > 0.01 {
                        let remaining_thrust = (actual_thrust - vert_thrust).max(0.0);
                        if remaining_thrust > 0.0 {
                            net_force[0] += -remaining_thrust * (velocity[0] / h_speed);
                            net_force[1] += -remaining_thrust * (velocity[1] / h_speed);
                        }
                    }
                }
            }
        }

        // 4. Euler Integration
        let ax = net_force[0] / total_m;
        let ay = net_force[1] / total_m;
        let az = net_force[2] / total_m;

        velocity[0] += ax * dt;
        velocity[1] += ay * dt;
        velocity[2] += az * dt;

        position[0] += velocity[0] * dt;
        position[1] += velocity[1] * dt;
        position[2] += velocity[2] * dt;

        // Ensure altitude doesn't go negative
        if position[2] < 0.0 {
            position[2] = 0.0;
            velocity[0] = 0.0;
            velocity[1] = 0.0;
            velocity[2] = 0.0;
        }

        let density = physics.atmosphere_density(position[2]);
        (density, drag_force, [ax, ay, az])
    }
}
