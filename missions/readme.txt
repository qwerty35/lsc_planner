{
  "quadrotors": {
    "crazyflie": {"max_vel": [1.0, 1.0, 1.0], # Maximum velocity of the crazyflie [v_min_x, v_min_y, v_min_z], [m/s]
                  "max_acc": [2.0, 2.0, 2.0], # Maximum acceleration of the crazyflie [a_min_x, a_min_y, a_min_z], [m/s^2]
                  "radius": 0.15, # Radius of the crazyflie, [m]
                  "nominal_velocity": 1.0, # Not used in this work
                  "downwash": 2.0}, # Downwash coefficient of the crazyflie
    "default": {"max_vel": [1.0, 1.0, 1.0],
                "max_acc": [2.0, 2.0, 2.0],
                "radius": 0.15,
                "nominal_velocity": 1.0,
                "downwash": 2.0}
  },

  "world": [
    {"dimension": [-5, -5, 0.0, 5, 5, 2.5]} # Size of the world [x_min, y_min, z_min, x_max, y_max, z_max], [m]
  ],

  "agents": [
    # type: quadrotor type (crazyflie or default), cid: crazyflie id for experiment (Not equal to the agent id used in the planner and rviz)
    # start: start point of the agent [x,y,z] [m], goal: goal point of the agent [x,y,z] [m]
    {"type": "crazyflie", "cid": 1, "start": [4.0, 0.0, 1.0], "goal": [-4.0, 0.0, 1.0]},
    {"type": "crazyflie", "cid": 2, "start": [4.0, 2.0, 1.0], "goal": [-4.0, -2.0, 1.0]},
    {"type": "crazyflie", "cid": 3, "start": [4.0, 4.0, 1.0], "goal": [-4.0, -4.0, 1.0]},
    {"type": "crazyflie", "cid": 4, "start": [2.0, 4.0, 1.0], "goal": [-2.0, -4.0, 1.0]}
  ],

  "obstacles": [
  ]
}