## Configuration of the Gazebo world

The simulation with the fin plugins have shown better results by configuring
the Gazebo's `.world` file with the following parameters for the physics engine:

```xml

<physics name="default_physics" default="true" type="ode">
  <max_step_size>0.01</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>100</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.2</sor>
    </solver>
  </ode>
</physics>

```

Check the [Mangalia world file](https://github.com/uuvsimulator/uuv_simulator/blob/master/uuv_gazebo_worlds/worlds/mangalia.world) to see an example.

## License

Riptide Gazebo package is inspired by the LAUV Gazebo package.
LAUV Gazebo package is open-sourced under the Apache-2.0 license. See the
[LICENSE](https://github.com/uuvsimulator/lauv_gazebo/blob/master/LICENSE) file for details.
