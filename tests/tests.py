import compas_fab

full_configuration = robot.inverse_kinematics(0,
                                                      0,
                                                      group=0,
                                                      avoid_collisions=0,
                                                      return_full_configuration=True,
                                                      )