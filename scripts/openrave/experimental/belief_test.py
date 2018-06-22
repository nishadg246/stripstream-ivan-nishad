def test_collisions(env, octree):
    # http://openrave.org/docs/0.8.0/openravepy/examples.collision/
    #options = CollisionOptions.Distance|CollisionOptions.Contacts # TODO: distance doesn't work with FCL but contacts do?
    options = CollisionOptions.Contacts
    checker = env.GetCollisionChecker()
    #with collision_saver(env, options):
    print checker.SetCollisionOptions(options)
    report = CollisionReport()
    print env.CheckCollision(octree, report)
    print len(report.contacts), report.minDistance, report.plink1, report.plink2 #, report.numCols
    for contact in report.contacts:
      print contact.depth, contact.norm, contact.pos
    return None


def base_test(robot):
    """
    world_from_base = robot.GetTransform()
    robot_z = point_from_trans(world_from_base)[2]
    robot_axis = rot_from_trans(world_from_base).dot([0, 0, 1])

    # https://github.com/rdiankov/openrave/blob/ff43549fb6db281c7bc9a85794b88c29d6522ab4/include/openrave/geometry.h
    # TODO: store the base explicitly as a transform to make this easier?
    def get_base_conf():
      quat = quat_from_pose(pose_from_trans(robot.GetTransform()))
      print quat
      #print angle_vector_from_quat(quat)
      base_theta = base_values_from_trans(robot.GetTransform(), axis=robot_axis)[2]
      quat_base = quat_from_angle_vector(base_theta, robot_axis)
      print quat_base
      print quat_dot(quat_inv(quat), quat_base)

      return Conf(base_values_from_trans(robot.GetTransform(), axis=robot_axis))
    def set_base_conf(q):
      x, y, theta = q
      print x, y, theta
      point = np.array([x, y, robot_z]) # TODO: ensure the point is correct given the rotation
      quat = quat_from_angle_vector(theta, robot_axis)
      set_pose(robot, pose_from_quat_point(quat, point))
    def set_active_base(indices=()):
      robot.SetActiveDOFs(indices, AFFINE_MASK, robot_axis)
    """