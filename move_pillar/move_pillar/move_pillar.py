import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SpawnEntity, GetModelList, SetEntityState

from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = 'move_pillar'


class MovePillar(Node):
    def __init__(self):
        super().__init__(PACKAGE_NAME)

        self.spawn_model('unit_cylinder', 'pillar.sdf')

        # Creating subscriber for 2D goal pose
        self.create_subscription(PoseStamped, '/goal_pose', self.move_pillar_callback, 10)

    def spawn_model(self, model_name, model_filename):
        self.get_logger().warning(f'Trying to spawn model: {model_name}')
        # Check if model already exists
        if not self.check_model_exists(model_name):
            # Loading pillar model
            model = SpawnEntity.Request()
            model.name = model_name
            model.xml = open(f'{get_package_share_directory(PACKAGE_NAME)}/models/{model_filename}', 'r').read()

            # Creating a service client
            spawn_model_client = self.create_client(SpawnEntity, '/spawn_entity')

            while not spawn_model_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning('Service /spawn_entity not available, waiting again...')

            future = spawn_model_client.call_async(model)
            while not future.done():
                rclpy.spin_once(self)


            if future.result().success:
                self.get_logger().info('Model spawned')
            else:
                self.get_logger().warning(f'{future.result().status_message}')
        else:
            self.get_logger().warning('Model already existing')

    def check_model_exists(self, model_name):
        properties = GetModelList.Request()

        model_list = self.create_client(GetModelList, '/get_model_list')
        while not model_list.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Service /get_model_list not available, waiting again...')

        future = model_list.call_async(properties)
        while not future.done():
            rclpy.spin_once(self)

        return model_name in future.result().model_names

    def move_pillar_callback(self, msg):
        if msg.pose.position is None:
            return

        new_pillar_position = SetEntityState.Request()
        new_pillar_position.state.name = 'unit_cylinder'
        new_pillar_position.state.pose.position.x = msg.pose.position.x
        new_pillar_position.state.pose.position.y = msg.pose.position.y
        new_pillar_position.state.pose.position.z = 1.0

        entity_state = self.create_client(SetEntityState, '/set_entity_state')
        while not entity_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Service /set_entity_state not available, waiting again...')

        future = entity_state.call_async(new_pillar_position)

def main(args=None):
    rclpy.init(args=args)

    move_pillar = MovePillar()
    rclpy.spin(move_pillar)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
