# import rclpy
# from std_msgs.msg import String
# from flask import *
# from flask_cors import CORS
from google.cloud import pubsub_v1

#
# app = Flask(__name__)
# app.config["DEBUG"] = True
# CORS(app)
#
#
# @app.route('/api/command/', methods=['GET'])
# def get():
#     return "<h1>Distant Reading Archive</h1><p>This site is a prototype API for distant reading of science fiction novels.</p>"
#
#
# @app.route('/hi', methods=['POST'])
# def post():
#     print(request.data)
#     return "thanks!"

project_id = "backend-pub-sub"
subscription_id = "switch-sub"
timeout = 60

class Controller:
    def __init__(self):
        # rclpy.init()
        # self.node = rclpy.create_node(node_name="controller")
        # self.switch_pub = self.node.create_publisher(String, "switch-controller-topic", 10)
        self.subscriber = pubsub_v1.SubscriberClient()
        self.subscription_path = self.subscriber.subscription_path(project_id, subscription_id)
        self.streaming_pull_future = self.subscriber.subscribe(self.subscription_path, callback=self.callback)

        self.mainloop()

    def callback(self, message):
        print(f"Received {message}.")
        message.ack()

    def mainloop(self):
        i = 0
        # msg = String()
        while True:
            with self.subscriber:
                try:
                    # When `timeout` is not set, result() will block indefinitely,
                    # unless an exception is encountered first.
                    self.streaming_pull_future.result(timeout=timeout)
                except TimeoutError:
                    self.streaming_pull_future.cancel()
            # msg.data = 'Hello World: %d' % i
            # i += 1
            # self.node.get_logger().info('Publishing: "%s"' % msg.data)
            # self.switch_pub.publish(msg)
            # rclpy.spin_once(self.node, timeout_sec=1)
            # if i == 10:
            #     break
        # self.node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    controller = Controller()
    controller.mainloop()
    # app.run()
